using System;
using System.Collections;
using System.Collections.Concurrent;
using UnityEngine;
using UnityEngine.Rendering;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Matrix4x4 = UnityEngine.Matrix4x4;

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

// ==============================================================================
//  PointCloudAccumulatorGPU_Anchored_IBGR
//  - Incremental Build, Global Redraw (IBGR) 架構
//  - 通道A (Incremental): 接收單一 Keyframe (pointcloud_in_map)，漸進式附加到畫面上 (滑順體驗)
//  - 通道B (Global Redraw): 接收完整地圖 (pointcloud_full_map)，並在背後準備好。
//                           一旦收到全量更新，瞬間 Switch (Double Buffering) 覆蓋畫面！
// ==============================================================================

[RequireComponent(typeof(RosConnector))]
public class PointCloudAccumulatorGPU_Anchored_IBGR : MonoBehaviour
{
    [Header("ROS Topics (IBGR Architecture)")]
    [Tooltip("單一 Topic，Unity 將透過偵測 frame_id 內的 kf_999999 來自動觸發全域重繪。")]
    public string pointcloudTopic = "/mast3r/pointcloud_in_map";

    [Header("Memory (Full Map)")]
    [Tooltip("Maximum points across the ENTIRE map (safety limit).")]
    public int maxGlobalPoints = 5_000_000;

    [Header("World Adjust (Draw-Time)")]
    [Tooltip("Apply an additional overall transform matrix to the entire point cloud.")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Debug")]
    public bool showStats = true;

    // ---- ROS ----
    private RosConnector rosConnector;
    private string subIdPointCloud;

    // ---- Batches (IPC Decoding to Main Thread) ----
    private struct Batch
    {
        public bool isFullMap;    // true = 舊式非分塊全域更新（相容用）
        public bool isChunk;      // true = 分塊全域地圖片段
        public bool isFirstChunk; // true = 本批次第一塊（應重置 Back Buffer）
        public bool isLastChunk;  // true = 本批次最後一塊（應觸發 Swap）
        public Vector3[] pos;
        public uint[] col;
        public int count;
    }
    private readonly ConcurrentQueue<Batch> batchQueue = new ConcurrentQueue<Batch>();

    // ---- Double Buffering GPU Memory ----
    // 0 = Buffer A, 1 = Buffer B
    private ComputeBuffer[] posBuffers = new ComputeBuffer[2];
    private ComputeBuffer[] colBuffers = new ComputeBuffer[2];
    private int[] bufferCounts = new int[2];

    private int activeBufferIdx = 0; // Displayed on screen
    private int backBufferIdx = 1;   // Background writing (for Full Map Redraw)

    // 為了相容原本的 PointCloudGPU.shader，準備兩個 Dummy Buffer
    private ComputeBuffer dummyIdxBuffer;
    private ComputeBuffer dummyPoseBuffer;

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();

        // Setup Shader
        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh == null) sh = Shader.Find("Hidden/Point Cloud GPU");
            if (sh != null) pointMaterial = new Material(sh);
        }

        // Initialize Double Buffers
        for (int i = 0; i < 2; i++)
        {
            posBuffers[i] = new ComputeBuffer(maxGlobalPoints, 12);
            colBuffers[i] = new ComputeBuffer(maxGlobalPoints, 4);
            bufferCounts[i] = 0;
        }

        // Initialize Dummy Buffers to satisfy the existing shader
        dummyIdxBuffer = new ComputeBuffer(maxGlobalPoints, 4);
        uint[] zeroIndices = new uint[maxGlobalPoints]; // All set to 0
        dummyIdxBuffer.SetData(zeroIndices);

        dummyPoseBuffer = new ComputeBuffer(1, 64);
        Matrix4x4[] identities = new Matrix4x4[] { Matrix4x4.identity };
        dummyPoseBuffer.SetData(identities);
    }

    void Start()
    {
        subIdPointCloud = rosConnector.RosSocket.Subscribe<PointCloud2>(pointcloudTopic, ReceivePointCloud);
        Debug.Log($"[IBGR] Subscribed to {pointcloudTopic}");
    }

    // ---- Chunked Full Map State ----
    // 記錄目前正在累積的分塊狀態
    private int _chunkExpectedTotal = 0;
    private int _chunkReceivedSoFar = 0;

    /// <summary>
    /// 解析 frame_id 中的分塊資訊。
    /// 支援 ROS 模式格式: kf_999999_N_T
    /// 也支援 IPC 模式格式: kf_<encodedId>|tx:... (encodedId = (chunkN << 16) | totalChunks)
    /// </summary>
    private (int chunkN, int totalChunks) ParseChunkFrameId(string frameId)
    {
        // Step 1: 先去掉 '|' 後面的 pose suffix
        string core = frameId;
        int pipeIdx = core.IndexOf('|');
        if (pipeIdx >= 0) core = core.Substring(0, pipeIdx); // e.g. "kf_65542" 或 "kf_999999_1_6"

        // 確認前綴
        if (!core.StartsWith("kf_")) return (0, 0);

        string remainder = core.Substring(3); // 去掉 "kf_"

        // 1. 嘗試解析 ROS 舊版格式 "999999_N_T"
        if (remainder.StartsWith("999999_"))
        {
            var parts = remainder.Substring(7).Split('_');
            if (parts.Length == 2 && int.TryParse(parts[0], out int n) && int.TryParse(parts[1], out int t))
            {
                if (n >= 1 && t >= 2 && n <= t && t < 1024) return (n, t);
            }
        }

        // 2. 嘗試解析 IPC 位元打包格式 (encodedId = (N << 16) | T)
        if (int.TryParse(remainder, out int encodedId))
        {
            // Step 2 & 3: 如果 ID 夠大，代表它是打包過的 chunk id
            if (encodedId > 65535) // 1 << 16
            {
                int chunkN = encodedId >> 16;
                int totalChunks = encodedId & 0xFFFF;

                // Step 4: 合理性檢查，避免誤判
                if (chunkN >= 1 && totalChunks >= 2 && chunkN <= totalChunks && totalChunks < 1024)
                {
                    return (chunkN, totalChunks);
                }
            }
        }

        // 不符合 Chunk 條件
        return (0, 0);
    }

    private void ReceivePointCloud(PointCloud2 msg)
    {
        var frameId = msg.header.frame_id;

        // 直接透過 Parse 統一處理，不一定要包含 "kf_999999_" 才能解析
        var (chunkN, totalChunks) = ParseChunkFrameId(frameId);
        if (chunkN > 0 && totalChunks > 0)
        {
            Debug.Log($"[IBGR] RX Chunk | chunk={chunkN}/{totalChunks} | raw={frameId}");
            DecodeChunk(msg, chunkN, totalChunks);
            return;
        }

        // Step 5: 如果不符合 chunk 條件，就當作普通 incremental pointcloud
        // (這裡兼任了舊有非分塊 PGO "kf_999999" 的相容性判斷)
        if (frameId.Contains("kf_999999"))
        {
            DecodePointCloud(msg, isFullMap: true);
            return;
        }

        // 一般 Incremental Keyframe (例如 kf_31)
        DecodePointCloud(msg, isFullMap: false);
    }

    /// <summary>解碼一個分塊，累積到 Back Buffer；最後一塊觸發 Buffer Swap。</summary>
    private void DecodeChunk(PointCloud2 msg, int chunkN, int totalChunks)
    {
        int pointCount = (int)(msg.width * msg.height);
        if (pointCount == 0) return;

        Vector3[] pArr = new Vector3[pointCount];
        uint[] cArr = new uint[pointCount];

        int xOff = -1, yOff = -1, zOff = -1, rgbOff = -1;
        foreach (var field in msg.fields)
        {
            if (field.name == "x") xOff = (int)field.offset;
            else if (field.name == "y") yOff = (int)field.offset;
            else if (field.name == "z") zOff = (int)field.offset;
            else if (field.name == "rgb" || field.name == "rgba") rgbOff = (int)field.offset;
        }
        int step = (int)msg.point_step;
        byte[] data = msg.data;
        for (int i = 0; i < pointCount; i++)
        {
            int b = i * step;
            pArr[i] = new Vector3(
                BitConverter.ToSingle(data, b + xOff),
                BitConverter.ToSingle(data, b + yOff),
                BitConverter.ToSingle(data, b + zOff));
            cArr[i] = (rgbOff != -1) ? BitConverter.ToUInt32(data, b + rgbOff) : 0xFFFFFFFF;
        }

        batchQueue.Enqueue(new Batch
        {
            isFullMap = false,
            isChunk = true,
            isFirstChunk = (chunkN == 1),
            isLastChunk = (chunkN == totalChunks),
            pos = pArr,
            col = cArr,
            count = pointCount
        });
    }

    /// <summary>舊式增量或非分塊全域地圖的解碼（相容性保留）</summary>
    private void DecodePointCloud(PointCloud2 msg, bool isFullMap)
    {
        int pointCount = (int)(msg.width * msg.height);
        if (pointCount == 0) return;
        if (pointCount > maxGlobalPoints)
        {
            Debug.LogWarning($"[IBGR] Points ({pointCount}) > maxGlobalPoints. Truncating.");
            pointCount = maxGlobalPoints;
        }

        Vector3[] pArr = new Vector3[pointCount];
        uint[] cArr = new uint[pointCount];

        int xOff = -1, yOff = -1, zOff = -1, rgbOff = -1;
        foreach (var field in msg.fields)
        {
            if (field.name == "x") xOff = (int)field.offset;
            else if (field.name == "y") yOff = (int)field.offset;
            else if (field.name == "z") zOff = (int)field.offset;
            else if (field.name == "rgb" || field.name == "rgba") rgbOff = (int)field.offset;
        }
        int step = (int)msg.point_step;
        byte[] data = msg.data;
        for (int i = 0; i < pointCount; i++)
        {
            int b = i * step;
            pArr[i] = new Vector3(
                BitConverter.ToSingle(data, b + xOff),
                BitConverter.ToSingle(data, b + yOff),
                BitConverter.ToSingle(data, b + zOff));
            cArr[i] = (rgbOff != -1) ? BitConverter.ToUInt32(data, b + rgbOff) : 0xFFFFFFFF;
        }
        batchQueue.Enqueue(new Batch { isFullMap = isFullMap, isChunk = false, pos = pArr, col = cArr, count = pointCount });
    }

    void Update()
    {
        while (batchQueue.TryDequeue(out Batch b))
        {
            if (b.isChunk)
            {
                // =====================================================
                // PGO Chunked Full Map：在後台 Back Buffer 靜默累積，
                // 全部 25/25 chunk 到齊後 Atomic Swap，畫面無空窗。
                // =====================================================
                if (b.isFirstChunk)
                {
                    bufferCounts[backBufferIdx] = 0;
                    Debug.Log($"[IBGR] ▶ PGO Chunk START: Silently accumulating into back buffer (active stays visible).");
                }

                int backCount = bufferCounts[backBufferIdx];
                int insertCount = Mathf.Min(b.count, maxGlobalPoints - backCount);
                if (insertCount > 0)
                {
                    posBuffers[backBufferIdx].SetData(b.pos, 0, backCount, insertCount);
                    colBuffers[backBufferIdx].SetData(b.col, 0, backCount, insertCount);
                    bufferCounts[backBufferIdx] = backCount + insertCount;
                }

                if (b.isLastChunk)
                {
                    // 全部 chunk 到齊 → 原子性翻轉！畫面只閃一幀
                    int temp = activeBufferIdx;
                    activeBufferIdx = backBufferIdx;
                    backBufferIdx = temp;

                    // 清空舊的 Active（現在變成 Back），等下次 PGO 使用
                    bufferCounts[backBufferIdx] = 0;

                    Debug.Log($"[IBGR] ⚡ DB Swap! Displaying {bufferCounts[activeBufferIdx]} PGO-aligned pts. Back buffer cleared for next cycle.");
                }
            }
            else if (b.isFullMap)
            {
                // =====================================================
                // 通道 B 舊式：一次性覆寫切換（相容用，理論上不再觸發）
                // =====================================================
                posBuffers[backBufferIdx].SetData(b.pos, 0, 0, b.count);
                colBuffers[backBufferIdx].SetData(b.col, 0, 0, b.count);
                bufferCounts[backBufferIdx] = b.count;

                int temp = activeBufferIdx;
                activeBufferIdx = backBufferIdx;
                backBufferIdx = temp;

                Debug.Log($"[IBGR] ⚡ Full Map DB Swap! Displaying {b.count} points perfectly aligned by PGO.");
            }
            else
            {
                // =====================================================
                // 通道 A：收到單幀點雲 (平時走路時漸進累加)
                // =====================================================
                int curCount = bufferCounts[activeBufferIdx];

                // 確認不會爆掉記憶體上限
                int insertCount = Mathf.Min(b.count, maxGlobalPoints - curCount);
                if (insertCount <= 0) continue;

                // 利用 ComputeBuffer.SetData 的偏移量功能，直接將新的點「附加 (Append)」在現有陣列的尾巴！
                posBuffers[activeBufferIdx].SetData(b.pos, 0, curCount, insertCount);
                colBuffers[activeBufferIdx].SetData(b.col, 0, curCount, insertCount);

                bufferCounts[activeBufferIdx] = curCount + insertCount;

                // (註：為了保險，我們也把相同的點附加到 backBufferIdx 上，這樣下次切換時才不會遺失剛收到的單幀，
                //  不過既然 PGO 切換通常伴隨 Full Map 重傳，這裡不同步也無傷大雅。)
            }
        }

        DrawActiveBuffer();
    }

    private void DrawActiveBuffer()
    {
        if (pointMaterial == null) return;
        int activeCount = bufferCounts[activeBufferIdx];
        if (activeCount == 0) return;

        // 計算 World Adjust
        Matrix4x4 adjust = (useWorldAdjust && worldAdjust != null)
                            ? worldAdjust.localToWorldMatrix
                            : Matrix4x4.identity;

        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);

        // 綁定真實的前景資料 (對應 Shader 內的緩衝區變數名稱)
        pointMaterial.SetBuffer("_Positions", posBuffers[activeBufferIdx]);
        pointMaterial.SetBuffer("_Colors", colBuffers[activeBufferIdx]);

        // 綁定虛假的 Dummy 陣列 (騙過 Shader 讓它做偏移為 0 的計算)
        pointMaterial.SetBuffer("_KeyframeIndices", dummyIdxBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", dummyPoseBuffer);

        // 繪製
        Graphics.DrawProcedural(pointMaterial,
                                new Bounds(Vector3.zero, Vector3.one * 10000f),
                                MeshTopology.Points,
                                activeCount, 1);
    }

    void OnGUI()
    {
        if (!showStats) return;
        GUI.color = Color.cyan;
        GUILayout.Label($"[IBGR Hybrid System] FPS: {1.0f / Time.smoothDeltaTime:0}");
        GUILayout.Label($"Displayed Points: {bufferCounts[activeBufferIdx]:N0} / {maxGlobalPoints}");
        GUILayout.Label($"Active GPU Buffer: {(activeBufferIdx == 0 ? "A" : "B")}");
    }

    void OnDestroy()
    {
        if (rosConnector != null && rosConnector.RosSocket != null)
        {
            if (subIdPointCloud != null) rosConnector.RosSocket.Unsubscribe(subIdPointCloud);
        }

        for (int i = 0; i < 2; i++)
        {
            posBuffers[i]?.Release();
            colBuffers[i]?.Release();
        }
        dummyIdxBuffer?.Release();
        dummyPoseBuffer?.Release();
    }
}
