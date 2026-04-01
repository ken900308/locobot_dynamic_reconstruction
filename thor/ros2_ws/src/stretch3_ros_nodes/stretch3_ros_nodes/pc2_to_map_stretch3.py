import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
import numpy as np

# 先嘗試使用官方的 do_transform_cloud；失敗就走 fallback
USE_TF2_SENSOR = True
try:
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
except Exception:
    USE_TF2_SENSOR = False
    from sensor_msgs_py import point_cloud2 as pc2

def transform_from_tf(T):
    t = T.transform.translation
    q = T.transform.rotation
    # 四元數 -> 旋轉矩陣
    x,y,z,w = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w),  2*(x*z + y*w)],
        [2*(x*y + z*w),  1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),  2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float32)
    tvec = np.array([t.x, t.y, t.z], dtype=np.float32)
    return R, tvec

def do_transform_cloud_fallback(msg_in, T):
    """
    僅支援包含 x,y,z,(rgb) 的 PointCloud2，將點雲用 T 轉到新座標。
    - 會保留 header.stamp（caller 再把 frame_id 改成目標）
    - rgb 以 UINT32 寫回（保持原始 4 bytes，不失真）
    """
    import struct
    import numpy as np
    from sensor_msgs_py import point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2, PointField

    # 取得旋轉/平移
    R, t = transform_from_tf(T)        # R: (3,3), t: (1,3) or (3,)

    # 解析欄位
    fields_dict = {f.name: f for f in msg_in.fields}
    has_rgb = 'rgb' in fields_dict

    # 讀取點資料
    gen = pc2.read_points(
        msg_in,
        field_names=('x', 'y', 'z', 'rgb') if has_rgb else ('x', 'y', 'z'),
        skip_nans=True
    )
    pts, colors = [], []
    for p in gen:
        if has_rgb:
            x, y, z, rgb = p
            colors.append(rgb)   # 可能是 float32（packed）或 uint32
        else:
            x, y, z = p
        pts.append((x, y, z))

    # 空雲：直接回傳空的（沿用 header）
    if not pts:
        out_empty = PointCloud2()
        out_empty.header = msg_in.header
        return out_empty

    # 幾何轉換
    P = np.asarray(pts, dtype=np.float32)     # (N,3)
    P_out = (P @ R.T) + t                     # 右乘 R^T 等價左乘 R

    # 準備輸出 fields（rgb 一定用 UINT32）
    out_fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    data = []

    if has_rgb:
        out_fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))

        # 依輸入 endianness 決定 pack/unpack 方向（預設 ROS 多為 little-endian）
        fmt_f = '<f' if not msg_in.is_bigendian else '>f'
        fmt_I = '<I' if not msg_in.is_bigendian else '>I'

        for i in range(P_out.shape[0]):
            rgb_val = colors[i]
            # 如果 read_points 回來是 float（常見：packed RGB 存成 float32）
            if isinstance(rgb_val, (float, np.floating)):
                rgb_bytes = struct.pack(fmt_f, float(rgb_val))  # 保留原 4 bytes
                rgb_uint  = struct.unpack(fmt_I, rgb_bytes)[0]
            else:
                # 已是 int/np.uint32，就取低 32 位
                rgb_uint = int(rgb_val) & 0xFFFFFFFF

            data.append((
                float(P_out[i, 0]), float(P_out[i, 1]), float(P_out[i, 2]), rgb_uint
            ))
    else:
        # 無 rgb
        for i in range(P_out.shape[0]):
            data.append((
                float(P_out[i, 0]), float(P_out[i, 1]), float(P_out[i, 2])
            ))

    # 建立輸出雲（沿用 stamp，caller 外部再把 frame_id 改目標）
    header = msg_in.header
    out_msg = pc2.create_cloud(header, out_fields, data)
    # create_cloud 會以 little-endian 建立；若你真的需要 big-endian，可自行改 out_msg.is_bigendian
    out_msg.is_bigendian = False

    # 點步長/列步長由 fields 自動計算（x,y,z,rgb → 16 bytes）
    return out_msg


class Pc2ToMap(Node):
    def __init__(self):
        super().__init__('pc2_to_map')
        self.declare_parameter('input_topic', '/mast3r/frame_pointcloud')  # 與 auto_anchor 訂閱同一 topic
        self.declare_parameter('output_topic', '/mast3r/pointcloud_in_map')  # 對應輸出
        self.declare_parameter('cloud_frame', 'mast3r_map')
        self.declare_parameter('world_frame', 'locobot/odom')
        self.declare_parameter('lookup_timeout_sec', 0.5)
        self.declare_parameter('qos_best_effort', False)  # 使用 RELIABLE QoS
        self.declare_parameter('max_publish_rate', 0.0)   # 0 = 無限制 (disabled)
        self.declare_parameter('wait_for_tf_at_startup', True)  # 新增：啟動時等待 TF 可用
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.cloud_frame = self.get_parameter('cloud_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.lookup_timeout = float(self.get_parameter('lookup_timeout_sec').value)
        best_effort = bool(self.get_parameter('qos_best_effort').value)
        self.max_rate = float(self.get_parameter('max_publish_rate').value)
        self.wait_for_tf = bool(self.get_parameter('wait_for_tf_at_startup').value)
        self.declare_parameter('force_cloud_frame', True)
        self.force_cloud_frame = bool(self.get_parameter('force_cloud_frame').value)

        # TF 就緒狀態
        self.tf_ready = False

        # FPS 控制
        self.min_interval = 1.0 / self.max_rate if self.max_rate > 0 else 0.0
        self.last_publish_time = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50  # 提高 depth 避免 chunk burst 時掉包
        )
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        
        # 定時器：持續檢查 TF 是否就緒（適用於點雲發布頻率低的情況）
        if self.wait_for_tf:
            self.tf_check_timer = self.create_timer(0.5, self.check_tf_ready)
        
        self.get_logger().info(f'[Pc2ToMap] {self.input_topic}({self.cloud_frame}) → '
                               f'{self.output_topic}({self.world_frame}); '
                               f'backend={"tf2_sensor_msgs" if USE_TF2_SENSOR else "python-fallback"}')

    def check_tf_ready(self):
        """定時器回呼：檢查 TF 是否就緒"""
        if self.tf_ready:
            return  # 已就緒，不再檢查
        
        src = self.cloud_frame
        try:
            self.tf_buffer.lookup_transform(
                self.world_frame, src,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            self.tf_ready = True
            self.get_logger().info(f'[Pc2ToMap] TF ready: {self.world_frame} <- {src}. Ready to process clouds.')
            # 停止定時器
            self.tf_check_timer.cancel()
        except Exception:
            pass  # 繼續等待

    def on_cloud(self, msg):
        # DEBUG: 確認 callback 有被觸發
        if not hasattr(self, '_debug_first_cloud'):
            self._debug_first_cloud = True
            self.get_logger().info(f'[Pc2ToMap] DEBUG: First cloud received! tf_ready={self.tf_ready}')

        # FPS 限制：檢查是否超過最小間隔
        import time
        current_time = time.time()
        if self.min_interval > 0:
            elapsed = current_time - self.last_publish_time
            if elapsed < self.min_interval:
                # 跳過此幀，避免過載
                return

        src = self.cloud_frame if self.force_cloud_frame else (msg.header.frame_id or self.cloud_frame)

        # 如果 TF 還沒就緒，先檢查是否可用（不重試，不報錯）
        if not self.tf_ready and self.wait_for_tf:
            try:
                self.tf_buffer.lookup_transform(
                    self.world_frame, src, 
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)  # 快速檢查
                )
                self.tf_ready = True
                self.get_logger().info(f'[Pc2ToMap] TF ready: {self.world_frame} <- {src}. Start processing.')
            except Exception as e:
                # DEBUG: 顯示為什麼 TF 還沒就緒（只印一次）
                if not hasattr(self, '_debug_tf_waiting'):
                    self._debug_tf_waiting = True
                    self.get_logger().info(f'[Pc2ToMap] Waiting for TF {self.world_frame} <- {src}... {repr(e)}')
                return

        # TF 已就緒，正常 lookup（短 timeout，失敗就報警）
        T = None
        try:
            T = self.tf_buffer.lookup_transform(
                self.world_frame, src, 
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout)
            )
        except Exception as e:
            self.get_logger().warn(f'[Pc2ToMap] TF lookup failed: {self.world_frame} <- {src}. Error: {repr(e)}')
            return

        if USE_TF2_SENSOR:
            out = do_transform_cloud(msg, T)
        else:
            out = do_transform_cloud_fallback(msg, T)

        # [NEW HYBRID FRAMEWORK] 
        # Pack the original kf_id and the used T_init into the frame_id
        # Format: "kf_42|tx:0.1|ty:0.5|tz:0.2|qx:0|qy:0|qz:0|qw:1"
        original_kf_id = msg.header.frame_id  # e.g. "kf_42"
        tx = T.transform.translation.x
        ty = T.transform.translation.y
        tz = T.transform.translation.z
        qx = T.transform.rotation.x
        qy = T.transform.rotation.y
        qz = T.transform.rotation.z
        qw = T.transform.rotation.w
        
        packed_frame_id = f"{original_kf_id}|tx:{tx:.6f}|ty:{ty:.6f}|tz:{tz:.6f}|qx:{qx:.6f}|qy:{qy:.6f}|qz:{qz:.6f}|qw:{qw:.6f}"
        out.header.frame_id = packed_frame_id
        
        self.pub.publish(out)

        # 更新最後發布時間
        self.last_publish_time = current_time

def main():
    rclpy.init()
    node = Pc2ToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
