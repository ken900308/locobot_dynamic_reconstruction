#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto Anchor from First PointCloud2

目的
- 監聽第一筆點雲的時間戳 t0
- 查詢 world(map) → camera 的 TF
- 組出 world(map) → mast3r_map 的 /tf_static

新增功能
- 支援「相機光學座標 → MASt3R 世界座標」的固定旋轉/平移修正（預設 Y 軸 180°）
- 以參數化方式開關「假設第一幀相機就是 MASt3R 原點」或「額外套用固定外參」

常用參數（--ros-args -p key:=value）：
- world_frame:            預設 "map"
- camera_frame:           預設 "camera_color_optical_frame"
- mast3r_frame:           預設 "mast3r_map"
- pointcloud_topic:       預設 "/mast3r/pointcloud"
- wait_timeout_sec:       預設 1.0
- fallback_to_latest:     預設 True
- latched_republish:      預設 True

# 關鍵新參數
- assume_camera_is_mast3r_origin: 預設 False
- mast3r_rot_corr_deg:            旋轉修正 [rx, ry, rz] (deg)  預設 [0.0, 180.0, 0.0]
- mast3r_trans_corr_m:            平移修正 [tx, ty, tz] (m)   預設 [0.0, 0.0, 0.0]
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros


def quat_normalize(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quat_from_euler_xyz_deg(rx_deg: float, ry_deg: float, rz_deg: float):
    """XYZ (roll, pitch, yaw) in degrees → quaternion (x,y,z,w)."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    # intrinsic XYZ
    qw = cx*cy*cz + sx*sy*sz
    qx = sx*cy*cz - cx*sy*sz
    qy = cx*sy*cz + sx*cy*sz
    qz = cx*cy*sz - sx*sy*cz
    return quat_normalize(qx, qy, qz, qw)


def quat_multiply(ax, ay, az, aw, bx, by, bz, bw):
    """Quaternion multiply q = A * B (x,y,z,w)."""
    qx = aw*bx + ax*bw + ay*bz - az*by
    qy = aw*by - ax*bz + ay*bw + az*bx
    qz = aw*bz + ax*by - ay*bx + az*bw
    qw = aw*bw - ax*bx - ay*by - az*bz
    return quat_normalize(qx, qy, qz, qw)


def rot_vec_by_quat(qx, qy, qz, qw, px, py, pz):
    """Rotate vector p by quaternion q (x,y,z,w)."""
    # Hamilton product optimized (q * p * q_conj)
    ix =  qw*px + qy*pz - qz*py
    iy =  qw*py + qz*px - qx*pz
    iz =  qw*pz + qx*py - qy*px
    iw = -qx*px - qy*py - qz*pz

    rx = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy)
    ry = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz)
    rz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx)
    return rx, ry, rz


def compose_tf(A: TransformStamped, B: TransformStamped, out_parent_frame: str, out_child_frame: str) -> TransformStamped:
    """
    組合變換：out = A ∘ B
    A: parent -> mid
    B: mid    -> child
    out: parent -> child
    平移：A.t + R(A)*B.t
    旋轉：A.q * B.q
    """
    # A rotation/translation
    ax, ay, az, aw = A.transform.rotation.x, A.transform.rotation.y, A.transform.rotation.z, A.transform.rotation.w
    atx, aty, atz = A.transform.translation.x, A.transform.translation.y, A.transform.translation.z
    # B rotation/translation
    bx, by, bz, bw = B.transform.rotation.x, B.transform.rotation.y, B.transform.rotation.z, B.transform.rotation.w
    btx, bty, btz = B.transform.translation.x, B.transform.translation.y, B.transform.translation.z

    # out rotation
    qx, qy, qz, qw = quat_multiply(ax, ay, az, aw, bx, by, bz, bw)
    # out translation
    rtx, rty, rtz = rot_vec_by_quat(ax, ay, az, aw, btx, bty, btz)
    rtx += atx
    rty += aty
    rtz += atz

    out = TransformStamped()
    out.header.frame_id = out_parent_frame
    out.child_frame_id = out_child_frame
    out.transform.translation.x = float(rtx)
    out.transform.translation.y = float(rty)
    out.transform.translation.z = float(rtz)
    out.transform.rotation.x = float(qx)
    out.transform.rotation.y = float(qy)
    out.transform.rotation.z = float(qz)
    out.transform.rotation.w = float(qw)
    return out


class AutoAnchor(Node):
    def __init__(self):
        super().__init__('auto_anchor_from_pointcloud')

        # ======== 基本參數 ========
        self.declare_parameter('world_frame', 'locobot/odom')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('mast3r_frame', 'mast3r_map')
        self.declare_parameter('wait_timeout_sec', 1.0)
        self.declare_parameter('fallback_to_latest', True)
        self.declare_parameter('prefer_latest_tf', True)
        self.declare_parameter('latched_republish', True)
        self.declare_parameter('pointcloud_topic', '/mast3r/frame_pointcloud')

        # ======== 新增：外參修正參數 ========
        # 是否沿用舊邏輯（map->mast3r_map = map->camera@t0）
        self.declare_parameter('assume_camera_is_mast3r_origin', False)

        # 相機→MASt3R 世界的固定旋轉（deg, XYZ），預設 Y=180°
        self.declare_parameter('mast3r_rot_corr_deg', [0.0, 180.0, 0.0])

        # 相機→MASt3R 世界的固定平移（m, XYZ），通常 0
        self.declare_parameter('mast3r_trans_corr_m', [0.0, 0.0, 0.0])

        # 取值
        self.world = self.get_parameter('world_frame').get_parameter_value().string_value
        self.camera = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.mast3r = self.get_parameter('mast3r_frame').get_parameter_value().string_value
        self.wait_timeout = float(self.get_parameter('wait_timeout_sec').value)
        self.fallback_latest = bool(self.get_parameter('fallback_to_latest').value)
        self.prefer_latest_tf = bool(self.get_parameter('prefer_latest_tf').value)
        self.latched_republish = bool(self.get_parameter('latched_republish').value)
        self.pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value

        self.assume_origin = bool(self.get_parameter('assume_camera_is_mast3r_origin').value)

        rot_corr = self.get_parameter('mast3r_rot_corr_deg').get_parameter_value().double_array_value
        self.rot_corr_deg = list(rot_corr) if len(rot_corr) == 3 else [0.0, 180.0, 0.0]

        trans_corr = self.get_parameter('mast3r_trans_corr_m').get_parameter_value().double_array_value
        self.trans_corr_m = list(trans_corr) if len(trans_corr) == 3 else [0.0, 0.0, 0.0]

        # TF I/O
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 訂閱第一筆點雲
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 匹配發布者的 RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pc_sub = self.create_subscription(PointCloud2, self.pc_topic, self.on_cloud, qos)

        self.anchored = False
        self.get_logger().info(
            f'[AutoAnchor] Wait first cloud on {self.pc_topic} → lookup {self.world}→{self.camera} @stamp '
            f'→ publish {self.world}→{self.mast3r} (/tf_static). '
            f'assume_origin={self.assume_origin}, rot_corr_deg={self.rot_corr_deg}, trans_corr_m={self.trans_corr_m}, '
            f'prefer_latest_tf={self.prefer_latest_tf}'
        )

        if self.latched_republish:
            self.create_timer(1.0, self.repub_if_ready)

    def repub_if_ready(self):
        if self.anchored and hasattr(self, 'last_tf'):
            self.static_broadcaster.sendTransform(self.last_tf)

    def on_cloud(self, msg: PointCloud2):
        if self.anchored:
            return

        stamp = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
        self.get_logger().info(
            f'[AutoAnchor] First cloud stamp = {stamp.nanoseconds/1e9:.9f}s ({msg.header.stamp.sec}+{msg.header.stamp.nanosec}ns)'
        )

        # 重試機制：TF buffer 可能需要時間填充
        max_retries = 5
        retry_delay = 1.0  # 秒
        T_map_cam = None

        def lookup_latest():
            return self.tf_buffer.lookup_transform(
                self.world, self.camera, rclpy.time.Time(), timeout=Duration(seconds=self.wait_timeout)
            )

        def lookup_at_stamp():
            return self.tf_buffer.lookup_transform(
                self.world, self.camera, stamp, timeout=Duration(seconds=self.wait_timeout)
            )
        
        for attempt in range(max_retries):
            # 先嘗試最新 TF（避免 rosbridge 延遲造成 future 時間）
            if self.prefer_latest_tf:
                try:
                    T_map_cam = lookup_latest()
                    self.get_logger().info('[AutoAnchor] lookup latest OK')
                    break
                except Exception as e:
                    self.get_logger().warn(f'[AutoAnchor] lookup latest failed (attempt {attempt+1}/{max_retries}): {repr(e)}')

            # 再嘗試精確時間戳
            try:
                T_map_cam = lookup_at_stamp()
                self.get_logger().info('[AutoAnchor] lookup @t0 OK')
                break
            except Exception as e:
                self.get_logger().warn(f'[AutoAnchor] lookup @t0 failed (attempt {attempt+1}/{max_retries}): {repr(e)}')
                
                # 嘗試最新的 TF
                if self.fallback_latest and not self.prefer_latest_tf:
                    try:
                        T_map_cam = lookup_latest()
                        self.get_logger().info('[AutoAnchor] fallback to latest OK')
                        break
                    except Exception as e2:
                        self.get_logger().warn(f'[AutoAnchor] fallback latest failed (attempt {attempt+1}/{max_retries}): {repr(e2)}')
                
                # 等待後重試
                if attempt < max_retries - 1:
                    self.get_logger().info(f'[AutoAnchor] Waiting {retry_delay}s for TF buffer to fill...')
                    import time
                    time.sleep(retry_delay)
        
        if T_map_cam is None:
            self.get_logger().error(f'[AutoAnchor] Failed to get TF after {max_retries} attempts. Give up.')
            return

        if self.assume_origin:
            # 舊邏輯：map→mast3r_map 直接等於 map→camera
            map_to_mast3r = TransformStamped()
            map_to_mast3r.header.frame_id = self.world
            map_to_mast3r.child_frame_id = self.mast3r
            map_to_mast3r.transform.translation.x = T_map_cam.transform.translation.x
            map_to_mast3r.transform.translation.y = T_map_cam.transform.translation.y
            map_to_mast3r.transform.translation.z = T_map_cam.transform.translation.z
            map_to_mast3r.transform.rotation = T_map_cam.transform.rotation
        else:
            # 新邏輯：map→mast3r_map = (map→camera) ∘ (camera→mast3r_map固定外參)
            # camera→mast3r_map 固定外參（旋轉+平移）
            qx, qy, qz, qw = quat_from_euler_xyz_deg(*self.rot_corr_deg)
            corr = TransformStamped()
            corr.header.frame_id = self.camera
            corr.child_frame_id = self.mast3r  # 中繼用，compose 時會覆寫
            corr.transform.translation.x = float(self.trans_corr_m[0])
            corr.transform.translation.y = float(self.trans_corr_m[1])
            corr.transform.translation.z = float(self.trans_corr_m[2])
            corr.transform.rotation.x = float(qx)
            corr.transform.rotation.y = float(qy)
            corr.transform.rotation.z = float(qz)
            corr.transform.rotation.w = float(qw)

            map_to_mast3r = compose_tf(T_map_cam, corr, self.world, self.mast3r)

        # 正規化四元數
        q = map_to_mast3r.transform.rotation
        qx, qy, qz, qw = quat_normalize(q.x, q.y, q.z, q.w)
        map_to_mast3r.transform.rotation.x = qx
        map_to_mast3r.transform.rotation.y = qy
        map_to_mast3r.transform.rotation.z = qz
        map_to_mast3r.transform.rotation.w = qw

        # 發 /tf_static
        self.static_broadcaster.sendTransform(map_to_mast3r)
        self.last_tf = map_to_mast3r
        self.anchored = True

        t = map_to_mast3r.transform.translation
        self.get_logger().info(
            f'[AutoAnchor] PUBLISHED static TF  {self.world} → {self.mast3r}  '
            f't=({t.x:.3f},{t.y:.3f},{t.z:.3f})  '
            f'q=({qx:.5f},{qy:.5f},{qz:.5f},{qw:.5f})'
        )

    def republish_once(self):
        if hasattr(self, 'last_tf'):
            self.static_broadcaster.sendTransform(self.last_tf)


def main():
    rclpy.init()
    node = AutoAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
