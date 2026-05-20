from typing import Dict

from sensor_msgs.msg import PointCloud2

from stretch3_ros_nodes.keyframe_cloud_store import parse_keyframe_frame_id
from stretch3_ros_nodes.optimized_pose_types import OptimizedKeyframePose
from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey


class OptimizedMapStore:
    def __init__(self):
        self.clouds: Dict[KeyframeKey, PointCloud2] = {}
        self.poses: Dict[KeyframeKey, OptimizedKeyframePose] = {}

    def upsert_cloud(self, msg: PointCloud2) -> KeyframeKey:
        key = parse_keyframe_frame_id(msg.header.frame_id)
        self.clouds[key] = msg
        return key

    def upsert_pose(self, pose: OptimizedKeyframePose) -> KeyframeKey:
        self.poses[pose.key] = pose
        return pose.key

    def ready_keys(self) -> list[KeyframeKey]:
        return sorted(set(self.clouds).intersection(self.poses))

    def has_ready(self, key: KeyframeKey) -> bool:
        return key in self.clouds and key in self.poses

    def cloud(self, key: KeyframeKey) -> PointCloud2:
        return self.clouds[key]

    def pose(self, key: KeyframeKey) -> OptimizedKeyframePose:
        return self.poses[key]
