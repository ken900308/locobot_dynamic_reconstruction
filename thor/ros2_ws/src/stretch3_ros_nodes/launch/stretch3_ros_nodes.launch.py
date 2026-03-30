#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stretch3_ros_nodes',
            executable='ipc_bridge_node',
            name='ipc_bridge_node',
            output='screen',
        ),
        Node(
            package='stretch3_ros_nodes',
            executable='ipc_pointcloud_receiver',
            name='ipc_pointcloud_receiver',
            output='screen',
            parameters=[{
                'socket_path': '/tmp/ipc_socket/mast3r_pointcloud.sock',
                'output_topic': '/mast3r/frame_pointcloud',
                'frame_id': 'mast3r_map',
                'qos_reliable': True,
            }],
        ),
        Node(
            package='stretch3_ros_nodes',
            executable='auto_anchor_from_pointcloud_stretch3',
            name='auto_anchor_from_pointcloud_stretch3',
            output='screen',
            parameters=[{
                'world_frame': 'map',
                'camera_frame': 'camera_color_optical_frame',
                'mast3r_frame': 'mast3r_map',
                'fallback_to_latest': True,
            }],
        ),
        Node(
            package='stretch3_ros_nodes',
            executable='pc2_to_map_stretch3',
            name='pc2_to_map_stretch3',
            output='screen',
            parameters=[{
                'input_topic': '/mast3r/frame_pointcloud',
                'output_topic': '/mast3r/pointcloud_in_map',
                'cloud_frame': 'mast3r_map',
                'world_frame': 'map',
            }],
        ),
    ])
