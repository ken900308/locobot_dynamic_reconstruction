#!/usr/bin/env python3
"""Republish /tf_static periodically for rosbridge clients.

Run this on the robot side, where /tf_static is complete. It subscribes with
TRANSIENT_LOCAL durability, caches every static transform, then republishes the
cache on a normal volatile topic so rosbridge clients that connect late can still
receive the full static tree.
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def key_for_transform(transform):
    return (transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/'))


class TfStaticRelay(Node):
    def __init__(self, input_topic, output_topic, rate_hz):
        super().__init__('tf_static_relay_for_rosbridge')
        self.cache = {}

        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(TFMessage, input_topic, self.on_static_tf, static_qos)
        self.pub = self.create_publisher(TFMessage, output_topic, volatile_qos)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_cache)
        self.get_logger().info(f'Relaying {input_topic} -> {output_topic} at {rate_hz:.2f} Hz')

    def on_static_tf(self, msg):
        updated = 0
        for transform in msg.transforms:
            key = key_for_transform(transform)
            self.cache[key] = transform
            updated += 1
        if updated:
            self.get_logger().info(f'Cached {len(self.cache)} static TF edges')
            self.publish_cache()

    def publish_cache(self):
        if not self.cache:
            return
        msg = TFMessage()
        msg.transforms = list(self.cache.values())
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-topic', default='/tf_static')
    parser.add_argument('--output-topic', default='/locobot/tf_static_relay')
    parser.add_argument('--rate', type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = TfStaticRelay(args.input_topic, args.output_topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
