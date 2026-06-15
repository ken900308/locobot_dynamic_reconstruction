import base64
from pathlib import Path
import time
from threading import RLock
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage

from image_bw_test.bandwidth_tracker import BandwidthTracker, TopicBandwidth


DEFAULT_RAW_TOPICS = [
    "/camera_left/camera_left/color/image_raw/compressed",
    "/camera_top/camera_top/color/image_raw/compressed",
    "/camera_right/camera_right/color/image_raw/compressed",
]
DEFAULT_RAW_TOPICS_PARAM = ",".join(DEFAULT_RAW_TOPICS)
DEFAULT_FUSION_TOPIC = "/stretch3/camera/camera/color/image_raw/compressed/fusion"


class CompressedImageBandwidthTestNode(Node):
    def __init__(self) -> None:
        super().__init__("compressed_image_bw_test_node")

        self.declare_parameter("raw_topics", DEFAULT_RAW_TOPICS_PARAM)
        self.declare_parameter("fusion_topic", DEFAULT_FUSION_TOPIC)
        self.declare_parameter("window_sec", 5.0)
        self.declare_parameter("report_period_sec", 1.0)
        self.declare_parameter("use_rosbridge", False)
        self.declare_parameter("rosbridge_host", "192.168.0.60")
        self.declare_parameter("rosbridge_port", 9090)
        self.declare_parameter("rosbridge_connect_timeout_sec", 5.0)
        self.declare_parameter("log_path", "")

        self._raw_topics = self._get_string_array_parameter("raw_topics")
        self._fusion_topic = self.get_parameter("fusion_topic").value
        self._window_sec = float(self.get_parameter("window_sec").value)
        report_period_sec = float(self.get_parameter("report_period_sec").value)
        self._use_rosbridge = bool(self.get_parameter("use_rosbridge").value)
        self._rosbridge_host = str(self.get_parameter("rosbridge_host").value)
        self._rosbridge_port = int(self.get_parameter("rosbridge_port").value)
        self._rosbridge_connect_timeout_sec = float(
            self.get_parameter("rosbridge_connect_timeout_sec").value
        )
        self._log_path = str(self.get_parameter("log_path").value).strip()

        if not self._raw_topics:
            raise ValueError("raw_topics must contain at least one topic")
        if not self._fusion_topic:
            raise ValueError("fusion_topic must not be empty")
        if report_period_sec <= 0.0:
            raise ValueError("report_period_sec must be greater than 0")
        if self._rosbridge_connect_timeout_sec <= 0.0:
            raise ValueError("rosbridge_connect_timeout_sec must be greater than 0")

        all_topics = [*self._raw_topics, self._fusion_topic]
        self._tracker = BandwidthTracker(all_topics, self._window_sec)
        self._tracker_lock = RLock()
        self._subscriptions = []
        self._rosbridge_client = None
        self._rosbridge_listeners = []
        self._log_file = None
        self._open_log_file()

        if self._use_rosbridge:
            self._start_rosbridge_subscriptions(all_topics)
        else:
            self._start_ros_subscriptions(all_topics)

        self._report_timer = self.create_timer(report_period_sec, self._report)

        self.get_logger().info("Compressed image bandwidth test node started.")
        self.get_logger().info(f"Raw topics: {', '.join(self._raw_topics)}")
        self.get_logger().info(f"Fusion topic: {self._fusion_topic}")
        self.get_logger().info(
            f"Window: {self._window_sec:.2f}s, report period: {report_period_sec:.2f}s"
        )
        if self._log_path:
            self.get_logger().info(f"Writing bandwidth txt log to: {self._log_path}")

    def _open_log_file(self) -> None:
        if not self._log_path:
            return

        log_path = Path(self._log_path).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = log_path.open("a", encoding="utf-8")
        self._log_path = str(log_path)
        self._log_file.write(f"# compressed image bandwidth test started at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        self._log_file.write(f"# raw_topics={','.join(self._raw_topics)}\n")
        self._log_file.write(f"# fusion_topic={self._fusion_topic}\n")
        self._log_file.write("# columns: wall_time, raw_total_mbps, fusion_mbps, saved_mbps, reduction_percent, per-topic detail\n")
        self._log_file.write("# per-topic detail includes Mbps, MB/s, msg/s, n, mean/min/max message size, total bytes in window\n")
        self._log_file.flush()

    def _get_string_array_parameter(self, name: str) -> List[str]:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return [topic.strip() for topic in value.split(",") if topic.strip()]
        return [str(topic) for topic in value]

    def _start_ros_subscriptions(self, topics: List[str]) -> None:
        self._subscriptions = [
            self.create_subscription(
                CompressedImage,
                topic,
                self._make_ros_callback(topic),
                10,
            )
            for topic in topics
        ]
        self.get_logger().info("Using native ROS2 subscriptions for image bandwidth test.")

    def _start_rosbridge_subscriptions(self, topics: List[str]) -> None:
        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError("roslibpy is required when use_rosbridge=true") from exc

        self.get_logger().info(
            f"Connecting to rosbridge at {self._rosbridge_host}:{self._rosbridge_port}"
        )
        self._rosbridge_client = roslibpy.Ros(
            host=self._rosbridge_host,
            port=self._rosbridge_port,
        )
        self._rosbridge_client.run()

        deadline_sec = time.time() + self._rosbridge_connect_timeout_sec
        while time.time() < deadline_sec:
            if self._rosbridge_client.is_connected:
                break
            time.sleep(0.1)

        if not self._rosbridge_client.is_connected:
            raise RuntimeError(
                f"Failed to connect to rosbridge at "
                f"{self._rosbridge_host}:{self._rosbridge_port}"
            )

        for topic in topics:
            listener = roslibpy.Topic(
                self._rosbridge_client,
                topic,
                "sensor_msgs/CompressedImage",
            )
            listener.subscribe(self._make_rosbridge_callback(topic))
            self._rosbridge_listeners.append(listener)
            self.get_logger().info(f"Subscribed to {topic} via rosbridge")

    def _make_ros_callback(self, topic: str):
        def callback(msg: CompressedImage) -> None:
            self._record_payload(topic, len(msg.data))

        return callback

    def _make_rosbridge_callback(self, topic: str):
        def callback(msg) -> None:
            data_base64 = msg.get("data", "")
            try:
                payload_bytes = len(base64.b64decode(data_base64))
            except Exception as exc:
                self.get_logger().warning(
                    f"Failed to decode rosbridge CompressedImage data from {topic}: {exc}"
                )
                return
            self._record_payload(topic, payload_bytes)

        return callback

    def _record_payload(self, topic: str, payload_bytes: int) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._tracker_lock:
            self._tracker.record(topic, now_sec, payload_bytes)

    def _report(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._tracker_lock:
            raw_reports = [
                self._tracker.topic_bandwidth(topic, now_sec)
                for topic in self._raw_topics
            ]
            fusion_report = self._tracker.topic_bandwidth(self._fusion_topic, now_sec)
            comparison = self._tracker.comparison(
                self._raw_topics,
                self._fusion_topic,
                now_sec,
            )

        topic_reports = [*raw_reports, fusion_report]
        topic_detail = ", ".join(
            self._format_topic_report(report) for report in topic_reports
        )
        message = (
            "[BW] "
            f"raw_total={comparison.raw_total_mbps:.3f} Mbps, "
            f"fusion={comparison.fusion_mbps:.3f} Mbps, "
            f"saved={comparison.saved_mbps:.3f} Mbps, "
            f"reduction={comparison.reduction_percent:.1f}% | "
            f"{topic_detail}"
        )
        self.get_logger().info(message)
        self._write_log_line(comparison, topic_detail)

    def _format_topic_report(self, report: TopicBandwidth) -> str:
        return (
            f"{report.topic}={report.mbps:.3f} Mbps "
            f"({report.msg_per_sec:.1f} msg/s, n={report.sample_count}, "
            f"bw={report.megabytes_per_sec:.3f} MB/s, "
            f"mean={self._format_bytes(report.mean_msg_bytes)}, "
            f"min={self._format_bytes(report.min_msg_bytes)}, "
            f"max={self._format_bytes(report.max_msg_bytes)}, "
            f"window_bytes={report.total_bytes})"
        )

    def _format_bytes(self, num_bytes: float) -> str:
        if num_bytes < 1_000.0:
            return f"{num_bytes:.0f} B"
        if num_bytes < 1_000_000.0:
            return f"{num_bytes / 1_000.0:.2f} KB"
        return f"{num_bytes / 1_000_000.0:.2f} MB"

    def _write_log_line(self, comparison, raw_detail: str) -> None:
        if self._log_file is None:
            return

        wall_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self._log_file.write(
            f"{wall_time}, "
            f"raw_total_mbps={comparison.raw_total_mbps:.6f}, "
            f"fusion_mbps={comparison.fusion_mbps:.6f}, "
            f"saved_mbps={comparison.saved_mbps:.6f}, "
            f"reduction_percent={comparison.reduction_percent:.3f}, "
            f"{raw_detail}\n"
        )
        self._log_file.flush()

    def destroy_node(self) -> bool:
        for listener in self._rosbridge_listeners:
            try:
                listener.unsubscribe()
            except Exception as exc:
                self.get_logger().warning(f"Failed to unsubscribe rosbridge topic: {exc}")
        self._rosbridge_listeners.clear()

        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

        if self._rosbridge_client is not None:
            try:
                self._rosbridge_client.terminate()
            except Exception as exc:
                self.get_logger().warning(f"Failed to terminate rosbridge client: {exc}")
            self._rosbridge_client = None

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompressedImageBandwidthTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
