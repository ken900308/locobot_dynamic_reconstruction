from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, Tuple


@dataclass(frozen=True)
class TopicBandwidth:
    topic: str
    bytes_per_sec: float
    msg_per_sec: float
    sample_count: int
    total_bytes: int
    mean_msg_bytes: float
    min_msg_bytes: int
    max_msg_bytes: int

    @property
    def mbps(self) -> float:
        return self.bytes_per_sec * 8.0 / 1_000_000.0

    @property
    def megabytes_per_sec(self) -> float:
        return self.bytes_per_sec / 1_000_000.0


@dataclass(frozen=True)
class BandwidthComparison:
    raw_total_mbps: float
    fusion_mbps: float
    saved_mbps: float
    reduction_percent: float


class BandwidthTracker:
    def __init__(self, topics: Iterable[str], window_sec: float) -> None:
        if window_sec <= 0.0:
            raise ValueError("window_sec must be greater than 0")

        self._window_sec = window_sec
        self._samples: Dict[str, Deque[Tuple[float, int]]] = {
            topic: deque() for topic in topics
        }

    def record(self, topic: str, stamp_sec: float, payload_bytes: int) -> None:
        if topic not in self._samples:
            raise KeyError(f"Unknown topic: {topic}")
        if payload_bytes < 0:
            raise ValueError("payload_bytes must not be negative")

        samples = self._samples[topic]
        samples.append((stamp_sec, payload_bytes))
        self._prune(samples, stamp_sec)

    def topic_bandwidth(self, topic: str, now_sec: float) -> TopicBandwidth:
        samples = self._samples[topic]
        self._prune(samples, now_sec)

        payload_sizes = [payload_bytes for _, payload_bytes in samples]
        total_bytes = sum(payload_sizes)
        elapsed_sec = self._effective_window(samples, now_sec)
        sample_count = len(payload_sizes)
        bytes_per_sec = total_bytes / elapsed_sec if elapsed_sec > 0.0 else 0.0
        msg_per_sec = sample_count / elapsed_sec if elapsed_sec > 0.0 else 0.0
        mean_msg_bytes = total_bytes / sample_count if sample_count else 0.0
        min_msg_bytes = min(payload_sizes) if payload_sizes else 0
        max_msg_bytes = max(payload_sizes) if payload_sizes else 0

        return TopicBandwidth(
            topic=topic,
            bytes_per_sec=bytes_per_sec,
            msg_per_sec=msg_per_sec,
            sample_count=sample_count,
            total_bytes=total_bytes,
            mean_msg_bytes=mean_msg_bytes,
            min_msg_bytes=min_msg_bytes,
            max_msg_bytes=max_msg_bytes,
        )

    def comparison(
        self,
        raw_topics: Iterable[str],
        fusion_topic: str,
        now_sec: float,
    ) -> BandwidthComparison:
        raw_total_mbps = sum(
            self.topic_bandwidth(topic, now_sec).mbps for topic in raw_topics
        )
        fusion_mbps = self.topic_bandwidth(fusion_topic, now_sec).mbps
        saved_mbps = raw_total_mbps - fusion_mbps
        reduction_percent = (
            saved_mbps / raw_total_mbps * 100.0 if raw_total_mbps > 0.0 else 0.0
        )

        return BandwidthComparison(
            raw_total_mbps=raw_total_mbps,
            fusion_mbps=fusion_mbps,
            saved_mbps=saved_mbps,
            reduction_percent=reduction_percent,
        )

    def _prune(self, samples: Deque[Tuple[float, int]], now_sec: float) -> None:
        cutoff_sec = now_sec - self._window_sec
        while samples and samples[0][0] < cutoff_sec:
            samples.popleft()

    def _effective_window(
        self,
        samples: Deque[Tuple[float, int]],
        now_sec: float,
    ) -> float:
        if not samples:
            return 0.0
        oldest_sec = samples[0][0]
        return min(self._window_sec, max(now_sec - oldest_sec, 1e-6))
