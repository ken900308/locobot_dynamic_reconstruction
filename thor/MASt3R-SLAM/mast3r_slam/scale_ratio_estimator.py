import bisect
import csv
import math
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np


CSV_COLUMNS = [
    "keyframe_index",
    "frame_id",
    "timestamp",
    "odom_source",
    "odom_x",
    "odom_y",
    "mast3r_x",
    "mast3r_y",
    "mast3r_z",
    "d_odom",
    "d_mast3r_3d",
    "d_mast3r_xy_debug",
    "total_odom",
    "total_mast3r",
    "scale_ratio",
    "used_segment",
    "skip_reason",
]


@dataclass
class OdomLookup:
    xy: np.ndarray | None
    reason: str = ""

    @property
    def ok(self):
        return self.xy is not None


def _to_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


def extract_translation_xyz(T_WC):
    """Return xyz translation from common pose/tensor representations."""
    value = T_WC
    if hasattr(value, "matrix"):
        value = value.matrix()
    arr = _to_numpy(value)

    if arr.ndim >= 3:
        arr = arr.reshape((-1,) + arr.shape[-2:])[0]

    if arr.shape[-2:] == (4, 4):
        return arr[:3, 3].astype(np.float64)
    if arr.shape[-2:] == (3, 4):
        return arr[:3, 3].astype(np.float64)

    flat = arr.reshape(-1)
    if flat.size >= 7:
        return flat[:3].astype(np.float64)
    if flat.size >= 3:
        return flat[:3].astype(np.float64)

    raise ValueError(f"Cannot extract translation from T_WC with shape {arr.shape}")


class TumOdomTrajectory:
    source_name = "file"

    def __init__(self, path):
        self.path = Path(path)
        self.timestamps, self.xy = self._load(self.path)

    @property
    def start_time(self):
        return float(self.timestamps[0])

    @property
    def end_time(self):
        return float(self.timestamps[-1])

    def __len__(self):
        return int(self.timestamps.shape[0])

    def _load(self, path):
        if not path.exists():
            raise FileNotFoundError(f"Odom trajectory file does not exist: {path}")

        rows = []
        with path.open("r", encoding="utf-8") as f:
            for line_no, line in enumerate(f, start=1):
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                parts = stripped.split()
                if len(parts) < 3:
                    raise ValueError(
                        f"Malformed odom TUM row at {path}:{line_no}: "
                        "expected at least 'timestamp x y'"
                    )
                try:
                    t = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                except ValueError as exc:
                    raise ValueError(
                        f"Malformed numeric value in odom TUM row at {path}:{line_no}"
                    ) from exc
                rows.append((t, x, y, line_no))

        if not rows:
            raise ValueError(f"Odom trajectory file has no samples: {path}")

        rows.sort(key=lambda row: row[0])
        dedup = []
        for row in rows:
            if dedup and math.isclose(row[0], dedup[-1][0], rel_tol=0.0, abs_tol=1e-9):
                if (
                    not math.isclose(row[1], dedup[-1][1], rel_tol=0.0, abs_tol=1e-9)
                    or not math.isclose(row[2], dedup[-1][2], rel_tol=0.0, abs_tol=1e-9)
                ):
                    raise ValueError(
                        f"Conflicting duplicate odom timestamp {row[0]:.9f} "
                        f"at lines {dedup[-1][3]} and {row[3]}"
                    )
                continue
            dedup.append(row)

        timestamps = np.array([row[0] for row in dedup], dtype=np.float64)
        xy = np.array([[row[1], row[2]] for row in dedup], dtype=np.float64)
        return timestamps, xy

    def sample_xy(self, timestamp):
        t = float(timestamp)
        if t < self.start_time or t > self.end_time:
            return OdomLookup(None, "odom_out_of_range")

        idx = int(np.searchsorted(self.timestamps, t, side="left"))
        if idx < len(self.timestamps) and math.isclose(
            float(self.timestamps[idx]), t, rel_tol=0.0, abs_tol=1e-9
        ):
            return OdomLookup(self.xy[idx].copy())
        if idx == 0 or idx >= len(self.timestamps):
            return OdomLookup(None, "odom_out_of_range")

        t0 = float(self.timestamps[idx - 1])
        t1 = float(self.timestamps[idx])
        alpha = (t - t0) / (t1 - t0)
        return OdomLookup((1.0 - alpha) * self.xy[idx - 1] + alpha * self.xy[idx])


class LiveOdomBuffer:
    source_name = "live"

    def __init__(self, max_time_diff=0.05, buffer_sec=300.0, logger=None):
        self.max_time_diff = float(max_time_diff)
        self.buffer_sec = float(buffer_sec)
        self.logger = logger
        self.lock = threading.RLock()
        self.timestamps = []
        self.xy = []
        self.first_sample_logged = False

    def _info(self, msg):
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg)

    def add_sample(self, timestamp, x, y):
        t = float(timestamp)
        point = np.array([float(x), float(y)], dtype=np.float64)
        with self.lock:
            if self.timestamps and t >= self.timestamps[-1]:
                if math.isclose(t, self.timestamps[-1], rel_tol=0.0, abs_tol=1e-9):
                    self.xy[-1] = point
                else:
                    self.timestamps.append(t)
                    self.xy.append(point)
            else:
                idx = bisect.bisect_left(self.timestamps, t)
                if idx < len(self.timestamps) and math.isclose(
                    self.timestamps[idx], t, rel_tol=0.0, abs_tol=1e-9
                ):
                    self.xy[idx] = point
                else:
                    self.timestamps.insert(idx, t)
                    self.xy.insert(idx, point)

            latest = self.timestamps[-1]
            prune_before = latest - self.buffer_sec
            prune_idx = bisect.bisect_left(self.timestamps, prune_before)
            if prune_idx > 0:
                del self.timestamps[:prune_idx]
                del self.xy[:prune_idx]

            if not self.first_sample_logged:
                self.first_sample_logged = True
                self._info(f"[ScaleRatio pre-PGO] First live odom timestamp: {t:.6f}")

    def __len__(self):
        with self.lock:
            return len(self.timestamps)

    @property
    def start_time(self):
        with self.lock:
            return float(self.timestamps[0]) if self.timestamps else None

    @property
    def end_time(self):
        with self.lock:
            return float(self.timestamps[-1]) if self.timestamps else None

    def sample_xy(self, timestamp):
        t = float(timestamp)
        with self.lock:
            if not self.timestamps:
                return OdomLookup(None, "live_odom_empty")

            idx = bisect.bisect_left(self.timestamps, t)
            if idx < len(self.timestamps) and math.isclose(
                self.timestamps[idx], t, rel_tol=0.0, abs_tol=1e-9
            ):
                return OdomLookup(self.xy[idx].copy())

            if 0 < idx < len(self.timestamps):
                t0 = self.timestamps[idx - 1]
                t1 = self.timestamps[idx]
                alpha = (t - t0) / (t1 - t0)
                return OdomLookup((1.0 - alpha) * self.xy[idx - 1] + alpha * self.xy[idx])

            candidates = []
            if idx > 0:
                candidates.append((abs(t - self.timestamps[idx - 1]), idx - 1))
            if idx < len(self.timestamps):
                candidates.append((abs(t - self.timestamps[idx]), idx))
            if not candidates:
                return OdomLookup(None, "live_odom_empty")

            best_dt, best_idx = min(candidates, key=lambda item: item[0])
            if best_dt <= self.max_time_diff:
                return OdomLookup(self.xy[best_idx].copy())
            return OdomLookup(None, "live_odom_time_diff_exceeded")


class KeyframeScaleRatioEstimator:
    def __init__(
        self,
        odom_path=None,
        odom_source=None,
        output_csv="logs/scale_ratio_log.csv",
        min_odom_delta=0.03,
        min_mast3r_delta=1e-4,
        logger=None,
        out_of_range_warn_every=5,
    ):
        if odom_path and odom_source is not None:
            raise ValueError("Provide either odom_path or odom_source, not both")
        if odom_source is None:
            if not odom_path:
                raise ValueError("KeyframeScaleRatioEstimator needs odom_path or odom_source")
            odom_source = TumOdomTrajectory(odom_path)

        self.odom = odom_source
        self.odom_source_name = getattr(self.odom, "source_name", "unknown")
        self.output_csv = Path(output_csv)
        self.min_odom_delta = float(min_odom_delta)
        self.min_mast3r_delta = float(min_mast3r_delta)
        self.logger = logger
        self.out_of_range_warn_every = int(out_of_range_warn_every)

        self.prev_odom_xy = None
        self.prev_mast3r_xyz = None
        self.prev_timestamp = None
        self.total_odom = 0.0
        self.total_mast3r = 0.0
        self.lookup_failure_count = 0
        self.first_keyframe_seen = False

        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        with self.output_csv.open("w", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=CSV_COLUMNS).writeheader()

        if self.odom_source_name == "file":
            self._info(
                "[ScaleRatio pre-PGO] Loaded odom TUM: "
                f"{len(self.odom)} samples, range=[{self.odom.start_time:.6f}, {self.odom.end_time:.6f}], "
                f"csv={self.output_csv}"
            )
        else:
            self._info(
                "[ScaleRatio pre-PGO] Live odom enabled: "
                f"buffer_sec={self.odom.buffer_sec:.3f}, max_time_diff={self.odom.max_time_diff:.3f}, "
                f"csv={self.output_csv}"
            )

    def _info(self, msg):
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg)

    def _warn(self, msg):
        if self.logger is not None:
            if hasattr(self.logger, "warning"):
                self.logger.warning(msg)
            else:
                self.logger.warn(msg)
        else:
            print(msg)

    def _append_row(self, row):
        with self.output_csv.open("a", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=CSV_COLUMNS).writerow(row)

    @staticmethod
    def _fmt(value):
        if value is None:
            return ""
        if isinstance(value, bool):
            return str(value)
        if isinstance(value, str):
            return value
        return f"{float(value):.9f}"

    def on_keyframe(self, keyframe_index, frame_id, timestamp, T_WC):
        timestamp = float(timestamp)
        mast3r_xyz = extract_translation_xyz(T_WC)

        if not self.first_keyframe_seen:
            self._info(f"[ScaleRatio pre-PGO] First keyframe timestamp: {timestamp:.6f}")
            self.first_keyframe_seen = True

        lookup = self.odom.sample_xy(timestamp)
        if not lookup.ok:
            self.lookup_failure_count += 1
            skip_reason = lookup.reason or "odom_unavailable"
            self._append_measurement_row(
                keyframe_index,
                frame_id,
                timestamp,
                None,
                mast3r_xyz,
                None,
                None,
                None,
                False,
                skip_reason,
            )
            self._info(
                f"[ScaleRatio pre-PGO] source={self.odom_source_name} KF={keyframe_index} "
                f"frame_id={frame_id} t={timestamp:.3f} used=False skip={skip_reason}"
            )
            if self.lookup_failure_count == 1 or (
                self.lookup_failure_count % self.out_of_range_warn_every == 0
            ):
                self._warn_lookup_failure(timestamp, skip_reason)
            return

        odom_xy = lookup.xy
        if self.prev_odom_xy is None or self.prev_mast3r_xyz is None:
            self.prev_odom_xy = odom_xy
            self.prev_mast3r_xyz = mast3r_xyz
            self.prev_timestamp = timestamp
            self._append_measurement_row(
                keyframe_index,
                frame_id,
                timestamp,
                odom_xy,
                mast3r_xyz,
                None,
                None,
                None,
                False,
                "first_keyframe",
            )
            self._info(
                f"[ScaleRatio pre-PGO] source={self.odom_source_name} KF={keyframe_index} "
                f"frame_id={frame_id} t={timestamp:.3f} used=False skip=first_keyframe"
            )
            return

        d_odom = float(np.linalg.norm(odom_xy - self.prev_odom_xy))
        d_mast3r_3d = float(np.linalg.norm(mast3r_xyz - self.prev_mast3r_xyz))
        d_mast3r_xy = float(np.linalg.norm(mast3r_xyz[:2] - self.prev_mast3r_xyz[:2]))

        used_segment = True
        skip_reason = ""
        if d_odom <= self.min_odom_delta:
            used_segment = False
            skip_reason = "odom_delta_below_threshold"
        elif d_mast3r_3d <= self.min_mast3r_delta:
            used_segment = False
            skip_reason = "mast3r_delta_below_threshold"

        if used_segment:
            self.total_odom += d_odom
            self.total_mast3r += d_mast3r_3d

        scale = self.total_odom / self.total_mast3r if self.total_mast3r > 1e-12 else None
        self._append_measurement_row(
            keyframe_index,
            frame_id,
            timestamp,
            odom_xy,
            mast3r_xyz,
            d_odom,
            d_mast3r_3d,
            d_mast3r_xy,
            used_segment,
            skip_reason,
            scale,
        )

        scale_text = f"{scale:.3f}" if scale is not None else "nan"
        self._info(
            f"[ScaleRatio pre-PGO] source={self.odom_source_name} KF={keyframe_index} "
            f"frame_id={frame_id} t={timestamp:.3f} d_odom_xy={d_odom:.3f} "
            f"d_mast3r_3d={d_mast3r_3d:.3f} total_odom_xy={self.total_odom:.3f} "
            f"total_mast3r_3d={self.total_mast3r:.3f} scale={scale_text} used={used_segment}"
        )

        self.prev_odom_xy = odom_xy
        self.prev_mast3r_xyz = mast3r_xyz
        self.prev_timestamp = timestamp

    def current_scale_ratio(self):
        """Return the current cumulative scale estimate, or None until it is usable."""
        if self.total_mast3r <= 1e-12:
            return None
        return self.total_odom / self.total_mast3r

    def _warn_lookup_failure(self, timestamp, skip_reason):
        if self.odom_source_name == "file":
            self._warn(
                f"[ScaleRatio pre-PGO] {self.lookup_failure_count} keyframe odom lookups failed "
                f"(latest t={timestamp:.6f}, reason={skip_reason}). Check timestamp-source mismatch: "
                "wall time vs ROS bag/header time, mp4-relative time vs ROS time, or mismatched odom file."
            )
            return

        start = self.odom.start_time
        end = self.odom.end_time
        range_text = "<empty>" if start is None else f"[{start:.6f}, {end:.6f}]"
        self._warn(
            f"[ScaleRatio pre-PGO] {self.lookup_failure_count} live odom lookups failed "
            f"(latest keyframe t={timestamp:.6f}, reason={skip_reason}, live range={range_text}). "
            "Check odom topic name/publication, ROS header timestamps, rosbag/image/odom timestamp "
            "mismatch, keyframe time outside live buffer, or MASt3R lag exceeding odom_buffer_sec."
        )

    def _append_measurement_row(
        self,
        keyframe_index,
        frame_id,
        timestamp,
        odom_xy,
        mast3r_xyz,
        d_odom,
        d_mast3r_3d,
        d_mast3r_xy_debug,
        used_segment,
        skip_reason,
        scale_ratio=None,
    ):
        row = {
            "keyframe_index": keyframe_index,
            "frame_id": frame_id,
            "timestamp": self._fmt(timestamp),
            "odom_source": self.odom_source_name,
            "odom_x": self._fmt(None if odom_xy is None else odom_xy[0]),
            "odom_y": self._fmt(None if odom_xy is None else odom_xy[1]),
            "mast3r_x": self._fmt(mast3r_xyz[0]),
            "mast3r_y": self._fmt(mast3r_xyz[1]),
            "mast3r_z": self._fmt(mast3r_xyz[2]),
            "d_odom": self._fmt(d_odom),
            "d_mast3r_3d": self._fmt(d_mast3r_3d),
            "d_mast3r_xy_debug": self._fmt(d_mast3r_xy_debug),
            "total_odom": self._fmt(self.total_odom),
            "total_mast3r": self._fmt(self.total_mast3r),
            "scale_ratio": self._fmt(scale_ratio),
            "used_segment": str(bool(used_segment)),
            "skip_reason": skip_reason,
        }
        self._append_row(row)
