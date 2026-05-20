from dataclasses import dataclass
from typing import Sequence

import numpy as np
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class ImageMatch:
    from_uv: tuple[float, float]
    to_uv: tuple[float, float]
    distance: float


class ImageFeatureMatchError(ValueError):
    pass


def image_msg_to_rgb8(msg: Image) -> np.ndarray:
    encoding = (msg.encoding or "").lower()
    height = int(msg.height)
    width = int(msg.width)
    if height <= 0 or width <= 0:
        raise ImageFeatureMatchError("empty image")

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = int(msg.step) if int(msg.step) > 0 else width

    if encoding in ("rgb8", "bgr8"):
        row_bytes = width * 3
        arr = raw.reshape(height, step)[:, :row_bytes].reshape(height, width, 3)
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)

    if encoding in ("rgba8", "bgra8"):
        row_bytes = width * 4
        arr = raw.reshape(height, step)[:, :row_bytes].reshape(height, width, 4)[:, :, :3]
        if encoding == "bgra8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)

    if encoding in ("mono8", "8uc1"):
        row_bytes = width
        arr = raw.reshape(height, step)[:, :row_bytes].reshape(height, width)
        return np.ascontiguousarray(np.repeat(arr[:, :, None], 3, axis=2))

    raise ImageFeatureMatchError(f"unsupported image encoding: {msg.encoding}")


def match_orb_features(
    from_msg: Image,
    to_msg: Image,
    max_features: int = 1500,
    ratio: float = 0.75,
    max_matches: int = 300,
) -> Sequence[ImageMatch]:
    try:
        import cv2
    except Exception as exc:
        raise ImageFeatureMatchError("OpenCV cv2 is not available in this environment") from exc

    from_rgb = image_msg_to_rgb8(from_msg)
    to_rgb = image_msg_to_rgb8(to_msg)
    from_gray = cv2.cvtColor(from_rgb, cv2.COLOR_RGB2GRAY)
    to_gray = cv2.cvtColor(to_rgb, cv2.COLOR_RGB2GRAY)

    orb = cv2.ORB_create(nfeatures=int(max_features))
    kp1, des1 = orb.detectAndCompute(from_gray, None)
    kp2, des2 = orb.detectAndCompute(to_gray, None)
    if des1 is None or des2 is None or len(kp1) == 0 or len(kp2) == 0:
        raise ImageFeatureMatchError("not enough ORB descriptors")

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = matcher.knnMatch(des1, des2, k=2)
    matches: list[ImageMatch] = []
    for pair in knn:
        if len(pair) < 2:
            continue
        best, second = pair
        if best.distance <= ratio * second.distance:
            p1 = kp1[best.queryIdx].pt
            p2 = kp2[best.trainIdx].pt
            matches.append(ImageMatch((float(p1[0]), float(p1[1])), (float(p2[0]), float(p2[1])), float(best.distance)))

    matches.sort(key=lambda item: item.distance)
    return matches[: int(max_matches)]
