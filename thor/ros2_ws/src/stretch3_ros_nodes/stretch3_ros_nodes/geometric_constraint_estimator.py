from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import Image, PointCloud2

from stretch3_ros_nodes.image_feature_matcher import ImageFeatureMatchError, match_orb_features
from stretch3_ros_nodes.pointcloud_correspondence import PointCloudCorrespondenceError, build_pixel_index
from stretch3_ros_nodes.sim3_umeyama import Sim3EstimationError, Sim3Estimate, ransac_sim3


@dataclass(frozen=True)
class GeometricVerifierConfig:
    max_features: int
    max_feature_matches: int
    feature_ratio: float
    max_correspondence_px: int
    min_3d_correspondences: int
    min_inliers: int
    ransac_iterations: int
    ransac_inlier_threshold_m: float
    max_rmse_m: float
    min_cloud_confidence: float


@dataclass(frozen=True)
class GeometricConstraintEstimate:
    sim3: Sim3Estimate
    match_count: int
    correspondence_count: int
    confidence: float


def estimate_geometric_constraint(
    from_image: Image,
    to_image: Image,
    from_cloud: PointCloud2,
    to_cloud: PointCloud2,
    config: GeometricVerifierConfig,
) -> GeometricConstraintEstimate:
    matches = match_orb_features(
        from_image,
        to_image,
        max_features=config.max_features,
        ratio=config.feature_ratio,
        max_matches=config.max_feature_matches,
    )
    if len(matches) < config.min_3d_correspondences:
        raise ImageFeatureMatchError(f"too few descriptor matches: {len(matches)}")

    from_index = build_pixel_index(from_cloud, min_confidence=config.min_cloud_confidence)
    to_index = build_pixel_index(to_cloud, min_confidence=config.min_cloud_confidence)

    src_points = []
    dst_points = []
    for match in matches:
        src = from_index.nearest_xyz(match.from_uv[0], match.from_uv[1], config.max_correspondence_px)
        dst = to_index.nearest_xyz(match.to_uv[0], match.to_uv[1], config.max_correspondence_px)
        if src is None or dst is None:
            continue
        src_points.append(src)
        dst_points.append(dst)

    correspondence_count = len(src_points)
    if correspondence_count < config.min_3d_correspondences:
        raise PointCloudCorrespondenceError(f"too few 2D-to-3D correspondences: {correspondence_count}")

    sim3 = ransac_sim3(
        np.asarray(src_points, dtype=np.float64),
        np.asarray(dst_points, dtype=np.float64),
        iterations=config.ransac_iterations,
        inlier_threshold=config.ransac_inlier_threshold_m,
        min_inliers=config.min_inliers,
    )
    if sim3.rmse > config.max_rmse_m:
        raise Sim3EstimationError(f"Sim(3) RMSE is too high: {sim3.rmse:.4f} m")

    inlier_ratio = sim3.inlier_count / max(1, correspondence_count)
    confidence = min(1.0, float(inlier_ratio) * min(1.0, correspondence_count / max(1, config.max_feature_matches)))
    return GeometricConstraintEstimate(
        sim3=sim3,
        match_count=len(matches),
        correspondence_count=correspondence_count,
        confidence=confidence,
    )
