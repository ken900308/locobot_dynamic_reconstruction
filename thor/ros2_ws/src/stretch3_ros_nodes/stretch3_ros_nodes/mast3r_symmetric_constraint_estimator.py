from __future__ import annotations

from dataclasses import dataclass
import os
import sys
from pathlib import Path
import time

import numpy as np
from sensor_msgs.msg import Image, PointCloud2

from stretch3_ros_nodes.geometric_constraint_estimator import GeometricConstraintEstimate
from stretch3_ros_nodes.image_feature_matcher import ImageFeatureMatchError, image_msg_to_rgb8
from stretch3_ros_nodes.pointcloud_correspondence import PointCloudCorrespondenceError, build_pixel_index
from stretch3_ros_nodes.sim3_umeyama import Sim3EstimationError, ransac_sim3


class Mast3rSymmetricMatchError(ValueError):
    pass


@dataclass(frozen=True)
class Mast3rSymmetricVerifierConfig:
    mast3r_slam_root: str
    mast3r_model_path: str
    mast3r_config_path: str
    mast3r_device: str
    mast3r_image_size: int
    mast3r_q_conf: float
    mast3r_max_matches: int
    max_correspondence_px: int
    min_3d_correspondences: int
    min_inliers: int
    ransac_iterations: int
    ransac_inlier_threshold_m: float
    max_rmse_m: float
    min_inlier_ratio: float
    min_cloud_confidence: float


class Mast3rSymmetricConstraintEstimator:
    def __init__(self, config: Mast3rSymmetricVerifierConfig, logger=None):
        self.config = config
        self.logger = logger
        self._loaded = False
        self._model = None
        self._torch = None
        self._lietorch = None
        self._create_frame = None
        self._load_config = None
        self._load_mast3r = None
        self._mast3r_match_symmetric = None

    def estimate(
        self,
        from_image: Image,
        to_image: Image,
        from_cloud: PointCloud2,
        to_cloud: PointCloud2,
    ) -> GeometricConstraintEstimate:
        self._ensure_loaded()
        if self.logger is not None:
            self.logger.info(
                f"Starting MASt3R symmetric matching: "
                f"from_image={from_image.width}x{from_image.height}, "
                f"to_image={to_image.width}x{to_image.height}, "
                f"q_conf={self.config.mast3r_q_conf}, max_matches={self.config.mast3r_max_matches}"
            )
        match_started_at = time.monotonic()
        from_frame = self._frame_from_image(0, from_image)
        to_frame = self._frame_from_image(1, to_image)

        with self._torch.inference_mode():
            if from_frame.feat is None:
                from_frame.feat, from_frame.pos, _ = self._model._encode_image(
                    from_frame.img, from_frame.img_true_shape
                )
            if to_frame.feat is None:
                to_frame.feat, to_frame.pos, _ = self._model._encode_image(
                    to_frame.img, to_frame.img_true_shape
                )
            (
                idx_i2j,
                _idx_j2i,
                valid_match_j,
                _valid_match_i,
                qii,
                _qjj,
                qji,
                _qij,
            ) = self._mast3r_match_symmetric(
                self._model,
                from_frame.feat,
                from_frame.pos,
                to_frame.feat,
                to_frame.pos,
                [from_frame.img_true_shape],
                [to_frame.img_true_shape],
            )

        matches = self._matches_from_tensors(
            idx_i2j,
            valid_match_j,
            qii,
            qji,
            from_frame,
            to_frame,
            from_image,
            to_image,
        )
        if self.logger is not None:
            elapsed = time.monotonic() - match_started_at
            self.logger.info(f"MASt3R symmetric matching produced {len(matches)} filtered matches in {elapsed:.3f}s")
        if len(matches) < self.config.min_3d_correspondences:
            raise Mast3rSymmetricMatchError(f"too few MASt3R symmetric matches: {len(matches)}")

        if self.logger is not None:
            self.logger.info(
                f"Building 2D-to-3D indices for MASt3R matches: "
                f"min_cloud_confidence={self.config.min_cloud_confidence}, "
                f"max_correspondence_px={self.config.max_correspondence_px}"
            )
        from_index = build_pixel_index(from_cloud, min_confidence=self.config.min_cloud_confidence)
        to_index = build_pixel_index(to_cloud, min_confidence=self.config.min_cloud_confidence)

        src_points = []
        dst_points = []
        for from_uv, to_uv, _quality in matches:
            src = from_index.nearest_xyz(from_uv[0], from_uv[1], self.config.max_correspondence_px)
            dst = to_index.nearest_xyz(to_uv[0], to_uv[1], self.config.max_correspondence_px)
            if src is None or dst is None:
                continue
            src_points.append(src)
            dst_points.append(dst)

        correspondence_count = len(src_points)
        if self.logger is not None:
            self.logger.info(
                f"MASt3R 2D-to-3D correspondences: {correspondence_count}/{len(matches)} "
                f"within {self.config.max_correspondence_px}px"
            )
        if correspondence_count < self.config.min_3d_correspondences:
            raise PointCloudCorrespondenceError(
                f"too few MASt3R 2D-to-3D correspondences: {correspondence_count}"
            )

        if self.logger is not None:
            self.logger.info(
                f"Running Sim(3) RANSAC: correspondences={correspondence_count}, "
                f"iterations={self.config.ransac_iterations}, "
                f"inlier_threshold_m={self.config.ransac_inlier_threshold_m}"
            )
        sim3 = ransac_sim3(
            np.asarray(src_points, dtype=np.float64),
            np.asarray(dst_points, dtype=np.float64),
            iterations=self.config.ransac_iterations,
            inlier_threshold=self.config.ransac_inlier_threshold_m,
            min_inliers=self.config.min_inliers,
        )
        if self.logger is not None:
            self.logger.info(
                f"Sim(3) RANSAC result: inliers={sim3.inlier_count}/{correspondence_count}, "
                f"rmse={sim3.rmse:.4f} m"
            )
        if sim3.rmse > self.config.max_rmse_m:
            raise Sim3EstimationError(f"MASt3R Sim(3) RMSE is too high: {sim3.rmse:.4f} m")

        inlier_ratio = sim3.inlier_count / max(1, correspondence_count)
        if inlier_ratio < self.config.min_inlier_ratio:
            raise Sim3EstimationError(
                f"MASt3R Sim(3) inlier ratio is too low: "
                f"{inlier_ratio:.3f} < {self.config.min_inlier_ratio:.3f}"
            )
        match_ratio = min(1.0, correspondence_count / max(1, self.config.mast3r_max_matches))
        confidence = min(1.0, float(inlier_ratio) * match_ratio)
        return GeometricConstraintEstimate(
            sim3=sim3,
            match_count=len(matches),
            correspondence_count=correspondence_count,
            confidence=confidence,
        )

    def _ensure_loaded(self) -> None:
        if self._loaded:
            return
        load_started_at = time.monotonic()
        if self.logger is not None:
            self.logger.info(
                f"Loading MASt3R symmetric verifier model: "
                f"root={self.config.mast3r_slam_root}, device={self.config.mast3r_device}"
            )
        root = Path(self.config.mast3r_slam_root).expanduser()
        if not root.exists():
            for candidate in (Path('/workspace/thor/MASt3R-SLAM'), Path('/workspace/MASt3R-SLAM')):
                if candidate.exists():
                    root = candidate
                    break
        if not root.exists():
            raise Mast3rSymmetricMatchError(f"MASt3R-SLAM root does not exist: {root}")

        thirdparty = root / 'thirdparty' / 'mast3r'
        for path in (thirdparty, root):
            path_text = str(path)
            if path_text not in sys.path:
                sys.path.insert(0, path_text)

        import torch
        import lietorch
        from mast3r_slam.config import load_config
        from mast3r_slam.frame import create_frame
        from mast3r_slam.mast3r_utils import load_mast3r, mast3r_match_symmetric

        config_path = Path(self.config.mast3r_config_path)
        if not config_path.is_absolute():
            config_path = root / config_path
        load_config(str(config_path))

        model_path = self.config.mast3r_model_path.strip()
        if model_path:
            model_path_obj = Path(model_path)
            if not model_path_obj.is_absolute():
                model_path_obj = root / model_path_obj
            model_path = str(model_path_obj)
        else:
            model_path = str(root / 'checkpoints' / 'MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth')

        self._torch = torch
        self._lietorch = lietorch
        self._create_frame = create_frame
        self._load_config = load_config
        self._load_mast3r = load_mast3r
        self._mast3r_match_symmetric = mast3r_match_symmetric
        self._model = load_mast3r(path=model_path, device=self.config.mast3r_device)
        self._model.eval()
        self._loaded = True
        if self.logger is not None:
            elapsed = time.monotonic() - load_started_at
            self.logger.info(
                f"Loaded MASt3R symmetric verifier in {elapsed:.3f}s: "
                f"root={root}, model={model_path}, "
                f"device={self.config.mast3r_device}, q_conf={self.config.mast3r_q_conf}"
            )

    def _frame_from_image(self, frame_id: int, msg: Image):
        rgb = image_msg_to_rgb8(msg).astype(np.float32) / 255.0
        transform = self._lietorch.Sim3.Identity(1, device=self.config.mast3r_device)
        return self._create_frame(
            frame_id,
            rgb,
            transform,
            img_size=int(self.config.mast3r_image_size),
            device=self.config.mast3r_device,
        )

    def _matches_from_tensors(self, idx_i2j, valid_match_j, qii, qji, from_frame, to_frame, from_image: Image, to_image: Image):
        idx_np = idx_i2j.detach().reshape(-1).cpu().numpy().astype(np.int64)
        valid_np = valid_match_j.detach().reshape(-1).cpu().numpy().astype(bool)
        qii_np = qii.detach().reshape(-1).cpu().numpy().astype(np.float32)
        qji_np = qji.detach().reshape(-1).cpu().numpy().astype(np.float32)

        src_flat = np.arange(idx_np.shape[0], dtype=np.int64)
        valid_np = valid_np & (idx_np >= 0) & (idx_np < qii_np.shape[0])
        if not np.any(valid_np):
            return []

        quality = np.sqrt(np.maximum(0.0, qii_np[idx_np[valid_np]]) * np.maximum(0.0, qji_np[valid_np]))
        keep = quality >= float(self.config.mast3r_q_conf)
        src_flat = src_flat[valid_np][keep]
        dst_flat = idx_np[valid_np][keep]
        quality = quality[keep]
        if src_flat.shape[0] == 0:
            return []

        order = np.argsort(-quality)
        max_matches = int(self.config.mast3r_max_matches)
        if max_matches > 0:
            order = order[:max_matches]
        src_flat = src_flat[order]
        dst_flat = dst_flat[order]
        quality = quality[order]

        from_image_width = int(from_image.width)
        from_image_height = int(from_image.height)
        to_image_width = int(to_image.width)
        to_image_height = int(to_image.height)
        from_shape = [int(x) for x in from_frame.img_shape.flatten().detach().cpu().tolist()]
        to_shape = [int(x) for x in to_frame.img_shape.flatten().detach().cpu().tolist()]
        from_match_height, from_match_width = from_shape[0], from_shape[1]
        to_match_height, to_match_width = to_shape[0], to_shape[1]
        if min(from_image_width, from_image_height, to_image_width, to_image_height) <= 0:
            raise ImageFeatureMatchError("empty image dimensions for MASt3R matches")
        if min(from_match_width, from_match_height, to_match_width, to_match_height) <= 0:
            raise ImageFeatureMatchError("empty MASt3R match grid dimensions")

        from_u_scale = from_image_width / float(from_match_width)
        from_v_scale = from_image_height / float(from_match_height)
        to_u_scale = to_image_width / float(to_match_width)
        to_v_scale = to_image_height / float(to_match_height)

        matches = []
        for src, dst, q in zip(src_flat, dst_flat, quality):
            from_u_grid = int(src % from_match_width)
            from_v_grid = int(src // from_match_width)
            to_u_grid = int(dst % to_match_width)
            to_v_grid = int(dst // to_match_width)
            if from_v_grid >= from_match_height or to_v_grid >= to_match_height:
                continue
            from_u = from_u_grid * from_u_scale
            from_v = from_v_grid * from_v_scale
            to_u = to_u_grid * to_u_scale
            to_v = to_v_grid * to_v_scale
            matches.append(((float(from_u), float(from_v)), (float(to_u), float(to_v)), float(q)))
        return matches
