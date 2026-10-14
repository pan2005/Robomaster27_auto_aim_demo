from __future__ import annotations

import dataclasses
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclasses.dataclass(frozen=True)
class TagPose:
    tag_id: int
    corners: np.ndarray
    center: tuple[float, float]
    rvec: np.ndarray
    tvec_m: np.ndarray
    reprojection_error_px: float
    capture_time_ns: int


def load_camera_calibration(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    matrix = np.asarray(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(data["distort_coeffs"], dtype=np.float64).reshape(-1, 1)
    return matrix, distortion, (int(data["image_width"]), int(data["image_height"]))


class AprilTagDetector:
    def __init__(self, camera_matrix: np.ndarray, distortion: np.ndarray,
                 tag_size_m: float = 0.1,
                 pose_distance_range_m: tuple[float, float] = (0.2, 6.0)) -> None:
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.tag_size_m = tag_size_m
        # Keep visual detection/PnP wider than the 1--4 m engagement envelope.
        # Otherwise a perfectly visible close tag disappears before the control
        # layer can explain why it is not safe to track.
        self.pose_distance_range_m = pose_distance_range_m
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        parameters = cv2.aruco.DetectorParameters()
        # CORNER_REFINE_APRILTAG is extremely expensive on textured camera
        # frames (tens of milliseconds on the target PC).  Detection only
        # needs integer/low-resolution corners here; every accepted marker is
        # refined below on the original grayscale image with cornerSubPix.
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def detect(self, rgb: np.ndarray, capture_time_ns: int,
               locked_id: int | None = None,
               roi: tuple[int, int, int, int] | None = None) -> list[TagPose]:
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if roi is None:
            search = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            scale, offset_x, offset_y = 2.0, 0, 0
        else:
            left, top, right, bottom = roi
            left = max(0, min(gray.shape[1] - 1, int(left)))
            top = max(0, min(gray.shape[0] - 1, int(top)))
            right = max(left + 1, min(gray.shape[1], int(right)))
            bottom = max(top + 1, min(gray.shape[0], int(bottom)))
            search = gray[top:bottom, left:right]
            scale, offset_x, offset_y = 1.0, left, top
        detected, ids, _ = self.detector.detectMarkers(search)
        if ids is None:
            return []
        object_points = self._object_points()
        poses: list[TagPose] = []
        for raw_corners, raw_id in zip(detected, ids.reshape(-1)):
            tag_id = int(raw_id)
            if locked_id is not None and tag_id != locked_id:
                continue
            corners = raw_corners.reshape(4, 2).astype(np.float32) * scale
            corners[:, 0] += offset_x
            corners[:, 1] += offset_y
            cv2.cornerSubPix(
                gray, corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            sides = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
            if float(np.min(sides)) < 20.0:
                continue
            count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                object_points, corners, self.camera_matrix, self.distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            best = None
            for index in range(int(count)):
                tvec = np.asarray(tvecs[index], dtype=np.float64).reshape(3)
                error = float(np.asarray(errors[index]).reshape(-1)[0])
                distance = float(np.linalg.norm(tvec))
                if (tvec[2] <= 0 or
                        not self.pose_distance_range_m[0] <= distance <=
                        self.pose_distance_range_m[1]):
                    continue
                if best is None or error < best[0]:
                    best = (error, np.asarray(rvecs[index]).reshape(3), tvec)
            if best is None or best[0] > 1.5:
                continue
            center = tuple(np.mean(corners, axis=0).astype(float))
            poses.append(TagPose(tag_id, corners, center, best[1], best[2],
                                 best[0], capture_time_ns))
        return poses

    def _object_points(self) -> np.ndarray:
        half = self.tag_size_m * 0.5
        return np.asarray([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float32)
