from __future__ import annotations

import dataclasses
import datetime as dt
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclasses.dataclass(frozen=True)
class DetectionResult:
    success: bool
    message: str
    preview_rgb: np.ndarray
    saved_path: Path | None = None


@dataclasses.dataclass(frozen=True)
class CalibrationResult:
    camera_matrix: np.ndarray
    distortion: np.ndarray
    rms: float
    mean_reprojection_error: float
    per_view_errors: list[float]
    image_size: tuple[int, int]
    output_path: Path


def rotate_rgb(image: np.ndarray, degrees: int) -> np.ndarray:
    rotation = degrees % 360
    if rotation == 0:
        return np.ascontiguousarray(image)
    if rotation == 90:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    if rotation == 180:
        return cv2.rotate(image, cv2.ROTATE_180)
    if rotation == 270:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError("rotation must be 0, 90, 180, or 270 degrees")


def _blob_detector(image_shape: tuple[int, ...]) -> cv2.SimpleBlobDetector:
    height, width = image_shape[:2]
    area = float(width * height)
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 5
    params.maxThreshold = 245
    params.thresholdStep = 10
    params.filterByArea = True
    params.minArea = max(20.0, area * 0.00001)
    params.maxArea = area * 0.03
    params.filterByCircularity = True
    params.minCircularity = 0.55
    params.filterByConvexity = True
    params.minConvexity = 0.7
    params.filterByInertia = True
    params.minInertiaRatio = 0.35
    params.filterByColor = True
    params.blobColor = 0
    return cv2.SimpleBlobDetector_create(params)


def find_circle_grid(image_rgb: np.ndarray, columns: int,
                     rows: int) -> tuple[bool, np.ndarray | None]:
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    flags = cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING
    success, centers = cv2.findCirclesGrid(
        gray, (columns, rows), flags=flags,
        blobDetector=_blob_detector(gray.shape))
    if not success:
        # The default OpenCV detector is a useful fallback for large, clean dots.
        success, centers = cv2.findCirclesGrid(
            gray, (columns, rows), flags=flags)
    return bool(success), centers


class IntrinsicCalibrationSession:
    def __init__(self, base_folder: Path, columns: int = 10, rows: int = 7,
                 spacing_mm: float = 40.0, rotation_degrees: int = 90,
                 session_folder: Path | None = None) -> None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = session_folder or (base_folder / stamp)
        self.images_folder = self.folder / "images"
        self.previews_folder = self.folder / "previews"
        self.columns = columns
        self.rows = rows
        self.spacing_mm = spacing_mm
        self.rotation_degrees = rotation_degrees
        self.image_paths: list[Path] = []
        self.image_points: list[np.ndarray] = []
        self.image_size: tuple[int, int] | None = None

    def _save_metadata(self) -> None:
        self.folder.mkdir(parents=True, exist_ok=True)
        metadata = {
            "pattern_cols": self.columns,
            "pattern_rows": self.rows,
            "center_distance_mm": float(self.spacing_mm),
            "rotation_degrees_clockwise": self.rotation_degrees,
            "image_width": self.image_size[0] if self.image_size else None,
            "image_height": self.image_size[1] if self.image_size else None,
        }
        with (self.folder / "session.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, allow_unicode=True, sort_keys=False)

    @classmethod
    def load(cls, folder: Path, columns: int = 10, rows: int = 7,
             spacing_mm: float = 40.0,
             rotation_degrees: int = 90) -> tuple["IntrinsicCalibrationSession", int]:
        """Re-detect saved, already-rotated images and restore a session."""
        metadata_path = folder / "session.yaml"
        if not metadata_path.is_file():
            metadata_path = folder / "camera_calibration.yaml"
        if metadata_path.is_file():
            with metadata_path.open(encoding="utf-8") as stream:
                metadata = yaml.safe_load(stream) or {}
            columns = int(metadata.get("pattern_cols", columns))
            rows = int(metadata.get("pattern_rows", rows))
            spacing_mm = float(metadata.get("center_distance_mm", spacing_mm))
            rotation_degrees = int(metadata.get(
                "rotation_degrees_clockwise", rotation_degrees))

        session = cls(folder.parent, columns, rows, spacing_mm,
                      rotation_degrees, session_folder=folder)
        image_files = sorted((folder / "images").glob("*.png"))
        skipped = 0
        for image_path in image_files:
            image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image_bgr is None:
                skipped += 1
                continue
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            current_size = (image_rgb.shape[1], image_rgb.shape[0])
            if session.image_size is not None and current_size != session.image_size:
                skipped += 1
                continue
            success, centers = find_circle_grid(image_rgb, columns, rows)
            if not success or centers is None:
                skipped += 1
                continue
            session.image_size = current_size
            session.image_paths.append(image_path)
            session.image_points.append(centers.astype(np.float32))
        if not session.image_paths:
            raise ValueError(f"{folder} 中没有可恢复的有效标定图片")
        session._save_metadata()
        return session, skipped

    def configure(self, columns: int, rows: int, spacing_mm: float,
                  rotation_degrees: int) -> None:
        if self.image_paths and (columns != self.columns or rows != self.rows or
                                 abs(spacing_mm - self.spacing_mm) > 1e-6 or
                                 rotation_degrees != self.rotation_degrees):
            raise ValueError("已有采样后不能修改标定板规格或图像方向，请新建标定会话")
        self.columns = columns
        self.rows = rows
        self.spacing_mm = spacing_mm
        self.rotation_degrees = rotation_degrees

    @property
    def count(self) -> int:
        return len(self.image_paths)

    def capture(self, raw_rgb: np.ndarray) -> DetectionResult:
        image = rotate_rgb(raw_rgb, self.rotation_degrees)
        success, centers = find_circle_grid(image, self.columns, self.rows)
        preview = image.copy()
        cv2.drawChessboardCorners(preview, (self.columns, self.rows), centers, success)
        if not success or centers is None:
            return DetectionResult(False, "未识别到完整圆点阵列，本帧未保存", preview)

        current_size = (image.shape[1], image.shape[0])
        if self.image_size is not None and current_size != self.image_size:
            return DetectionResult(False, "图像分辨率与本会话之前的采样不一致", preview)
        self.image_size = current_size
        self.images_folder.mkdir(parents=True, exist_ok=True)
        self.previews_folder.mkdir(parents=True, exist_ok=True)
        number = self.count + 1
        while (self.images_folder / f"{number:03d}.png").exists():
            number += 1
        image_path = self.images_folder / f"{number:03d}.png"
        preview_path = self.previews_folder / f"{number:03d}.png"
        cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(preview_path), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
        self.image_paths.append(image_path)
        self.image_points.append(centers.astype(np.float32))
        self._save_metadata()
        return DetectionResult(True, f"已保存第 {number} 张有效标定图", preview, image_path)

    def calibrate(self, minimum_images: int = 12) -> CalibrationResult:
        if self.count < minimum_images:
            raise ValueError(f"至少需要 {minimum_images} 张有效图片，当前只有 {self.count} 张")
        if self.image_size is None:
            raise ValueError("没有有效图像尺寸")

        object_template = np.zeros((self.columns * self.rows, 3), np.float32)
        grid = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)
        object_template[:, :2] = grid * float(self.spacing_mm)
        object_points = [object_template.copy() for _ in self.image_points]

        criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)
        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points, self.image_points, self.image_size, None, None,
            flags=cv2.CALIB_FIX_K3, criteria=criteria)

        per_view: list[float] = []
        error_sum = 0.0
        point_count = 0
        for object_set, image_set, rvec, tvec in zip(
                object_points, self.image_points, rvecs, tvecs):
            projected, _ = cv2.projectPoints(object_set, rvec, tvec, matrix, distortion)
            differences = image_set.reshape(-1, 2) - projected.reshape(-1, 2)
            distances = np.linalg.norm(differences, axis=1)
            per_view.append(float(np.sqrt(np.mean(np.square(distances)))))
            error_sum += float(np.sum(distances))
            point_count += len(distances)
        mean_error = error_sum / point_count

        output = {
            "model": "opencv_pinhole",
            "image_width": self.image_size[0],
            "image_height": self.image_size[1],
            "rotation_degrees_clockwise": self.rotation_degrees,
            "pattern_cols": self.columns,
            "pattern_rows": self.rows,
            "center_distance_mm": float(self.spacing_mm),
            "images_used": self.count,
            "opencv_rms": float(rms),
            "mean_reprojection_error_px": float(mean_error),
            "per_view_rmse_px": per_view,
            "camera_matrix": matrix.reshape(-1).astype(float).tolist(),
            "distort_coeffs": distortion.reshape(-1)[:5].astype(float).tolist(),
        }
        self.folder.mkdir(parents=True, exist_ok=True)
        output_path = self.folder / "camera_calibration.yaml"
        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, allow_unicode=True, sort_keys=False)
        return CalibrationResult(matrix, distortion, float(rms), float(mean_error),
                                 per_view, self.image_size, output_path)
