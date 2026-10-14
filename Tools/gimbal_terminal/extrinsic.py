from __future__ import annotations

import datetime as dt
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from calibration import find_circle_grid, rotate_rgb
from protocol import AttitudeSample


# Rotated display/PnP camera coordinates are x right, y down, z forward.
# The mechanism frame is X forward, Y left, Z up.  A planar symmetric target
# cannot by itself distinguish the two camera-normal branches, so use the
# known physical mounting as the branch-selection prior.
NOMINAL_CAMERA2GIMBAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)
PLANAR_NORMAL_FLIP = np.diag([-1.0, 1.0, -1.0])


def resolve_mount_orientation(rotation: np.ndarray) -> tuple[np.ndarray, bool]:
    """Select the physically possible branch of a planar hand-eye result."""
    candidate = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    flipped = candidate @ PLANAR_NORMAL_FLIP
    direct_error = np.linalg.norm(candidate-NOMINAL_CAMERA2GIMBAL)
    flipped_error = np.linalg.norm(flipped-NOMINAL_CAMERA2GIMBAL)
    return (flipped, True) if flipped_error < direct_error else (candidate, False)


def validate_mount_orientation(rotation: np.ndarray) -> None:
    """Reject an extrinsic that points the camera away from gimbal forward."""
    value = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    camera_forward = value[:, 2]
    camera_right = value[:, 0]
    camera_down = value[:, 1]
    if (camera_forward[0] < 0.7 or camera_right[1] > -0.7 or
            camera_down[2] > -0.7):
        raise ValueError(
            "外参方向与机械安装不符：要求相机前方→云台+X、"
            "相机右方→云台-Y、相机下方→云台-Z")


def quaternion_matrix(wxyz: tuple[float, float, float, float]) -> np.ndarray:
    q = np.asarray(wxyz, dtype=float)
    q /= max(1e-12, np.linalg.norm(q))
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class ExtrinsicCalibrationSession:
    def __init__(self, base: Path, camera_matrix: np.ndarray, distortion: np.ndarray,
                 columns: int = 10, rows: int = 7, spacing_mm: float = 20.0) -> None:
        self.folder = base / dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.camera_matrix, self.distortion = camera_matrix, distortion
        self.columns, self.rows, self.spacing_mm = columns, rows, spacing_mm
        self.camera_poses: list[tuple[np.ndarray, np.ndarray]] = []
        self.gimbal_rotations: list[np.ndarray] = []
        self.sample_numbers: list[int] = []
        self.last_error = ""

    @property
    def count(self) -> int:
        return len(self.camera_poses)

    @staticmethod
    def canonical_centers(centers: np.ndarray) -> np.ndarray:
        """Resolve the 180-degree ordering ambiguity of a symmetric grid."""
        ordered = np.asarray(centers, dtype=np.float32).reshape(-1, 2)
        if float(np.sum(ordered[0])) > float(np.sum(ordered[-1])):
            ordered = ordered[::-1].copy()
        return ordered.reshape(-1, 1, 2)

    def capture(self, raw_rgb: np.ndarray, attitude: AttitudeSample) -> bool:
        image = rotate_rgb(raw_rgb, 90)
        success, centers = find_circle_grid(image, self.columns, self.rows)
        if not success or centers is None:
            self.last_error = "未识别完整圆点板"
            return False
        centers = self.canonical_centers(centers)
        obj = np.zeros((self.columns*self.rows, 3), np.float32)
        grid = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)
        obj[:, :2] = grid * self.spacing_mm
        ok, rvec, tvec = cv2.solvePnP(obj, centers, self.camera_matrix,
                                      self.distortion, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            self.last_error = "圆点PnP求解失败"
            return False
        projected, _ = cv2.projectPoints(
            obj, rvec, tvec, self.camera_matrix, self.distortion)
        reprojection_error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(-1, 2)-centers.reshape(-1, 2))**2, axis=1))))
        if reprojection_error > 0.5:
            self.last_error = f"圆点重投影误差过大：{reprojection_error:.3f} px"
            return False
        self.camera_poses.append((rvec, tvec))
        self.gimbal_rotations.append(quaternion_matrix(attitude.quaternion_wxyz).T)
        self.folder.mkdir(parents=True, exist_ok=True)
        number = self.count
        self.sample_numbers.append(number)
        cv2.imwrite(str(self.folder / f"{number:03d}.png"),
                    cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        np.savez(self.folder / f"{number:03d}.npz", centers=centers,
                 quaternion=np.asarray(attitude.quaternion_wxyz),
                 mcu_time_ms=attitude.mcu_time_ms,
                 host_time_ns=attitude.host_time_ns,
                 pitch_rate_mdeg_s=attitude.pitch_rate_mdeg_s,
                 yaw_rate_mdeg_s=attitude.yaw_rate_mdeg_s,
                 temperature_cdeg=attitude.temperature_cdeg,
                 calibration_temperature_cdeg=attitude.calibration_temperature_cdeg,
                 gyro_offset_urad_s=np.asarray(attitude.gyro_offset_urad_s),
                 warmup_timed_out=bool(attitude.flags & (1 << 8)),
                 reprojection_error_px=reprojection_error)
        self.last_error = ""
        return True

    @classmethod
    def restore(cls, folder: Path, camera_matrix: np.ndarray,
                distortion: np.ndarray, columns: int = 10, rows: int = 7,
                spacing_mm: float = 20.0) -> "ExtrinsicCalibrationSession":
        session = cls(folder.parent, camera_matrix, distortion,
                      columns, rows, spacing_mm)
        session.folder = folder
        obj = np.zeros((columns * rows, 3), np.float32)
        grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
        obj[:, :2] = grid * spacing_mm
        for path in sorted(folder.glob("*.npz")):
            saved = np.load(path)
            centers = session.canonical_centers(saved["centers"])
            ok, rvec, tvec = cv2.solvePnP(
                obj, centers, camera_matrix, distortion,
                flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue
            session.camera_poses.append((rvec, tvec))
            session.gimbal_rotations.append(
                quaternion_matrix(tuple(saved["quaternion"])).T)
            session.sample_numbers.append(int(path.stem))
        return session

    def solve(self, minimum: int = 25,
              enforce_mount_prior: bool = True) -> Path:
        if self.count < minimum:
            raise ValueError(f"外参至少需要 {minimum} 个姿态，当前 {self.count} 个")
        if len(self.sample_numbers) != self.count:
            self.sample_numbers = list(range(1, self.count + 1))
        rotations = [cv2.Rodrigues(item[0])[0] for item in self.camera_poses]
        translations = [item[1] for item in self.camera_poses]
        zeros = [np.zeros((3, 1), np.float64) for _ in self.gimbal_rotations]
        active, rejected = self._reject_outliers(rotations, translations, zeros)
        if len(active) < minimum:
            raise ValueError("鲁棒筛选后有效外参姿态不足25个")
        holdout = active[4::5]
        training = [index for index in active if index not in holdout]
        validation_solution = cv2.calibrateRobotWorldHandEye(
            [rotations[index] for index in training],
            [translations[index] for index in training],
            [self.gimbal_rotations[index] for index in training],
            [zeros[index] for index in training])
        validation = self._validate(rotations, translations, zeros,
                                    validation_solution, holdout)
        development_max_error_deg = 0.8
        development_rms_error_deg = 0.5
        if (validation["max_rotation_error_deg"] > development_max_error_deg or
                validation["rms_rotation_error_deg"] > development_rms_error_deg):
            raise ValueError(
                "外参留出姿态误差未通过开发门槛："
                f"最大 {validation['max_rotation_error_deg']:.3f}° / "
                f"RMS {validation['rms_rotation_error_deg']:.3f}°，"
                "要求最大 < 0.8° 且 RMS < 0.5°")
        r_world_board, t_world_board, r_gimbal_camera, t_gimbal_camera = (
            cv2.calibrateRobotWorldHandEye(
                [rotations[index] for index in active],
                [translations[index] for index in active],
                [self.gimbal_rotations[index] for index in active],
                [zeros[index] for index in active]))
        r_camera_gimbal = r_gimbal_camera.T
        orientation_corrected = False
        if enforce_mount_prior:
            r_camera_gimbal, orientation_corrected = resolve_mount_orientation(
                r_camera_gimbal)
            validate_mount_orientation(r_camera_gimbal)
        # Translation belongs to the inverse rigid transform and must be
        # recomputed after choosing the rotation branch.
        t_camera_gimbal = -r_camera_gimbal @ t_gimbal_camera / 1000.0
        output = {
            "model": "robot_world_hand_eye",
            "provisional": True,
            "validation_policy": "development_max_lt_0.8deg_rms_lt_0.5deg",
            "final_acceptance_max_rotation_error_deg": 0.5,
            "samples": self.count,
            "used_samples": len(active),
            "used_sample_numbers": [self.sample_numbers[index]
                                    for index in active],
            "rejected_sample_numbers": [self.sample_numbers[index]
                                          for index in rejected],
            "mount_orientation_prior": (
                "camera_forward_to_gimbal_positive_x" if enforce_mount_prior
                else "disabled"),
            "planar_normal_branch_corrected": bool(orientation_corrected),
            "R_camera2gimbal": r_camera_gimbal.reshape(-1).tolist(),
            "t_camera2gimbal": t_camera_gimbal.reshape(-1).tolist(),
            "R_board2world": r_world_board.T.reshape(-1).tolist(),
            "t_board2world_m": (-r_world_board.T @ t_world_board / 1000.0).reshape(-1).tolist(),
            "holdout_samples": len(holdout),
            "holdout_max_rotation_error_deg": validation["max_rotation_error_deg"],
            "holdout_rms_rotation_error_deg": validation["rms_rotation_error_deg"],
            "holdout_rms_position_error_mm": validation["rms_position_error_mm"],
        }
        self.folder.mkdir(parents=True, exist_ok=True)
        path = self.folder / "camera_to_gimbal.yaml"
        with path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False)
        return path

    def _reject_outliers(self, rotations: list[np.ndarray],
                         translations: list[np.ndarray],
                         zeros: list[np.ndarray]) -> tuple[list[int], list[int]]:
        active = list(range(self.count))
        rejected: list[int] = []
        reject_limit = max(1, self.count // 10)
        while len(rejected) < reject_limit and len(active) > 25:
            solution = cv2.calibrateRobotWorldHandEye(
                [rotations[index] for index in active],
                [translations[index] for index in active],
                [self.gimbal_rotations[index] for index in active],
                [zeros[index] for index in active])
            errors = self._rotation_errors(rotations, translations, zeros,
                                           solution, active)
            values = np.asarray([errors[index] for index in active])
            median = float(np.median(values))
            mad = float(np.median(np.abs(values-median)))
            threshold = max(1.0, median + 3.0 * 1.4826 * max(mad, 1e-6))
            worst = max(active, key=errors.get)
            if errors[worst] <= threshold:
                break
            active.remove(worst)
            rejected.append(worst)
        return active, rejected

    def _rotation_errors(self, rotations: list[np.ndarray],
                         translations: list[np.ndarray], zeros: list[np.ndarray],
                         solution: tuple[np.ndarray, ...],
                         indices: list[int]) -> dict[int, float]:
        r_base_world, t_base_world, r_gimbal_camera, t_gimbal_camera = solution
        base_world = self._transform(r_base_world, t_base_world)
        gimbal_camera = self._transform(r_gimbal_camera, t_gimbal_camera)
        inverse_base_world = np.linalg.inv(base_world)
        result: dict[int, float] = {}
        for index in indices:
            predicted = (gimbal_camera @
                         self._transform(self.gimbal_rotations[index], zeros[index]) @
                         inverse_base_world)
            delta_rotation = predicted[:3, :3] @ rotations[index].T
            cosine = float(np.clip((np.trace(delta_rotation)-1.0)*0.5,
                                   -1.0, 1.0))
            result[index] = math.degrees(math.acos(cosine))
        return result

    def _validate(self, rotations: list[np.ndarray], translations: list[np.ndarray],
                  zeros: list[np.ndarray], solution: tuple[np.ndarray, ...],
                  indices: list[int]) -> dict[str, float]:
        r_base_world, t_base_world, r_gimbal_camera, t_gimbal_camera = solution
        base_world = self._transform(r_base_world, t_base_world)
        gimbal_camera = self._transform(r_gimbal_camera, t_gimbal_camera)
        inverse_base_world = np.linalg.inv(base_world)
        angle_errors: list[float] = []
        position_errors: list[float] = []
        for index in indices:
            base_gimbal = self._transform(self.gimbal_rotations[index], zeros[index])
            predicted = gimbal_camera @ base_gimbal @ inverse_base_world
            measured = self._transform(rotations[index], translations[index])
            delta = predicted @ np.linalg.inv(measured)
            cosine = float(np.clip((np.trace(delta[:3, :3]) - 1.0) * 0.5,
                                   -1.0, 1.0))
            angle_errors.append(math.degrees(math.acos(cosine)))
            position_errors.append(float(np.linalg.norm(delta[:3, 3])))
        return {
            "max_rotation_error_deg": max(angle_errors),
            "rms_rotation_error_deg": math.sqrt(np.mean(np.square(angle_errors))),
            "rms_position_error_mm": math.sqrt(np.mean(np.square(position_errors))),
        }

    @staticmethod
    def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
        return result
