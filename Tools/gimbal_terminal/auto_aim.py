from __future__ import annotations

import dataclasses
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import yaml
import cv2

from apriltag import TagPose
from extrinsic import validate_mount_orientation
from protocol import (AttitudeSample, STATUS_IMU_VALID,
                      STATUS_IMU_WARMUP_TIMEOUT, STATUS_SAFE)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _gimbal_to_world(yaw: float, pitch: float) -> np.ndarray:
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    ry = np.array([[cp, 0, -sp], [0, 1.0, 0], [sp, 0, cp]])
    return rz @ ry


class AttitudeBuffer:
    def __init__(self) -> None:
        self._samples: deque[AttitudeSample] = deque(maxlen=256)
        self._lock = threading.Lock()

    def put(self, sample: AttitudeSample) -> None:
        with self._lock:
            self._samples.append(sample)

    def at(self, host_time_ns: int) -> AttitudeSample | None:
        with self._lock:
            samples = sorted(self._samples, key=lambda item: item.host_time_ns)
        if not samples:
            return None
        if host_time_ns <= samples[0].host_time_ns:
            return (samples[0] if samples[0].host_time_ns-host_time_ns <= 20_000_000
                    else None)
        for left, right in zip(samples, samples[1:]):
            if left.host_time_ns <= host_time_ns <= right.host_time_ns:
                span = max(1, right.host_time_ns - left.host_time_ns)
                k = (host_time_ns - left.host_time_ns) / span
                left_q = np.asarray(left.quaternion_wxyz, dtype=float)
                right_q = np.asarray(right.quaternion_wxyz, dtype=float)
                if np.dot(left_q, right_q) < 0.0:
                    right_q = -right_q
                quaternion = left_q + k * (right_q - left_q)
                quaternion /= max(1e-12, np.linalg.norm(quaternion))
                return dataclasses.replace(
                    left,
                    quaternion_wxyz=tuple(float(value) for value in quaternion),
                    pitch_mdeg=round(left.pitch_mdeg + k * (right.pitch_mdeg-left.pitch_mdeg)),
                    yaw_mdeg=round(left.yaw_mdeg + k * (right.yaw_mdeg-left.yaw_mdeg)),
                    pitch_rate_mdeg_s=round(left.pitch_rate_mdeg_s + k * (
                        right.pitch_rate_mdeg_s-left.pitch_rate_mdeg_s)),
                    yaw_rate_mdeg_s=round(left.yaw_rate_mdeg_s + k * (
                        right.yaw_rate_mdeg_s-left.yaw_rate_mdeg_s)),
                    temperature_cdeg=round(left.temperature_cdeg + k * (
                        right.temperature_cdeg-left.temperature_cdeg)),
                    calibration_temperature_cdeg=round(
                        left.calibration_temperature_cdeg + k * (
                            right.calibration_temperature_cdeg-
                            left.calibration_temperature_cdeg)),
                    host_time_ns=host_time_ns)
        return (samples[-1] if host_time_ns-samples[-1].host_time_ns <= 20_000_000
                else None)

    def nearest_delta_ms(self, host_time_ns: int) -> float | None:
        """Signed timestamp delta (attitude - image) of the nearest sample."""
        with self._lock:
            if not self._samples:
                return None
            nearest = min(self._samples,
                          key=lambda item: abs(item.host_time_ns-host_time_ns))
        return (nearest.host_time_ns-host_time_ns) * 1e-6

    def capture_quality(self, host_time_ns: int) -> tuple[AttitudeSample | None, str]:
        attitude = self.at(host_time_ns)
        if attitude is None:
            return None, "图像与姿态时间差超过20 ms"
        with self._lock:
            samples = sorted(self._samples, key=lambda item: item.host_time_ns)
        window_start = host_time_ns - 300_000_000
        window = [item for item in samples
                  if window_start <= item.host_time_ns <= host_time_ns]
        if len(window) < 20 or window[-1].host_time_ns-window[0].host_time_ns < 250_000_000:
            return None, "静止观测不足300 ms"
        rates = np.asarray([
            max(abs(item.pitch_rate_mdeg_s), abs(item.yaw_rate_mdeg_s))
            for item in window], dtype=float)
        peak_rate = float(np.max(rates))
        rms_rate = float(np.sqrt(np.mean(np.square(rates))))
        if rms_rate > 800 or peak_rate > 1500:
            return None, ("云台未完全停稳："
                          f"RMS {rms_rate/1000:.2f}°/s，"
                          f"峰值 {peak_rate/1000:.2f}°/s")
        if (attitude.calibration_temperature_cdeg and
                abs(attitude.temperature_cdeg-
                    attitude.calibration_temperature_cdeg) > 150):
            return None, "IMU当前温度偏离零偏标定温度超过1.5℃"
        warning = ("；注意：启动加热曾超时" if
                   attitude.flags & STATUS_IMU_WARMUP_TIMEOUT else "")
        return attitude, (f"质量门控通过（角速度RMS {rms_rate/1000:.2f}°/s）" +
                          warning)

    def latest(self) -> AttitudeSample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None


class ConstantAccelerationFilter:
    def __init__(self, jerk_sigma: float = 15.0) -> None:
        self.x = np.zeros(9)
        self.p = np.eye(9) * 10.0
        self.jerk_sigma = jerk_sigma
        self.last_time_ns: int | None = None
        self.initialized = False

    def update(self, position: np.ndarray, time_ns: int,
               measurement_sigma: float = 0.015) -> None:
        if not self.initialized:
            self.x[:3] = position
            self.p = np.eye(9)
            self.last_time_ns = time_ns
            self.initialized = True
            return
        self.predict_to(time_ns)
        h = np.zeros((3, 9)); h[:, :3] = np.eye(3)
        r = np.eye(3) * measurement_sigma**2
        innovation = position - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x += k @ innovation
        self.p = (np.eye(9) - k @ h) @ self.p

    def predict_to(self, time_ns: int) -> None:
        if self.last_time_ns is None:
            self.last_time_ns = time_ns
            return
        dt = max(0.0, min(0.2, (time_ns - self.last_time_ns) * 1e-9))
        f = np.eye(9)
        for axis in range(3):
            f[axis, axis+3] = dt
            f[axis, axis+6] = 0.5 * dt * dt
            f[axis+3, axis+6] = dt
        q = np.eye(9) * (self.jerk_sigma**2 * max(dt, 1e-4)**3)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q
        self.last_time_ns = time_ns

    def position_at(self, time_ns: int) -> np.ndarray:
        if not self.initialized or self.last_time_ns is None:
            raise ValueError("filter is not initialized")
        dt = max(0.0, min(1.0, (time_ns - self.last_time_ns) * 1e-9))
        return self.x[:3] + self.x[3:6] * dt + 0.5 * self.x[6:9] * dt * dt


class MpcTrajectoryPlanner:
    """Condensed finite-horizon double-integrator planner."""
    def __init__(self, dt: float = 0.01, horizon: int = 100,
                 q_position: float = 9e6, r_accel: float = 1.0) -> None:
        self.dt, self.horizon = dt, horizon
        t = np.tril(np.ones((horizon, horizon))) * dt * dt
        t -= np.eye(horizon) * (0.5 * dt * dt)
        self._gain = np.linalg.solve(
            q_position * (t.T @ t) + r_accel * np.eye(horizon),
            q_position * t.T)

    def axis(self, position: float, velocity: float, target: np.ndarray,
             max_velocity: float, max_acceleration: float) -> tuple[float, float, float, bool]:
        times = np.arange(1, self.horizon + 1) * self.dt
        base = position + velocity * times
        acceleration = np.clip(self._gain @ (target - base),
                               -max_acceleration, max_acceleration)
        new_velocity = float(np.clip(velocity + acceleration[0] * self.dt,
                                     -max_velocity, max_velocity))
        new_position = position + new_velocity * self.dt
        saturated = abs(acceleration[0]) >= max_acceleration * 0.999 or abs(new_velocity) >= max_velocity * 0.999
        return new_position, new_velocity, float(acceleration[0]), saturated


@dataclasses.dataclass(frozen=True)
class AimCommand:
    valid: bool
    yaw: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_accel: float = 0.0
    pitch_accel: float = 0.0
    distance: float = 0.0
    fire: bool = False
    saturated: bool = False
    gate_reason: str = "无目标"
    target_world: np.ndarray | None = None
    aim_error_rad: float = math.inf
    tracking_error_rad: float = math.inf


class AutoAimEngine:
    def __init__(self, attitudes: AttitudeBuffer, bullet_speed: float = 15.0) -> None:
        self._lock = threading.Lock()
        self.attitudes = attitudes
        self.filter = ConstantAccelerationFilter()
        self.planner = MpcTrajectoryPlanner()
        self.bullet_speed = bullet_speed
        self.processing_delay_s = 0.020
        self.launch_delay_s = 0.100
        self.camera_to_gimbal_r = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        self.camera_to_gimbal_t = np.zeros(3)
        self.extrinsic_valid = False
        self.extrinsic_error = "缺少相机—云台外参"
        self.last_observation_ns = 0
        self.plan_yaw: float | None = None
        self.plan_pitch: float | None = None
        self.plan_yaw_rate = self.plan_pitch_rate = 0.0
        self._gate_since_ns: int | None = None
        self.last_observation_error = "尚未收到合格目标观测"

    def load_extrinsic(self, path: Path) -> None:
        if not path.is_file():
            return
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        rotation = np.asarray(
            data["R_camera2gimbal"], dtype=float).reshape(3, 3)
        try:
            validate_mount_orientation(rotation)
        except ValueError as exc:
            with self._lock:
                self.extrinsic_valid = False
                self.extrinsic_error = str(exc)
            return
        with self._lock:
            self.camera_to_gimbal_r = rotation
            self.camera_to_gimbal_t = np.asarray(
                data["t_camera2gimbal"], dtype=float).reshape(3)
            self.extrinsic_valid = True
            self.extrinsic_error = ""

    def reset(self) -> None:
        with self._lock:
            self.filter.initialized = False
            self.filter.last_time_ns = None
            self.plan_yaw = self.plan_pitch = None
            self.plan_yaw_rate = self.plan_pitch_rate = 0.0
            self._gate_since_ns = None
            self.last_observation_error = "等待锁定目标的首帧观测"

    def observe(self, pose: TagPose) -> bool:
        attitude = self.attitudes.at(pose.capture_time_ns)
        if attitude is None:
            delta_ms = self.attitudes.nearest_delta_ms(pose.capture_time_ns)
            with self._lock:
                self.last_observation_error = (
                    "没有云台姿态数据" if delta_ms is None else
                    f"图像与姿态未对齐（最近差 {delta_ms:+.1f} ms）")
            return False
        yaw, pitch = math.radians(attitude.yaw_mdeg/1000), math.radians(attitude.pitch_mdeg/1000)
        point_gimbal = self.camera_to_gimbal_r @ pose.tvec_m + self.camera_to_gimbal_t
        point_world = _gimbal_to_world(yaw, pitch) @ point_gimbal
        with self._lock:
            self.filter.update(point_world, pose.capture_time_ns,
                               0.005 + 0.01 * pose.reprojection_error_px)
            self.last_observation_ns = pose.capture_time_ns
            self.last_observation_error = ""
        return True

    def project_world_point(self, point_world: np.ndarray,
                            attitude: AttitudeSample,
                            camera_matrix: np.ndarray,
                            distortion: np.ndarray) -> tuple[float, float] | None:
        yaw = math.radians(attitude.yaw_mdeg / 1000.0)
        pitch = math.radians(attitude.pitch_mdeg / 1000.0)
        point_gimbal = _gimbal_to_world(yaw, pitch).T @ np.asarray(point_world)
        point_camera = self.camera_to_gimbal_r.T @ (
            point_gimbal - self.camera_to_gimbal_t)
        if point_camera[2] <= 1e-4:
            return None
        pixels, _ = cv2.projectPoints(
            point_camera.reshape(1, 3), np.zeros(3), np.zeros(3),
            camera_matrix, distortion)
        pixel = pixels.reshape(2)
        return float(pixel[0]), float(pixel[1])

    def command(self, now_ns: int, fire_authorized: bool) -> AimCommand:
        with self._lock:
            return self._command_unlocked(now_ns, fire_authorized)

    def _command_unlocked(self, now_ns: int, fire_authorized: bool) -> AimCommand:
        attitude = self.attitudes.latest()
        if attitude is None:
            return AimCommand(False, gate_reason="尚未收到云台姿态")
        if not self.filter.initialized:
            return AimCommand(False, gate_reason=self.last_observation_error)
        if not self.extrinsic_valid:
            return AimCommand(False, gate_reason=self.extrinsic_error)
        attitude_age_s = (now_ns - attitude.host_time_ns) * 1e-9
        if (attitude_age_s < 0.0 or attitude_age_s > 0.050 or
                not (attitude.flags & STATUS_IMU_VALID) or
                not (attitude.flags & STATUS_SAFE)):
            return AimCommand(False, gate_reason="云台姿态过期或不安全")
        age_s = (now_ns - self.last_observation_ns) * 1e-9
        if age_s > 0.150:
            return AimCommand(False, gate_reason="目标丢失")
        actual_yaw = math.radians(attitude.yaw_mdeg / 1000.0)
        actual_pitch = math.radians(attitude.pitch_mdeg / 1000.0)
        flight = 0.0
        target = self.filter.position_at(now_ns)
        observed_distance = float(np.linalg.norm(target))
        if not 1.0 <= observed_distance <= 4.0:
            return AimCommand(
                False, distance=observed_distance,
                gate_reason=f"目标距离 {observed_distance:.2f} m，跟踪范围为 1～4 m",
                target_world=target)
        pitch_target = 0.0
        for _ in range(4):
            impact_ns = now_ns + int((self.processing_delay_s + self.launch_delay_s + flight) * 1e9)
            target = self.filter.position_at(impact_ns)
            horizontal = max(1e-3, math.hypot(target[0], target[1]))
            speed2 = self.bullet_speed**2
            disc = speed2**2 - 9.80665 * (9.80665 * horizontal**2 + 2 * target[2] * speed2)
            pitch_target = (math.atan2(target[2], horizontal) if disc < 0 else
                            math.atan2(speed2 - math.sqrt(disc), 9.80665 * horizontal))
            flight = horizontal / max(1e-3, self.bullet_speed * math.cos(pitch_target))
        yaw_target = math.atan2(target[1], target[0])
        if self.plan_yaw is None:
            self.plan_yaw, self.plan_pitch = actual_yaw, actual_pitch
        yaw_path = np.full(100, self.plan_yaw + _wrap(yaw_target-self.plan_yaw))
        pitch_path = np.full(100, pitch_target)
        yaw, yv, ya, ys = self.planner.axis(self.plan_yaw, self.plan_yaw_rate, yaw_path, 1.68, 10.0)
        pitch, pv, pa, ps = self.planner.axis(self.plan_pitch, self.plan_pitch_rate, pitch_path, 3.2, 10.0)
        self.plan_yaw, self.plan_pitch = yaw, pitch
        self.plan_yaw_rate, self.plan_pitch_rate = yv, pv
        tracking_error = max(abs(_wrap(yaw-actual_yaw)), abs(pitch-actual_pitch))
        aim_error = max(abs(_wrap(yaw_target-actual_yaw)),
                        abs(pitch_target-actual_pitch))
        track_valid = (age_s <= 0.030 and not (ys or ps) and
                       aim_error <= math.radians(0.6) and
                       tracking_error <= math.radians(0.6))
        if track_valid and fire_authorized:
            self._gate_since_ns = self._gate_since_ns or now_ns
            fire = now_ns - self._gate_since_ns >= 30_000_000
            reason = "允许开火" if fire else "命中窗口稳定中"
        else:
            self._gate_since_ns = None
            fire = False
            reason = ("未按住左键" if not fire_authorized else
                      "观测过期" if age_s > 0.030 else
                      "轨迹饱和" if ys or ps else
                      "预测瞄准误差过大" if aim_error > math.radians(0.6) else
                      "云台跟踪误差过大")
        return AimCommand(True, yaw, pitch, yv, pv, ya, pa,
                          float(np.linalg.norm(target)), fire, ys or ps,
                          reason, target, aim_error, tracking_error)
