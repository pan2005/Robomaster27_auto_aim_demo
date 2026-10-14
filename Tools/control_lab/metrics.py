from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd


def velocity_metrics(csv_path: str | Path) -> dict[str, float | int | bool]:
    path = Path(csv_path)
    data = pd.read_csv(path)
    if data.empty:
        raise ValueError(f"No samples in {path}")

    error = data["target_rpm"].to_numpy(float) - data["speed_rpm"].to_numpy(float)
    target = data["target_rpm"].to_numpy(float)
    current = data["output_current"].to_numpy(float)
    moving = np.abs(target) >= 1.0
    steady = np.abs(target) >= 0.99 * max(1.0, np.max(np.abs(target)))
    zero = np.abs(target) < 0.01
    dt = float(np.median(np.diff(data["timestamp_ms"].to_numpy(float))) / 1000.0)

    moving_rmse = float(np.sqrt(np.mean(error[moving] ** 2))) if moving.any() else 0.0
    zero_peak = float(np.max(np.abs(data.loc[zero, "speed_rpm"]))) if zero.any() else 0.0
    zero_mean = float(np.mean(np.abs(data.loc[zero, "speed_rpm"]))) if zero.any() else 0.0
    steady_error = float(np.mean(np.abs(error[steady]))) if steady.any() else 0.0
    current_rms = float(np.sqrt(np.mean(current ** 2)))
    control_tv = float(np.sum(np.abs(np.diff(current))))
    tv_per_sample = control_tv / max(1, len(data) - 1)

    # Tracking-only objective. Current RMS and control variation remain logged
    # diagnostics and hard-limit gates, but are deliberately not soft costs:
    # RoboMaster gimbals are not power-limited and should trade actuator effort
    # for the best achievable tracking inside immutable safety boundaries.
    score = 100.0 * (
        0.40 * moving_rmse / 8.0
        + 0.20 * float(np.max(np.abs(error))) / 8.0
        + 0.20 * steady_error / 8.0
        + 0.10 * zero_peak / 8.0
        + 0.10 * zero_mean / 8.0
    )
    flags = int(data["safety_flags"].max())
    metrics = {
        "score": float(score),
        "score_version": "tracking_only_v2",
        "rmse_rpm": float(np.sqrt(np.mean(error ** 2))),
        "moving_rmse_rpm": moving_rmse,
        "iae_rpm_s": float(np.sum(np.abs(error)) * dt),
        "max_error_rpm": float(np.max(np.abs(error))),
        "steady_error_rpm": steady_error,
        "zero_peak_rpm": zero_peak,
        "zero_mean_rpm": zero_mean,
        "current_rms": current_rms,
        "current_peak": float(np.max(np.abs(current))),
        "control_tv": control_tv,
        "pid_rms": float(np.sqrt(np.mean(data["pid_output"].to_numpy(float) ** 2))),
        "pitch_range_deg": float(np.rad2deg(data["imu_pitch_rad"].max()
                                                   - data["imu_pitch_rad"].min())),
        "initial_pitch_deg": float(np.rad2deg(data["imu_pitch_rad"].iloc[0])),
        "yaw_net_deg": float(np.rad2deg(data["motor_angle_rad"].iloc[-1]
                                         - data["motor_angle_rad"].iloc[0])),
        "samples": int(len(data)),
        "safety_flags": flags,
        "valid": bool(flags == 0 and np.all(np.diff(data["sample_index"]) == 1)),
    }
    json_path = path.with_suffix(".json")
    if json_path.exists():
        metadata = json.loads(json_path.read_text(encoding="utf-8"))
        metrics["crc_errors"] = int(metadata.get("crc_errors", 0))
        metrics["missing_samples"] = int(metadata.get("host_detected_missing_samples", 0))
        metrics["valid"] = bool(metrics["valid"] and
                                metrics["crc_errors"] == 0 and
                                metrics["missing_samples"] == 0)
    return metrics
