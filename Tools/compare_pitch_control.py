#!/usr/bin/env python3
"""Compare Pitch position tracking with and without gravity feedforward."""

import argparse
import csv
import json
import math
from pathlib import Path


def load(path, np):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    column = lambda name: np.asarray([float(row[name]) for row in rows])
    return {name: column(name) for name in rows[0]}


def metrics(data, np):
    error_deg = np.degrees(data["target_pitch_rad"] - data["imu_pitch_rad"])
    ramp = data["phase"] == 1
    hold = data["phase"] == 2
    ramp_error = error_deg[ramp]
    hold_error = error_deg[hold]
    return {
        "samples": int(len(error_deg)),
        "tracking_rmse_deg": float(np.sqrt(np.mean(error_deg ** 2))),
        "ramp_rmse_deg": float(np.sqrt(np.mean(ramp_error ** 2))),
        "ramp_mae_deg": float(np.mean(np.abs(ramp_error))),
        "ramp_p95_abs_error_deg": float(np.percentile(np.abs(ramp_error), 95)),
        "ramp_max_abs_error_deg": float(np.max(np.abs(ramp_error))),
        "hold_rmse_deg": float(np.sqrt(np.mean(hold_error ** 2))),
        "hold_mae_deg": float(np.mean(np.abs(hold_error))),
        "hold_p95_abs_error_deg": float(np.percentile(np.abs(hold_error), 95)),
        "hold_max_abs_error_deg": float(np.max(np.abs(hold_error))),
        "hold_steady_bias_deg": float(np.mean(hold_error)),
        "gyro_rms_rad_s": float(np.sqrt(np.mean(data["gyro_y_rad_s"] ** 2))),
        "gyro_peak_rad_s": float(np.max(np.abs(data["gyro_y_rad_s"]))),
        "pid_current_rms": float(np.sqrt(np.mean(data["pid_current"] ** 2))),
        "total_current_rms": float(np.sqrt(np.mean(data["output_current"] ** 2))),
    }


def improvement(before, after, key):
    return 100.0 * (before[key] - after[key]) / before[key] if before[key] else float("nan")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--pid", type=Path, help="Legacy alias for --baseline")
    parser.add_argument("--feedforward", type=Path, help="Legacy alias for --candidate")
    parser.add_argument("--output-prefix", required=True, type=Path)
    args = parser.parse_args()
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit("Install numpy first: python -m pip install numpy") from exc
    baseline_path = args.baseline or args.pid
    candidate_path = args.candidate or args.feedforward
    if baseline_path is None or candidate_path is None:
        parser.error("provide --baseline and --candidate")
    pid_data, ff_data = load(baseline_path, np), load(candidate_path, np)
    pid_metrics, ff_metrics = metrics(pid_data, np), metrics(ff_data, np)
    keys = ["tracking_rmse_deg", "ramp_rmse_deg", "ramp_mae_deg",
            "ramp_p95_abs_error_deg", "ramp_max_abs_error_deg",
            "hold_rmse_deg", "hold_mae_deg",
            "hold_p95_abs_error_deg", "hold_max_abs_error_deg"]
    summary = {
        "schema": "pitch-control-comparison-v1",
        "baseline_csv": str(baseline_path.resolve()),
        "candidate_csv": str(candidate_path.resolve()),
        "score_excludes_control_effort": True,
        "baseline": pid_metrics, "candidate": ff_metrics,
        "improvement_percent": {key: improvement(pid_metrics, ff_metrics, key)
                                for key in keys},
    }
    prefix = args.output_prefix
    prefix.parent.mkdir(parents=True, exist_ok=True)
    summary_path = prefix.with_name(prefix.name + "_summary.json")
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
        for data, label in ((pid_data, "Baseline"), (ff_data, "Candidate")):
            time_s = (data["timestamp_ms"] - data["timestamp_ms"][0]) / 1000.0
            axes[0].plot(time_s, np.degrees(data["imu_pitch_rad"]), label=label, linewidth=1)
            axes[1].plot(time_s,
                         np.degrees(data["target_pitch_rad"] - data["imu_pitch_rad"]),
                         label=label, linewidth=1)
        pid_time = (pid_data["timestamp_ms"] - pid_data["timestamp_ms"][0]) / 1000.0
        axes[0].plot(pid_time, np.degrees(pid_data["target_pitch_rad"]), "k--",
                     label="Target", linewidth=1)
        axes[0].set_ylabel("Pitch (deg)"); axes[1].set_ylabel("Error (deg)")
        axes[1].set_xlabel("Time (s)")
        for axis in axes: axis.grid(True, alpha=0.3); axis.legend()
        fig.tight_layout()
        fig.savefig(prefix.with_name(prefix.name + "_tracking.png"), dpi=160)
        plt.close(fig)
    except ImportError:
        pass
    print(f"Baseline ramp RMSE: {pid_metrics['ramp_rmse_deg']:.4f} deg")
    print(f"Candidate ramp RMSE: {ff_metrics['ramp_rmse_deg']:.4f} deg")
    print(f"Ramp improvement: {summary['improvement_percent']['ramp_rmse_deg']:.2f}%")
    print(f"Hold RMSE: {pid_metrics['hold_rmse_deg']:.4f} -> "
          f"{ff_metrics['hold_rmse_deg']:.4f} deg")
    print(f"Saved {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
