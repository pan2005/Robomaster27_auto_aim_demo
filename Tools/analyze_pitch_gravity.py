#!/usr/bin/env python3
"""Fit Pitch holding current versus angle, including direction-dependent friction."""

import argparse
import csv
import json
import math
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def settled_groups(rows: list[dict[str, float]]) -> list[list[dict[str, float]]]:
    groups: list[list[dict[str, float]]] = []
    current: list[dict[str, float]] = []
    last_target = None
    for row in rows:
        target = row["target_pitch_rad"]
        is_settled = int(row["phase"]) == 2 and int(row["safety_flags"]) == 0
        if is_settled and (last_target is None or abs(target - last_target) < 1e-5):
            current.append(row)
        else:
            if current:
                groups.append(current)
            current = [row] if is_settled else []
        last_target = target if is_settled else None
    if current:
        groups.append(current)
    return groups


def mean(values):
    return sum(values) / len(values)


def fit_model(point_rows, np):
    theta = np.radians([r["actual_pitch_deg"] for r in point_rows])
    direction = np.asarray([r["direction"] for r in point_rows], dtype=float)
    current = np.asarray([r["output_current_mean"] for r in point_rows], dtype=float)
    design = np.column_stack((np.sin(theta), np.cos(theta), np.ones_like(theta), direction))
    coeff, _, rank, _ = np.linalg.lstsq(design, current, rcond=None)
    prediction = design @ coeff
    residual = current - prediction
    r2_den = float(np.sum((current - np.mean(current)) ** 2))
    return coeff, residual, int(rank), (
        1.0 - float(np.sum(residual ** 2)) / r2_den if r2_den else float("nan"))


def fit_direction_paired_model(point_rows, np):
    """Cancel Coulomb friction by averaging opposite-direction holds."""
    buckets = {}
    for row in point_rows:
        target = round(row["target_pitch_deg"], 3)
        if row["direction"] in (-1, 1):
            buckets.setdefault(target, {})[row["direction"]] = row
    pairs = []
    for target, directions in sorted(buckets.items()):
        if -1 not in directions or 1 not in directions:
            continue
        positive, negative = directions[1], directions[-1]
        pairs.append({
            "target_pitch_deg": target,
            "actual_pitch_deg": 0.5 * (positive["actual_pitch_deg"] +
                                        negative["actual_pitch_deg"]),
            "gravity_current": 0.5 * (positive["output_current_mean"] +
                                       negative["output_current_mean"]),
            "friction_current": 0.5 * (positive["output_current_mean"] -
                                        negative["output_current_mean"]),
        })
    if len(pairs) < 3:
        raise ValueError("at least three opposite-direction angle pairs are required")
    theta = np.radians([row["actual_pitch_deg"] for row in pairs])
    current = np.asarray([row["gravity_current"] for row in pairs])
    design = np.column_stack((np.sin(theta), np.ones_like(theta)))
    coefficient, _, rank, _ = np.linalg.lstsq(design, current, rcond=None)
    residual = current - design @ coefficient
    return pairs, coefficient, residual, int(rank), float(np.linalg.cond(design))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-prefix", type=Path)
    parser.add_argument("--tail-fraction", type=float, default=0.5,
                        help="Fraction at the end of every settle period to average")
    args = parser.parse_args()
    if not 0.1 <= args.tail_fraction <= 1.0:
        parser.error("--tail-fraction must be in 0.1..1.0")
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit("Install numpy first: python -m pip install numpy") from exc

    rows = read_rows(args.input)
    point_rows = []
    for group_index, group in enumerate(settled_groups(rows)):
        tail = group[max(0, int(len(group) * (1.0 - args.tail_fraction))):]
        if len(tail) < 20:
            continue
        point_rows.append({
            "group": group_index,
            "target_pitch_deg": math.degrees(mean([r["target_pitch_rad"] for r in tail])),
            "actual_pitch_deg": math.degrees(mean([r["imu_pitch_rad"] for r in tail])),
            "gyro_y_rms_rad_s": math.sqrt(mean([r["gyro_y_rad_s"] ** 2 for r in tail])),
            "output_current_mean": mean([r["output_current"] for r in tail]),
            "output_current_std": float(np.std([r["output_current"] for r in tail])),
            "feedback_current_mean": mean([r["feedback_current"] for r in tail]),
            "direction": int(round(mean([r["direction"] for r in tail]))),
            "sample_count": len(tail),
        })
    if len(point_rows) < 4:
        raise SystemExit("Not enough settled angle groups to fit a gravity model")

    coeff, residual, rank, r2 = fit_model(point_rows, np)
    sin_coeff, cos_coeff, bias, friction = map(float, coeff)
    amplitude = math.hypot(sin_coeff, cos_coeff)
    phase_deg = math.degrees(math.atan2(cos_coeff, sin_coeff))
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    pairs, robust_coeff, robust_residual, robust_rank, robust_condition = \
        fit_direction_paired_model(point_rows, np)
    robust_sin, robust_bias = map(float, robust_coeff)
    robust_rmse = float(np.sqrt(np.mean(robust_residual ** 2)))
    friction_mean = float(np.mean([row["friction_current"] for row in pairs]))
    full_design = np.column_stack((np.sin(np.radians(
        [r["actual_pitch_deg"] for r in point_rows])), np.cos(np.radians(
        [r["actual_pitch_deg"] for r in point_rows])),
        np.ones(len(point_rows)), [r["direction"] for r in point_rows]))
    full_condition = float(np.linalg.cond(full_design))

    prefix = args.output_prefix or args.input.with_name(args.input.stem + "_gravity_fit")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    points_path = prefix.with_name(prefix.name + "_points.csv")
    summary_path = prefix.with_name(prefix.name + "_summary.json")
    with points_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(point_rows[0]))
        writer.writeheader(); writer.writerows(point_rows)
    summary = {
        "schema": "pitch-gravity-fit-v1",
        "source_csv": str(args.input.resolve()),
        "model": "u = a*sin(theta) + b*cos(theta) + c + f*direction",
        "sin_coefficient": sin_coeff,
        "cos_coefficient": cos_coeff,
        "gravity_amplitude_current": amplitude,
        "gravity_phase_deg": phase_deg,
        "bias_current": bias,
        "coulomb_friction_current": friction,
        "rmse_current": rmse,
        "r_squared": r2,
        "matrix_rank": rank,
        "settled_points": len(point_rows),
        "tail_fraction": args.tail_fraction,
        "full_model_condition_number": full_condition,
        "recommended_model": "u_ff = kg*sin(theta) + bias",
        "recommended_gravity_sin_coefficient": robust_sin,
        "recommended_bias_current": robust_bias,
        "recommended_friction_current": friction_mean,
        "recommended_fit_rmse_current": robust_rmse,
        "recommended_fit_condition_number": robust_condition,
        "recommended_fit_rank": robust_rank,
        "opposite_direction_pair_count": len(pairs),
        "opposite_direction_pairs": pairs,
    }
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Gravity model: u = {sin_coeff:.3f} sin(theta) + {cos_coeff:.3f} cos(theta) "
          f"+ {bias:.3f} + {friction:.3f} direction")
    print(f"Amplitude={amplitude:.3f}, phase={phase_deg:.3f} deg, "
          f"RMSE={rmse:.3f}, R^2={r2:.5f}")
    print(f"Recommended gravity FF: u_ff = {robust_sin:.3f} sin(theta) "
          f"+ {robust_bias:.3f}; paired RMSE={robust_rmse:.3f}, "
          f"friction estimate={friction_mean:.3f}")
    print(f"Saved {points_path} and {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
