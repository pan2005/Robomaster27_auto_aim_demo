#!/usr/bin/env python3
"""Capture a Pitch position-control run with or without gravity feedforward."""

import argparse
import csv
import json
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    from capture_sysid import (
        ACK, MSG_ACK, MSG_START, MSG_STATUS, MSG_STOP, STATE_NAMES, STATUS,
        FrameParser, send_command,
    )
except ImportError:
    from Tools.capture_sysid import (
        ACK, MSG_ACK, MSG_START, MSG_STATUS, MSG_STOP, STATE_NAMES, STATUS,
        FrameParser, send_command,
    )

MSG_PITCH_CONTROL_CONFIG = 0x06
MSG_PITCH_CONTROL_SAMPLE = 0x85
MODES = {"pid": 1, "gravity-ff": 2, "dynamic-ff": 3}
CONFIG = struct.Struct("<BhhHHBH")
SAMPLE = struct.Struct("<IIfffhhhhHfhfBBBH")
SAMPLE_COLUMNS = [
    "timestamp_ms", "sample_index", "target_pitch_rad", "imu_pitch_rad",
    "gyro_y_rad_s", "pid_current", "feedforward_current", "output_current",
    "feedback_current", "encoder", "motor_angle_rad", "yaw_hold_current",
    "imu_yaw_rad", "phase", "mode", "state", "safety_flags",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", required=True, help="Output path stem")
    parser.add_argument("--mode", choices=MODES, required=True)
    parser.add_argument("--center-deg", type=float, default=0.0)
    parser.add_argument("--amplitude-deg", type=float, default=15.0)
    parser.add_argument("--ramp-ms", type=int, default=1000)
    parser.add_argument("--hold-ms", type=int, default=1500)
    parser.add_argument("--repetitions", type=int, default=2)
    parser.add_argument("--dynamic-ff-scale", type=float, default=0.5,
                        help="Scale for identified Kv/Ka in dynamic-ff mode, 0..1.2")
    args = parser.parse_args()
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc

    center = round(args.center_deg * 100)
    amplitude = round(args.amplitude_deg * 100)
    if not -3000 <= center <= 3000 or not 1 <= amplitude <= 2000:
        parser.error("center must be within ±30 deg and amplitude within 0.01..20 deg")
    if center - amplitude < -3000 or center + amplitude > 3000:
        parser.error("center ± amplitude must remain within ±30 deg")
    if not 200 <= args.ramp_ms <= 5000 or not 200 <= args.hold_ms <= 5000:
        parser.error("ramp and hold time must each be in 200..5000 ms")
    if not 1 <= args.repetitions <= 5:
        parser.error("--repetitions must be in 1..5")
    if not 0.0 <= args.dynamic_ff_scale <= 1.2:
        parser.error("--dynamic-ff-scale must be in 0..1.2")
    duration_ms = 3 * args.repetitions * (args.ramp_ms + args.hold_ms)
    if duration_ms > 60000:
        parser.error(f"test duration is {duration_ms} ms; firmware limit is 60000 ms")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_path, csv_path, meta_path = (
        output.with_suffix(".bin"), output.with_suffix(".csv"), output.with_suffix(".json"))
    config = CONFIG.pack(MODES[args.mode], center, amplitude, args.ramp_ms,
                         args.hold_ms, args.repetitions,
                         round(args.dynamic_ff_scale * 1000))
    frame_parser = FrameParser()
    samples = []
    raw_data = bytearray()
    final_status = None
    started_at = time.time()

    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1.0) as port:
        time.sleep(0.3)
        port.reset_input_buffer()
        send_command(port, MSG_PITCH_CONTROL_CONFIG, 0, config)
        time.sleep(0.1)
        send_command(port, MSG_START, 1)
        try:
            deadline = time.monotonic() + duration_ms / 1000.0 + 10.0
            while time.monotonic() < deadline:
                chunk = port.read(port.in_waiting or 1)
                if not chunk:
                    continue
                raw_data.extend(chunk)
                for kind, _, payload, _ in frame_parser.feed(chunk):
                    if kind == MSG_PITCH_CONTROL_SAMPLE and len(payload) == SAMPLE.size:
                        samples.append(SAMPLE.unpack(payload))
                    elif kind == MSG_ACK and len(payload) == ACK.size:
                        command, accepted, state, reason = ACK.unpack(payload)
                        print(f"ACK command={command} accepted={accepted} "
                              f"state={STATE_NAMES.get(state, state)} reason=0x{reason:04X}")
                        if not accepted:
                            raise RuntimeError("Device rejected the Pitch control test")
                    elif kind == MSG_STATUS and len(payload) == STATUS.size:
                        final_status = STATUS.unpack(payload)
                        state = final_status[0]
                        print(f"STATUS state={STATE_NAMES.get(state, state)} "
                              f"dropped={final_status[6]} reason=0x{final_status[7]:04X}")
                        if state in (3, 4):
                            deadline = time.monotonic()
        except KeyboardInterrupt:
            print("Stopping Pitch control test...")
            send_command(port, MSG_STOP, 2)
            time.sleep(0.2)
            raw_data.extend(port.read(port.in_waiting))

    raw_path.write_bytes(raw_data)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream); writer.writerow(SAMPLE_COLUMNS); writer.writerows(samples)
    indexes = [row[1] for row in samples]
    missing = sum(max(0, b - a - 1) for a, b in zip(indexes, indexes[1:]))
    metadata = {
        "schema": "pitch-control-comparison-v1",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "port": args.port, "axis": "pitch", "experiment": "pitch-control",
        "mode": args.mode, "center_deg": center / 100.0,
        "amplitude_deg": amplitude / 100.0, "ramp_ms": args.ramp_ms,
        "hold_ms": args.hold_ms, "repetitions": args.repetitions,
        "duration_ms": duration_ms,
        "gravity_feedforward": {
            "formula": "kg*sin(imu_pitch_rad)+bias",
            "kg": -2298.8448, "bias": -99.3683,
        },
        "dynamic_feedforward": {
            "enabled": args.mode == "dynamic-ff",
            "formula": "scale*(kv*target_velocity_rad_s+ka*target_acceleration_rad_s2)",
            "kv": 1830.4092, "ka": 50.6137,
            "scale": args.dynamic_ff_scale,
        },
        "sample_count": len(samples), "host_detected_missing_samples": missing,
        "crc_errors": frame_parser.crc_errors,
        "elapsed_host_s": time.time() - started_at,
        "final_status": list(final_status) if final_status else None,
    }
    meta_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Saved {len(samples)} samples to {csv_path}; raw stream: {raw_path}")
    return 0 if samples else 2


if __name__ == "__main__":
    sys.exit(main())
