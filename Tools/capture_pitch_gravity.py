#!/usr/bin/env python3
"""Capture a protected quasi-static Pitch gravity-torque scan over USB CDC."""

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

MSG_GRAVITY_CONFIG = 0x05
MSG_GRAVITY_SAMPLE = 0x84
GRAVITY_CONFIG = struct.Struct("<hhhHHB")
GRAVITY_SAMPLE = struct.Struct("<IIfffhhHfhfBbBH")
GRAVITY_COLUMNS = [
    "timestamp_ms", "sample_index", "target_pitch_rad", "imu_pitch_rad",
    "gyro_y_rad_s", "output_current", "feedback_current", "encoder",
    "motor_angle_rad", "yaw_hold_current", "imu_yaw_rad", "phase",
    "direction", "state", "safety_flags",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB CDC port, for example COM7")
    parser.add_argument("--output", required=True, help="Output path stem")
    parser.add_argument("--min-deg", type=float, default=-30.0)
    parser.add_argument("--max-deg", type=float, default=30.0)
    parser.add_argument("--step-deg", type=float, default=15.0)
    parser.add_argument("--ramp-ms", type=int, default=1500)
    parser.add_argument("--settle-ms", type=int, default=1500)
    parser.add_argument("--repetitions", type=int, default=1)
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc

    minimum = round(args.min_deg * 100)
    maximum = round(args.max_deg * 100)
    step = round(args.step_deg * 100)
    if not (-4500 <= minimum < maximum <= 4500):
        parser.error("angle range must satisfy -45 <= min < max <= 45 deg")
    if step <= 0 or (maximum - minimum) % step:
        parser.error("--step-deg must divide the requested angle span exactly")
    if not 200 <= args.ramp_ms <= 5000 or not 200 <= args.settle_ms <= 5000:
        parser.error("ramp and settle time must each be in 200..5000 ms")
    if not 1 <= args.repetitions <= 5:
        parser.error("--repetitions must be in 1..5")
    points = (maximum - minimum) // step + 1
    # min -> max -> min -> experiment-start angle
    duration_ms = (2 * points) * args.repetitions * (args.ramp_ms + args.settle_ms)
    if duration_ms > 60000:
        parser.error(f"scan duration is {duration_ms} ms; firmware limit is 60000 ms")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_path, csv_path, meta_path = (
        output.with_suffix(".bin"), output.with_suffix(".csv"), output.with_suffix(".json"))
    config = GRAVITY_CONFIG.pack(minimum, maximum, step, args.ramp_ms,
                                 args.settle_ms, args.repetitions)
    frame_parser = FrameParser()
    samples: list[tuple] = []
    raw_data = bytearray()
    final_status = None
    started_at = time.time()

    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1.0) as port:
        time.sleep(0.3)
        port.reset_input_buffer()
        send_command(port, MSG_GRAVITY_CONFIG, 0, config)
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
                    if kind == MSG_GRAVITY_SAMPLE and len(payload) == GRAVITY_SAMPLE.size:
                        samples.append(GRAVITY_SAMPLE.unpack(payload))
                    elif kind == MSG_ACK and len(payload) == ACK.size:
                        command, accepted, state, reason = ACK.unpack(payload)
                        print(f"ACK command={command} accepted={accepted} "
                              f"state={STATE_NAMES.get(state, state)} reason=0x{reason:04X}")
                        if not accepted:
                            raise RuntimeError("Device rejected the gravity scan command")
                    elif kind == MSG_STATUS and len(payload) == STATUS.size:
                        final_status = STATUS.unpack(payload)
                        state = final_status[0]
                        print(f"STATUS state={STATE_NAMES.get(state, state)} "
                              f"dropped={final_status[6]} reason=0x{final_status[7]:04X}")
                        if state in (3, 4):
                            deadline = time.monotonic()
        except KeyboardInterrupt:
            print("Stopping gravity scan...")
            send_command(port, MSG_STOP, 2)
            time.sleep(0.2)
            raw_data.extend(port.read(port.in_waiting))

    raw_path.write_bytes(raw_data)
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(GRAVITY_COLUMNS)
        writer.writerows(samples)
    indexes = [row[1] for row in samples]
    host_missing = sum(max(0, b - a - 1) for a, b in zip(indexes, indexes[1:]))
    metadata = {
        "schema": "pitch-gravity-scan-v1",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "axis": "pitch",
        "experiment": "gravity",
        "minimum_deg": minimum / 100.0,
        "maximum_deg": maximum / 100.0,
        "step_deg": step / 100.0,
        "ramp_ms": args.ramp_ms,
        "settle_ms": args.settle_ms,
        "repetitions": args.repetitions,
        "duration_ms": duration_ms,
        "sample_count": len(samples),
        "host_detected_missing_samples": host_missing,
        "crc_errors": frame_parser.crc_errors,
        "elapsed_host_s": time.time() - started_at,
        "final_status": list(final_status) if final_status else None,
    }
    meta_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Saved {len(samples)} samples to {csv_path}; raw stream: {raw_path}")
    return 0 if samples else 2


if __name__ == "__main__":
    sys.exit(main())
