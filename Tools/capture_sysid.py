#!/usr/bin/env python3
"""Run a protected Yaw or Pitch identification experiment over USB CDC."""

import argparse
import csv
import json
import math
import struct
import sys
import time
from pathlib import Path

MAGIC = 0xA55A
VERSION = 4
HEADER = struct.Struct("<HBBHI")
CONFIG = struct.Struct("<BBhHII")
PITCH_PRBS_CONFIG = struct.Struct("<hhHIIHH")
ACK = struct.Struct("<BBBH")
STATUS = struct.Struct("<BBhHIIIH")
SAMPLE = struct.Struct("<IIhhhHfhhffffffBH")

MSG_CONFIG = 0x01
MSG_START = 0x02
MSG_STOP = 0x03
MSG_QUERY = 0x04
MSG_PITCH_PRBS_CONFIG = 0x07
MSG_ACK = 0x81
MSG_STATUS = 0x82
MSG_SAMPLE = 0x83

STATE_NAMES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "COMPLETE", 4: "ABORTED"}
SAMPLE_COLUMNS = [
    "timestamp_ms", "sample_index", "excitation_current", "output_current",
    "support_hold_current", "encoder", "motor_angle_rad", "speed_rpm", "feedback_current",
    "imu_yaw_rad", "imu_pitch_rad", "imu_roll_rad", "gyro_x_rad_s",
    "gyro_y_rad_s", "gyro_z_rad_s", "state", "safety_flags",
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    body = HEADER.pack(MAGIC, VERSION, message_type, len(payload), sequence) + payload
    return body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        magic_bytes = struct.pack("<H", MAGIC)
        while len(self.buffer) >= HEADER.size + 2:
            if self.buffer[:2] != magic_bytes:
                del self.buffer[0]
                continue
            magic, version, kind, length, sequence = HEADER.unpack_from(self.buffer)
            total = HEADER.size + length + 2
            if version != VERSION or total > 4096:
                del self.buffer[0]
                continue
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            expected = struct.unpack_from("<H", frame, total - 2)[0]
            if crc16_ccitt(frame[:-2]) != expected:
                self.crc_errors += 1
                continue
            frames.append((kind, sequence, frame[HEADER.size:-2], frame))
        return frames


def send_command(port, kind, sequence, payload=b""):
    port.write(make_frame(kind, sequence, payload))
    port.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB CDC port, for example COM7")
    parser.add_argument("--axis", choices=("yaw", "pitch"), default="yaw")
    parser.add_argument("--output", default="sysid_capture", help="Output path stem")
    parser.add_argument("--type", choices=("prbs", "step"), default="prbs")
    parser.add_argument("--amplitude", type=int, default=500, help="Absolute current command, max 2000")
    parser.add_argument("--switch-ms", type=int, default=20, help="PRBS switching interval, 5..1000 ms")
    parser.add_argument("--duration-ms", type=int, default=10000, help="Experiment duration, 100..60000 ms")
    parser.add_argument("--seed", type=lambda value: int(value, 0), default=0x12345678)
    parser.add_argument("--operating-angle-deg", type=float,
                        help="Automatically move Pitch to this absolute IMU angle before PRBS")
    parser.add_argument("--move-ms", type=int, default=2500,
                        help="Pitch automatic pre-position/return ramp time")
    parser.add_argument("--settle-ms", type=int, default=1000,
                        help="Pitch settling time before PRBS and before completion")
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc

    if not 1 <= abs(args.amplitude) <= 2000:
        parser.error("--amplitude must be in 1..2000")
    if not 100 <= args.duration_ms <= 60000:
        parser.error("--duration-ms must be in 100..60000")
    if args.type == "prbs" and not 5 <= args.switch_ms <= 1000:
        parser.error("--switch-ms must be in 5..1000")
    positioned_pitch = args.operating_angle_deg is not None
    if positioned_pitch and (args.axis != "pitch" or args.type != "prbs"):
        parser.error("--operating-angle-deg is only valid for Pitch PRBS")
    if positioned_pitch and not -20.0 <= args.operating_angle_deg <= 20.0:
        parser.error("--operating-angle-deg must be within -20..20 deg")
    if positioned_pitch and not 500 <= args.move_ms <= 10000:
        parser.error("--move-ms must be in 500..10000")
    if positioned_pitch and not 200 <= args.settle_ms <= 5000:
        parser.error("--settle-ms must be in 200..5000")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_path = output.with_suffix(".bin")
    csv_path = output.with_suffix(".csv")
    meta_path = output.with_suffix(".json")
    excitation = 2 if args.type == "prbs" else 1
    axis = 2 if args.axis == "pitch" else 1
    if positioned_pitch:
        total_duration_ms = (args.duration_ms + 2 * args.move_ms +
                             2 * args.settle_ms)
        if total_duration_ms > 60000:
            parser.error("positioning + PRBS + return must not exceed 60000 ms")
        config_kind = MSG_PITCH_PRBS_CONFIG
        config = PITCH_PRBS_CONFIG.pack(
            round(args.operating_angle_deg * 100), abs(args.amplitude),
            args.switch_ms, args.duration_ms, args.seed & 0xFFFFFFFF,
            args.move_ms, args.settle_ms)
    else:
        total_duration_ms = args.duration_ms
        config_kind = MSG_CONFIG
        config = CONFIG.pack(axis, excitation, abs(args.amplitude), args.switch_ms,
                             args.duration_ms, args.seed & 0xFFFFFFFF)

    frame_parser = FrameParser()
    samples = []
    raw_data = bytearray()
    command_sequence = 0
    final_status = None
    started_at = time.time()

    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1.0) as port:
        time.sleep(0.3)
        port.reset_input_buffer()
        send_command(port, config_kind, command_sequence, config)
        command_sequence += 1
        time.sleep(0.1)
        send_command(port, MSG_START, command_sequence)
        command_sequence += 1

        try:
            deadline = time.monotonic() + total_duration_ms / 1000.0 + 10.0
            while time.monotonic() < deadline:
                chunk = port.read(port.in_waiting or 1)
                if not chunk:
                    continue
                raw_data.extend(chunk)
                for kind, _, payload, _ in frame_parser.feed(chunk):
                    if kind == MSG_SAMPLE and len(payload) == SAMPLE.size:
                        samples.append(SAMPLE.unpack(payload))
                    elif kind == MSG_ACK and len(payload) == ACK.size:
                        command, accepted, state, reason = ACK.unpack(payload)
                        print(f"ACK command={command} accepted={accepted} state={STATE_NAMES.get(state, state)} reason=0x{reason:04X}")
                        if not accepted:
                            raise RuntimeError("Device rejected the experiment command")
                    elif kind == MSG_STATUS and len(payload) == STATUS.size:
                        final_status = STATUS.unpack(payload)
                        state = final_status[0]
                        print(f"STATUS state={STATE_NAMES.get(state, state)} dropped={final_status[6]} reason=0x{final_status[7]:04X}")
                        if state in (3, 4):
                            deadline = time.monotonic()
        except KeyboardInterrupt:
            print("Stopping experiment...")
            send_command(port, MSG_STOP, command_sequence)
            time.sleep(0.2)
            raw_data.extend(port.read(port.in_waiting))

    raw_path.write_bytes(raw_data)
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(SAMPLE_COLUMNS)
        writer.writerows(samples)

    indexes = [row[1] for row in samples]
    host_missing = sum(max(0, b - a - 1) for a, b in zip(indexes, indexes[1:]))
    metadata = {
        "port": args.port,
        "axis": args.axis,
        "excitation": args.type,
        "amplitude": abs(args.amplitude),
        "switch_ms": args.switch_ms,
        "duration_ms": args.duration_ms,
        "seed": args.seed & 0xFFFFFFFF,
        "sample_count": len(samples),
        "host_detected_missing_samples": host_missing,
        "crc_errors": frame_parser.crc_errors,
        "elapsed_host_s": time.time() - started_at,
        "final_status": list(final_status) if final_status else None,
    }
    if args.axis == "pitch":
        metadata["pitch_baseline"] = "position_feedback + gravity_feedforward"
        metadata["gravity_feedforward"] = {
            "formula": "kg*sin(imu_pitch_rad)+bias",
            "kg": -2298.8448,
            "bias": -99.3683,
        }
        pitch_values = [row[10] for row in samples]
        if pitch_values:
            metadata["measured_pitch_mean_deg"] = math.degrees(
                sum(pitch_values) / len(pitch_values))
            metadata["measured_pitch_range_deg"] = math.degrees(
                max(pitch_values) - min(pitch_values))
    if positioned_pitch:
        metadata["automatic_positioning"] = True
        metadata["operating_angle_deg"] = args.operating_angle_deg
        metadata["move_ms"] = args.move_ms
        metadata["settle_ms"] = args.settle_ms
        metadata["total_sequence_duration_ms"] = total_duration_ms
    meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Saved {len(samples)} samples to {csv_path}; raw stream: {raw_path}")
    return 0 if samples else 2


if __name__ == "__main__":
    sys.exit(main())
