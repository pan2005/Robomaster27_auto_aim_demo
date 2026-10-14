#!/usr/bin/env python3
"""Run one yaw velocity-control comparison experiment over USB CDC."""

import argparse
import csv
import json
import struct
import sys
import time
from pathlib import Path

MAGIC = 0xA65A
VERSION = 3
HEADER = struct.Struct("<HBBHI")
CONFIG = struct.Struct("<BBhHHH")
ACK = struct.Struct("<BBBH")
STATUS = struct.Struct("<BBhHHHIH")
PARAMS = struct.Struct("<11f")
SAMPLE = struct.Struct("<IIffhhfhhhhhhfffBH")

MSG_CONFIG, MSG_START, MSG_STOP, MSG_QUERY = 0x01, 0x02, 0x03, 0x04
MSG_SET_PARAMS, MSG_GET_PARAMS = 0x05, 0x06
MSG_ACK, MSG_STATUS, MSG_SAMPLE, MSG_PARAMS = 0x81, 0x82, 0x83, 0x84
MODES = {"pid": 1, "velocity-ff": 2, "full-ff": 3}
STATE_NAMES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "COMPLETE", 4: "ABORTED"}
SAMPLE_COLUMNS = [
    "timestamp_ms", "sample_index", "target_rpm", "target_accel_rpm_s",
    "speed_rpm", "feedback_current", "motor_angle_rad",
    "pid_output", "friction_ff", "velocity_ff", "acceleration_ff",
    "output_current", "pitch_hold_current", "imu_yaw_rad", "imu_pitch_rad",
    "gyro_z_rad_s", "state", "safety_flags",
]
PARAM_NAMES = [
    "pid_kp", "pid_ki", "pid_integral_limit", "positive_friction",
    "positive_velocity", "negative_friction", "negative_velocity",
    "friction_blend_rpm", "friction_slew_per_s", "zero_target_rpm",
    "integral_leak_tau_s",
]


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def frame(kind: int, sequence: int, payload: bytes = b"") -> bytes:
    body = HEADER.pack(MAGIC, VERSION, kind, len(payload), sequence) + payload
    return body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self):
        self.data = bytearray()
        self.crc_errors = 0

    def feed(self, chunk: bytes):
        self.data.extend(chunk)
        found = []
        magic = struct.pack("<H", MAGIC)
        while len(self.data) >= HEADER.size + 2:
            if self.data[:2] != magic:
                del self.data[0]
                continue
            _, version, kind, length, sequence = HEADER.unpack_from(self.data)
            total = HEADER.size + length + 2
            if version != VERSION or total > 4096:
                del self.data[0]
                continue
            if len(self.data) < total:
                break
            packet = bytes(self.data[:total])
            del self.data[:total]
            expected = struct.unpack_from("<H", packet, total - 2)[0]
            if crc16(packet[:-2]) != expected:
                self.crc_errors += 1
                continue
            found.append((kind, sequence, packet[HEADER.size:-2]))
        return found


def main() -> int:
    argp = argparse.ArgumentParser(description=__doc__)
    argp.add_argument("--port", required=True, help="USB CDC port, e.g. COM7")
    argp.add_argument("--output", required=True, help="Unique output path stem")
    argp.add_argument("--mode", choices=MODES, required=True)
    argp.add_argument("--target-rpm", type=int, default=15)
    argp.add_argument("--ramp-ms", type=int, default=400)
    argp.add_argument("--hold-ms", type=int, default=1500)
    argp.add_argument("--repetitions", type=int, default=1)
    argp.add_argument("--params-json", help="Runtime controller parameter JSON")
    args = argp.parse_args()

    if not 1 <= args.target_rpm <= 30:
        argp.error("--target-rpm must be 1..30")
    if not 50 <= args.ramp_ms <= 2000:
        argp.error("--ramp-ms must be 50..2000")
    if not 100 <= args.hold_ms <= 5000:
        argp.error("--hold-ms must be 100..5000")
    if not 1 <= args.repetitions <= 20:
        argp.error("--repetitions must be 1..20")
    duration_ms = 4 * (args.ramp_ms + args.hold_ms) * args.repetitions
    if duration_ms > 60000:
        argp.error("configured experiment exceeds the 60 s firmware limit")

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_path, csv_path, json_path = output.with_suffix(".bin"), output.with_suffix(".csv"), output.with_suffix(".json")
    config = CONFIG.pack(MODES[args.mode], 1, args.target_rpm, args.ramp_ms,
                         args.hold_ms, args.repetitions)
    parser = Parser()
    raw = bytearray()
    samples = []
    final_status = None
    applied_params = None
    sequence = 0
    started = time.time()

    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1.0) as port:
        time.sleep(0.3)
        port.reset_input_buffer()
        if args.params_json:
            supplied = json.loads(Path(args.params_json).read_text(encoding="utf-8"))
            try:
                packed_params = PARAMS.pack(*(float(supplied[name]) for name in PARAM_NAMES))
            except KeyError as exc:
                raise SystemExit(f"Missing controller parameter: {exc.args[0]}") from exc
            port.write(frame(MSG_SET_PARAMS, sequence, packed_params)); sequence += 1
        else:
            port.write(frame(MSG_GET_PARAMS, sequence)); sequence += 1
        port.flush()
        time.sleep(0.1)
        port.write(frame(MSG_CONFIG, sequence, config)); sequence += 1
        port.flush()
        time.sleep(0.1)
        port.write(frame(MSG_START, sequence)); sequence += 1
        port.flush()
        deadline = time.monotonic() + duration_ms / 1000.0 + 10.0
        try:
            while time.monotonic() < deadline:
                chunk = port.read(port.in_waiting or 1)
                if not chunk:
                    continue
                raw.extend(chunk)
                for kind, _, payload in parser.feed(chunk):
                    if kind == MSG_SAMPLE and len(payload) == SAMPLE.size:
                        samples.append(SAMPLE.unpack(payload))
                    elif kind == MSG_ACK and len(payload) == ACK.size:
                        command, accepted, state, reason = ACK.unpack(payload)
                        print(f"ACK command={command} accepted={accepted} state={STATE_NAMES.get(state, state)} reason=0x{reason:04X}")
                        if not accepted:
                            raise RuntimeError("Device rejected command")
                    elif kind == MSG_STATUS and len(payload) == STATUS.size:
                        final_status = STATUS.unpack(payload)
                        state, dropped, reason = final_status[0], final_status[6], final_status[7]
                        print(f"STATUS state={STATE_NAMES.get(state, state)} dropped={dropped} reason=0x{reason:04X}")
                        if state in (3, 4):
                            deadline = time.monotonic()
                    elif kind == MSG_PARAMS and len(payload) == PARAMS.size:
                        applied_params = dict(zip(PARAM_NAMES, PARAMS.unpack(payload)))
        except KeyboardInterrupt:
            print("Stopping experiment...")
            port.write(frame(MSG_STOP, sequence))
            port.flush()
            time.sleep(0.2)
            raw.extend(port.read(port.in_waiting))

    raw_path.write_bytes(raw)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(SAMPLE_COLUMNS)
        writer.writerows(samples)

    indexes = [row[1] for row in samples]
    missing = sum(max(0, b - a - 1) for a, b in zip(indexes, indexes[1:]))
    metadata = {
        "port": args.port, "mode": args.mode, "target_rpm": args.target_rpm,
        "ramp_ms": args.ramp_ms, "hold_ms": args.hold_ms,
        "repetitions": args.repetitions, "expected_duration_ms": duration_ms,
        "sample_count": len(samples), "host_detected_missing_samples": missing,
        "crc_errors": parser.crc_errors, "elapsed_host_s": time.time() - started,
        "final_status": list(final_status) if final_status else None,
        "controller_params": applied_params,
    }
    json_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Saved {len(samples)} samples to {csv_path}; raw stream: {raw_path}")
    return 0 if samples else 2


if __name__ == "__main__":
    sys.exit(main())
