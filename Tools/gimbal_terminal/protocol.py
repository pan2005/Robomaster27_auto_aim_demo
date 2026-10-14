"""Wire protocol shared conceptually with Tasks/gimbal_protocol.[ch]."""

from __future__ import annotations

import dataclasses
import struct
from typing import Iterable

SYNC = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 48
CONTROL_PERIOD_MS = 10
COMMAND_TIMEOUT_MS = 200

MSG_HELLO = 0x01
MSG_TIME_SYNC_REQ = 0x02
MSG_CONTROL = 0x10
MSG_DISARM = 0x11
MSG_ESTOP = 0x12
MSG_AIM_TARGET = 0x13
MSG_STATUS = 0x80
MSG_ATTITUDE = 0x81
MSG_TIME_SYNC_RESP = 0x82

CONTROL_ARMED = 1 << 0
CONTROL_FIRE = 1 << 1

AIM_VALID = 1 << 0
AIM_ARMED = 1 << 1
AIM_FIRE = 1 << 2

STATUS_IMU_VALID = 1 << 0
STATUS_PITCH_ONLINE = 1 << 1
STATUS_YAW_ONLINE = 1 << 2
STATUS_REFERENCES_READY = 1 << 3
STATUS_CONTROL_ENABLED = 1 << 4
STATUS_LINK_ALIVE = 1 << 5
STATUS_FIRE_REQUESTED = 1 << 6
STATUS_SAFE = 1 << 7
STATUS_IMU_WARMUP_TIMEOUT = 1 << 8


def crc16_ccitt(data: bytes | bytearray | memoryview) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def sequence_is_newer(sequence: int, previous: int, initialized: bool = True) -> bool:
    """Mirror the firmware's wrap-safe duplicate/out-of-order rejection rule."""
    if not initialized:
        return True
    difference = ((sequence - previous + 0x8000) & 0xFFFF) - 0x8000
    return difference > 0


def encode_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")
    body = struct.pack("<BBBH", VERSION, message_type & 0xFF, len(payload), sequence & 0xFFFF) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


@dataclasses.dataclass(frozen=True)
class Frame:
    message_type: int
    sequence: int
    payload: bytes


class FrameParser:
    """Incremental parser that recovers after noise, bad CRC, and partial frames."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0
        self.version_errors = 0

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                if self.buffer[-1:] == SYNC[:1]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 9:
                break
            version, msg_type, length, sequence = struct.unpack_from("<BBBH", self.buffer, 2)
            if version != VERSION:
                self.version_errors += 1
                del self.buffer[0]
                continue
            if length > MAX_PAYLOAD:
                self.length_errors += 1
                del self.buffer[0]
                continue
            frame_length = 9 + length
            if len(self.buffer) < frame_length:
                break
            body = self.buffer[2 : 7 + length]
            expected = crc16_ccitt(body)
            received = struct.unpack_from("<H", self.buffer, 7 + length)[0]
            if expected != received:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append(Frame(msg_type, sequence, bytes(self.buffer[7 : 7 + length])))
            del self.buffer[:frame_length]
        return frames


def encode_control(sequence: int, armed: bool, fire: bool,
                   yaw_delta_mdeg: int, pitch_delta_mdeg: int) -> bytes:
    flags = (CONTROL_ARMED if armed else 0) | (CONTROL_FIRE if fire else 0)
    yaw = max(-32768, min(32767, int(yaw_delta_mdeg)))
    pitch = max(-32768, min(32767, int(pitch_delta_mdeg)))
    return encode_frame(MSG_CONTROL, sequence, struct.pack("<Bhh", flags, yaw, pitch))


def _i32(value: int) -> int:
    return max(-(1 << 31), min((1 << 31) - 1, int(value)))


def encode_aim_target(sequence: int, valid: bool, armed: bool, fire: bool,
                      execute_at_ms: int, yaw_mdeg: int, pitch_mdeg: int,
                      yaw_rate_mdeg_s: int, pitch_rate_mdeg_s: int,
                      yaw_accel_mdeg_s2: int,
                      pitch_accel_mdeg_s2: int) -> bytes:
    flags = ((AIM_VALID if valid else 0) | (AIM_ARMED if armed else 0) |
             (AIM_FIRE if fire else 0))
    payload = struct.pack(
        "<BBIiiiiii", flags, 1, execute_at_ms & 0xFFFFFFFF,
        _i32(yaw_mdeg), _i32(pitch_mdeg), _i32(yaw_rate_mdeg_s),
        _i32(pitch_rate_mdeg_s), _i32(yaw_accel_mdeg_s2),
        _i32(pitch_accel_mdeg_s2))
    return encode_frame(MSG_AIM_TARGET, sequence, payload)


def encode_time_sync_request(sequence: int, pc_time_us: int) -> bytes:
    return encode_frame(MSG_TIME_SYNC_REQ, sequence,
                        struct.pack("<Q", pc_time_us & 0xFFFFFFFFFFFFFFFF))


@dataclasses.dataclass(frozen=True)
class GimbalStatus:
    flags: int
    pitch_mdeg: int
    yaw_mdeg: int
    pitch_target_mdeg: int
    yaw_target_mdeg: int
    pitch_speed_rpm: int
    yaw_speed_rpm: int
    pitch_temperature: int
    yaw_temperature: int
    protocol_errors: int
    last_control_sequence: int
    protocol_version: int

    def has(self, flag: int) -> bool:
        return bool(self.flags & flag)


@dataclasses.dataclass(frozen=True)
class AttitudeSample:
    mcu_time_ms: int
    flags: int
    quaternion_wxyz: tuple[float, float, float, float]
    pitch_mdeg: int
    yaw_mdeg: int
    pitch_rate_mdeg_s: int
    yaw_rate_mdeg_s: int
    host_time_ns: int = 0
    temperature_cdeg: int = 0
    calibration_temperature_cdeg: int = 0
    gyro_offset_urad_s: tuple[int, int, int] = (0, 0, 0)


def decode_attitude(frame: Frame, host_time_ns: int = 0) -> AttitudeSample:
    if frame.message_type != MSG_ATTITUDE or len(frame.payload) not in (30, 46):
        raise ValueError("not a valid ATTITUDE frame")
    values = struct.unpack("<IHhhhhiiii", frame.payload[:30])
    quaternion = tuple(component / 32767.0 for component in values[2:6])
    temperature = calibration_temperature = 0
    offsets = (0, 0, 0)
    if len(frame.payload) == 46:
        temperature, calibration_temperature, *raw_offsets = struct.unpack_from(
            "<hhiii", frame.payload, 30)
        offsets = tuple(raw_offsets)
    return AttitudeSample(values[0], values[1], quaternion,
                          values[6], values[7], values[8], values[9],
                          host_time_ns, temperature, calibration_temperature,
                          offsets)


def decode_time_sync_response(frame: Frame) -> tuple[int, int]:
    if frame.message_type != MSG_TIME_SYNC_RESP or len(frame.payload) != 12:
        raise ValueError("not a valid TIME_SYNC_RESP frame")
    return struct.unpack("<QI", frame.payload)


def decode_status(frame: Frame) -> GimbalStatus:
    if frame.message_type != MSG_STATUS or len(frame.payload) != 29:
        raise ValueError("not a valid STATUS frame")
    return GimbalStatus(*struct.unpack("<HiiiihhBBHHB", frame.payload))


def split_chunks(data: bytes, sizes: Iterable[int]) -> list[bytes]:
    """Test helper for simulating arbitrarily fragmented USB reads."""
    chunks: list[bytes] = []
    offset = 0
    for size in sizes:
        chunks.append(data[offset : offset + size])
        offset += size
    if offset < len(data):
        chunks.append(data[offset:])
    return chunks
