from __future__ import annotations

import queue
import dataclasses
import threading
import time
from collections.abc import Callable

import serial
from serial.tools import list_ports

from protocol import (
    AttitudeSample,
    FrameParser,
    GimbalStatus,
    MSG_ATTITUDE,
    MSG_DISARM,
    MSG_ESTOP,
    MSG_HELLO,
    MSG_STATUS,
    MSG_TIME_SYNC_RESP,
    decode_attitude,
    decode_status,
    decode_time_sync_response,
    encode_aim_target,
    encode_control,
    encode_frame,
    encode_time_sync_request,
)

STM32_VID = 0x0483
STM32_PID = 0x5740


def available_ports() -> list[tuple[str, str]]:
    result = []
    for port in list_ports.comports():
        label = f"{port.device} — {port.description}"
        result.append((port.device, label))
    return result


def preferred_port(configured: str = "") -> str | None:
    ports = list(list_ports.comports())
    if configured and any(item.device == configured for item in ports):
        return configured
    for item in ports:
        if item.vid == STM32_VID and item.pid == STM32_PID:
            return item.device
    return ports[0].device if len(ports) == 1 else None


class SerialLink:
    def __init__(self, status_callback: Callable[[GimbalStatus], None],
                 connection_callback: Callable[[bool, str], None],
                 attitude_callback: Callable[[AttitudeSample], None] | None = None) -> None:
        self._status_callback = status_callback
        self._connection_callback = connection_callback
        self._attitude_callback = attitude_callback or (lambda sample: None)
        self._port = ""
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._outgoing: queue.Queue[bytes] = queue.Queue(maxsize=64)
        self._thread: threading.Thread | None = None
        self._sequence = 0
        self.connected = False
        self._connection_detail = ""
        self._clock_lock = threading.Lock()
        self._clock_offset_ms: float | None = None
        self._sync_samples: list[tuple[float, float]] = []

    def start(self, port: str = "") -> None:
        self._port = port
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="gimbal-serial", daemon=True)
        self._thread.start()

    def set_port(self, port: str) -> None:
        self._port = port
        self._wake.set()

    def _next_sequence(self) -> int:
        value = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        return value

    def _enqueue(self, frame: bytes) -> None:
        try:
            self._outgoing.put_nowait(frame)
        except queue.Full:
            try:
                self._outgoing.get_nowait()
            except queue.Empty:
                pass
            self._outgoing.put_nowait(frame)

    def send_control(self, armed: bool, fire: bool,
                     yaw_delta_mdeg: int, pitch_delta_mdeg: int) -> None:
        self._enqueue(encode_control(self._next_sequence(), armed, fire,
                                     yaw_delta_mdeg, pitch_delta_mdeg))

    def send_aim_target(self, valid: bool, armed: bool, fire: bool,
                        execute_at_ms: int, yaw_mdeg: int, pitch_mdeg: int,
                        yaw_rate_mdeg_s: int, pitch_rate_mdeg_s: int,
                        yaw_accel_mdeg_s2: int,
                        pitch_accel_mdeg_s2: int) -> None:
        self._enqueue(encode_aim_target(
            self._next_sequence(), valid, armed, fire, execute_at_ms,
            yaw_mdeg, pitch_mdeg, yaw_rate_mdeg_s, pitch_rate_mdeg_s,
            yaw_accel_mdeg_s2, pitch_accel_mdeg_s2))

    def mcu_time_ms(self, host_time_ns: int | None = None) -> int | None:
        with self._clock_lock:
            offset = self._clock_offset_ms
        if offset is None:
            return None
        host_ms = (host_time_ns if host_time_ns is not None
                   else time.perf_counter_ns()) * 1.0e-6
        return int(round(host_ms + offset)) & 0xFFFFFFFF

    def host_time_ns(self, mcu_time_ms: int) -> int:
        now_ns = time.perf_counter_ns()
        current = self.mcu_time_ms(now_ns)
        if current is None:
            return now_ns
        delta = ((mcu_time_ms - current + 0x80000000) & 0xFFFFFFFF) - 0x80000000
        return now_ns + int(delta * 1_000_000)

    @property
    def clock_synchronized(self) -> bool:
        with self._clock_lock:
            return self._clock_offset_ms is not None

    def _accept_time_sync(self, echoed_pc_us: int, mcu_time_ms: int,
                          receive_ns: int) -> None:
        receive_us = receive_ns // 1000
        rtt_ms = max(0.0, (receive_us - echoed_pc_us) / 1000.0)
        midpoint_ms = (echoed_pc_us + receive_us) / 2000.0
        raw_offset = float(mcu_time_ms) - midpoint_ms
        # Select the lowest-jitter samples. The modulo result remains correct
        # when converted back to uint32 in mcu_time_ms().
        self._sync_samples.append((rtt_ms, raw_offset))
        self._sync_samples = sorted(self._sync_samples, key=lambda item: item[0])[:8]
        best = self._sync_samples[:3]
        offset = sum(item[1] for item in best) / len(best)
        with self._clock_lock:
            self._clock_offset_ms = offset

    def disarm(self, emergency: bool = False) -> None:
        msg = MSG_ESTOP if emergency else MSG_DISARM
        for _ in range(3):
            self._enqueue(encode_frame(msg, self._next_sequence()))

    def stop(self) -> None:
        self.disarm()
        time.sleep(0.03)
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=1.5)

    def _run(self) -> None:
        while not self._stop.is_set():
            port = preferred_port(self._port)
            if not port:
                self._set_connection(False, "未找到 STM32 虚拟串口")
                self._wake.wait(1.0)
                self._wake.clear()
                continue
            try:
                with serial.Serial(port, 115200, timeout=0.01, write_timeout=0.05) as device:
                    parser = FrameParser()
                    self._set_connection(True, f"已连接 {port}，等待握手")
                    device.write(encode_frame(MSG_HELLO, self._next_sequence()))
                    last_hello = time.monotonic()
                    last_sync = 0.0
                    last_status = last_hello
                    while not self._stop.is_set() and not self._wake.is_set():
                        try:
                            while True:
                                device.write(self._outgoing.get_nowait())
                        except queue.Empty:
                            pass
                        data = device.read(device.in_waiting or 1)
                        for frame in parser.feed(data):
                            if frame.message_type == MSG_STATUS:
                                try:
                                    status = decode_status(frame)
                                except ValueError:
                                    continue
                                last_status = time.monotonic()
                                self._status_callback(status)
                                self._set_connection(True, f"{port} 协议 v{status.protocol_version}")
                            elif frame.message_type == MSG_ATTITUDE:
                                try:
                                    sample = decode_attitude(frame)
                                except ValueError:
                                    continue
                                sample = dataclasses.replace(
                                    sample, host_time_ns=self.host_time_ns(sample.mcu_time_ms))
                                self._attitude_callback(sample)
                            elif frame.message_type == MSG_TIME_SYNC_RESP:
                                try:
                                    echoed, mcu_ms = decode_time_sync_response(frame)
                                except ValueError:
                                    continue
                                self._accept_time_sync(echoed, mcu_ms,
                                                       time.perf_counter_ns())
                        now = time.monotonic()
                        if now - last_sync >= 1.0:
                            pc_us = time.perf_counter_ns() // 1000
                            device.write(encode_time_sync_request(
                                self._next_sequence(), pc_us))
                            last_sync = now
                        if now - last_status > 0.35 and now - last_hello > 0.50:
                            self._set_connection(False, f"{port} 协议心跳超时，正在重新握手")
                            device.write(encode_frame(MSG_HELLO, self._next_sequence()))
                            last_hello = now
                    self._wake.clear()
            except (serial.SerialException, OSError) as exc:
                self._set_connection(False, f"串口断开：{exc}")
                self._wake.wait(1.0)
                self._wake.clear()
        self._set_connection(False, "串口已关闭")

    def _set_connection(self, connected: bool, detail: str) -> None:
        changed = connected != self.connected or detail != self._connection_detail
        self.connected = connected
        self._connection_detail = detail
        if changed:
            self._connection_callback(connected, detail)
