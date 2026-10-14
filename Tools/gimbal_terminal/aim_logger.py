from __future__ import annotations

import csv
import datetime as dt
import queue
import threading
from pathlib import Path


class AimLogger:
    FIELDS = ("host_time_ns", "mode", "tag_id", "distance_m", "vision_ms",
              "actual_yaw_deg", "actual_pitch_deg", "command_yaw_deg",
              "command_pitch_deg", "yaw_rate_deg_s", "pitch_rate_deg_s",
              "yaw_accel_deg_s2", "pitch_accel_deg_s2", "fire_authorized",
              "fire", "saturated", "aim_error_deg", "tracking_error_deg",
              "target_world_x_m", "target_world_y_m", "target_world_z_m",
              "gate_reason")

    def __init__(self, base: Path) -> None:
        base.mkdir(parents=True, exist_ok=True)
        self.path = base / f"aim_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
        self._queue: queue.Queue[dict | None] = queue.Queue(maxsize=2048)
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="aim-logger")
        self._thread.start()

    def put(self, row: dict) -> None:
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            pass

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        with self.path.open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.FIELDS)
            writer.writeheader()
            while True:
                row = self._queue.get()
                if row is None:
                    break
                writer.writerow({field: row.get(field, "") for field in self.FIELDS})
