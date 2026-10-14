from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from apriltag import AprilTagDetector, TagPose
from auto_aim import AutoAimEngine
from calibration import rotate_rgb
from mvs_camera import LatestFrame


@dataclass(frozen=True)
class VisionSnapshot:
    frame_id: int = -1
    poses: tuple[TagPose, ...] = ()
    locked_id: int | None = None
    processing_ms: float = 0.0
    observation_valid: bool = False
    observation_reason: str = "尚未处理目标"


class VisionWorker:
    def __init__(self, frames: LatestFrame, detector: AprilTagDetector,
                 engine: AutoAimEngine) -> None:
        self.frames, self.detector, self.engine = frames, detector, engine
        self._lock = threading.Lock()
        self._snapshot = VisionSnapshot()
        self._locked_id: int | None = None
        self._last_center: tuple[float, float] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="apriltag-vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> VisionSnapshot:
        with self._lock:
            return self._snapshot

    def lock_nearest(self, center: tuple[float, float], radius_px: float = 300.0) -> int | None:
        snap = self.snapshot()
        if not snap.poses:
            return None
        pose = min(snap.poses, key=lambda item: math.dist(item.center, center))
        if math.dist(pose.center, center) > radius_px:
            return None
        with self._lock:
            self._locked_id = pose.tag_id
            self._last_center = pose.center
        return pose.tag_id

    def unlock(self) -> None:
        with self._lock:
            self._locked_id = None
            self._last_center = None
        self.engine.reset()

    def _run(self) -> None:
        previous = -1
        while not self._stop.is_set():
            frame = self.frames.get(previous)
            if frame is None:
                self._stop.wait(0.002)
                continue
            previous = frame.frame_id
            started = time.perf_counter_ns()
            image = np.frombuffer(frame.rgb, np.uint8).reshape(frame.height, frame.width, 3)
            rotated = rotate_rgb(image, 90)
            with self._lock:
                locked = self._locked_id
                last_center = self._last_center
            roi = None
            if locked is not None and last_center is not None:
                cx, cy = last_center
                roi = (int(cx - 320), int(cy - 320), int(cx + 320), int(cy + 320))
            poses = self.detector.detect(rotated, frame.capture_time_ns, locked, roi)
            if locked is not None and roi is not None and not poses:
                poses = self.detector.detect(rotated, frame.capture_time_ns, locked)
            observation_valid = False
            observation_reason = "未锁定目标"
            if locked is not None and not poses:
                observation_reason = "锁定目标当前未通过检测/PnP"
            elif locked is not None:
                for pose in poses:
                    observation_valid = self.engine.observe(pose) or observation_valid
                observation_reason = ("目标与姿态已配对" if observation_valid else
                                      self.engine.last_observation_error)
            elapsed = (time.perf_counter_ns() - started) * 1e-6
            with self._lock:
                if poses:
                    self._last_center = poses[0].center
                self._snapshot = VisionSnapshot(
                    frame.frame_id, tuple(poses), locked, elapsed,
                    observation_valid, observation_reason)
