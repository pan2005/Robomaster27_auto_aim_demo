from __future__ import annotations

import ctypes
import importlib
import os
import sys
import threading
import time
import winreg
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


class MvsError(RuntimeError):
    pass


_DLL_HANDLES: list[object] = []
_MVS_MODULE = None
_MVS_LOAD_LOCK = threading.Lock()


def _candidate_roots() -> list[Path]:
    roots: list[Path] = []
    if os.environ.get("HIKROBOT_MVS_ROOT"):
        roots.append(Path(os.environ["HIKROBOT_MVS_ROOT"]))
    for name in ("MVCAM_COMMON_RUNENV", "MVCAM_COMMON_RUNENV_64"):
        if os.environ.get(name):
            roots.append(Path(os.environ[name]))
    for variable in ("ProgramFiles(x86)", "ProgramFiles"):
        base = os.environ.get(variable)
        if base:
            roots.extend([
                Path(base) / "MVS",
                Path(base) / "Common Files" / "MVS",
            ])
    uninstall_locations = (
        (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall"),
        (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall"),
        (winreg.HKEY_CURRENT_USER, r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall"),
    )
    for hive, key_path in uninstall_locations:
        try:
            with winreg.OpenKey(hive, key_path) as parent:
                for index in range(winreg.QueryInfoKey(parent)[0]):
                    try:
                        with winreg.OpenKey(parent, winreg.EnumKey(parent, index)) as item:
                            name = str(winreg.QueryValueEx(item, "DisplayName")[0])
                            if name != "MVS" and not name.startswith("MVS SDK Development"):
                                continue
                            try:
                                location = str(winreg.QueryValueEx(item, "InstallLocation")[0]).strip()
                            except OSError:
                                location = ""
                            if not location:
                                uninstall = str(winreg.QueryValueEx(item, "UninstallString")[0]).strip('" ')
                                location = str(Path(uninstall).parent)
                            roots.append(Path(location))
                    except OSError:
                        continue
        except OSError:
            continue
    unique: list[Path] = []
    for root in roots:
        if root not in unique:
            unique.append(root)
    return unique


def load_mvs_module():
    global _MVS_MODULE
    if _MVS_MODULE is not None:
        return _MVS_MODULE

    with _MVS_LOAD_LOCK:
        if _MVS_MODULE is not None:
            return _MVS_MODULE

        try:
            _MVS_MODULE = importlib.import_module("MvCameraControl_class")
            return _MVS_MODULE
        except (ImportError, OSError):
            # A failed import can leave a half-initialized module behind.
            sys.modules.pop("MvCameraControl_class", None)

        import_paths: list[Path] = []
        dll_paths: list[Path] = []
        for root in _candidate_roots():
            import_paths.extend([
                root / "Development" / "Samples" / "Python" / "MvImport",
                root / "Samples" / "Python" / "MvImport",
                root / "MvImport",
            ])
            dll_paths.extend([
                root,
                root / "Runtime" / "Win64_x64",
                root / "Runtime" / "Win64",
                root / "Win64_x64",
            ])

        valid_dll_paths = [path for path in dll_paths if path.is_dir()]
        if valid_dll_paths:
            # Hikrobot's supplied wrapper loads MvCameraControl.dll by basename.
            # Keep both Python's DLL search handles and PATH active so this also
            # works when MVS Development is on D: but the runtime is on C:.
            existing_path = os.environ.get("PATH", "")
            additions = [str(path) for path in valid_dll_paths
                         if str(path).lower() not in existing_path.lower()]
            if additions:
                os.environ["PATH"] = os.pathsep.join(additions + [existing_path])
            if hasattr(os, "add_dll_directory"):
                for path in valid_dll_paths:
                    try:
                        _DLL_HANDLES.append(os.add_dll_directory(str(path)))
                    except OSError:
                        pass
            for path in valid_dll_paths:
                dll = path / "MvCameraControl.dll"
                if dll.is_file():
                    try:
                        _DLL_HANDLES.append(ctypes.WinDLL(str(dll), winmode=0))
                        break
                    except OSError:
                        continue

        for path in import_paths:
            if (path / "MvCameraControl_class.py").is_file():
                if str(path) not in sys.path:
                    sys.path.insert(0, str(path))
                try:
                    _MVS_MODULE = importlib.import_module("MvCameraControl_class")
                    return _MVS_MODULE
                except (ImportError, OSError) as exc:
                    sys.modules.pop("MvCameraControl_class", None)
                    raise MvsError(
                        "已找到海康 MVS Python SDK，但无法加载 MvCameraControl.dll。"
                        "请确认 MVS Runtime 位数与 Python 一致。"
                    ) from exc
        raise MvsError(
            "未找到海康机器人 MVS Python SDK（MvCameraControl_class.py）。"
            "请安装海康 MVS，并在安装器中勾选 USB3 Vision 驱动和开发组件。"
        )


def _decode_char_array(value) -> str:
    return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")


@dataclass(frozen=True)
class CameraInfo:
    index: int
    model: str
    serial: str

    @property
    def label(self) -> str:
        suffix = f" [{self.serial}]" if self.serial else ""
        return f"{self.model or 'HIKROBOT Camera'}{suffix}"


def enumerate_hikrobot_devices() -> list[CameraInfo]:
    mvs = load_mvs_module()
    device_list = mvs.MV_CC_DEVICE_INFO_LIST()
    mask = mvs.MV_USB_DEVICE | mvs.MV_GIGE_DEVICE
    result = mvs.MvCamera.MV_CC_EnumDevices(mask, device_list)
    if result != 0:
        raise MvsError(f"MVS 枚举设备失败：0x{result & 0xFFFFFFFF:08X}")
    cameras: list[CameraInfo] = []
    for index in range(device_list.nDeviceNum):
        info = device_list.pDeviceInfo[index].contents
        if info.nTLayerType == mvs.MV_USB_DEVICE:
            specific = info.SpecialInfo.stUsb3VInfo
            model = _decode_char_array(specific.chModelName)
            serial = _decode_char_array(specific.chSerialNumber)
        else:
            specific = info.SpecialInfo.stGigEInfo
            model = _decode_char_array(specific.chModelName)
            serial = _decode_char_array(specific.chSerialNumber)
        cameras.append(CameraInfo(index, model, serial))
    return cameras


@dataclass(frozen=True)
class VideoFrame:
    width: int
    height: int
    rgb: bytes
    fps: float
    frame_id: int
    capture_time_ns: int


class LatestFrame:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: VideoFrame | None = None

    def put(self, frame: VideoFrame) -> None:
        with self._lock:
            self._frame = frame

    def get(self, previous_id: int = -1) -> VideoFrame | None:
        with self._lock:
            if self._frame is None or self._frame.frame_id == previous_id:
                return None
            return self._frame


class CameraWorker:
    def __init__(self, frames: LatestFrame,
                 status_callback: Callable[[bool, str], None]) -> None:
        self.frames = frames
        self.status_callback = status_callback
        self.serial = ""
        self.target_fps = 120.0
        self.exposure_us = 8000.0
        self.gain_db = 16.08
        self._stop = threading.Event()
        self._reconnect = threading.Event()
        self._thread: threading.Thread | None = None
        self.connected = False

    def start(self, serial: str = "", target_fps: float = 120.0,
              exposure_us: float = 8000.0, gain_db: float = 16.08) -> None:
        self.serial = serial
        self.target_fps = target_fps
        self.exposure_us = exposure_us
        self.gain_db = gain_db
        self._stop.clear()
        self._reconnect.clear()
        self._thread = threading.Thread(target=self._run, name="hikrobot-camera", daemon=True)
        self._thread.start()

    def configure(self, serial: str, target_fps: float,
                  exposure_us: float, gain_db: float) -> None:
        self.serial = serial
        self.target_fps = target_fps
        self.exposure_us = exposure_us
        self.gain_db = gain_db
        self._reconnect.set()

    def stop(self) -> None:
        self._stop.set()
        self._reconnect.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _set_status(self, connected: bool, detail: str) -> None:
        self.connected = connected
        self.status_callback(connected, detail)

    def _run(self) -> None:
        while not self._stop.is_set():
            camera = None
            try:
                mvs = load_mvs_module()
                device_list = mvs.MV_CC_DEVICE_INFO_LIST()
                result = mvs.MvCamera.MV_CC_EnumDevices(
                    mvs.MV_USB_DEVICE | mvs.MV_GIGE_DEVICE, device_list)
                if result != 0:
                    raise MvsError(f"MVS 枚举失败：0x{result & 0xFFFFFFFF:08X}")
                if not device_list.nDeviceNum:
                    raise MvsError(
                        "MVS 未发现相机；若设备管理器显示错误码 28，请安装/绑定海康 USB3 Vision 驱动"
                    )
                selected = 0
                if self.serial:
                    infos = enumerate_hikrobot_devices()
                    match = next((item for item in infos if item.serial == self.serial), None)
                    if match is None:
                        raise MvsError(f"未找到指定相机 {self.serial}")
                    selected = match.index
                info = device_list.pDeviceInfo[selected].contents
                camera = mvs.MvCamera()
                self._check(camera.MV_CC_CreateHandle(info), "创建设备句柄")
                access = getattr(mvs, "MV_ACCESS_Exclusive", 1)
                self._check(camera.MV_CC_OpenDevice(access, 0), "打开相机")
                trigger_off = getattr(mvs, "MV_TRIGGER_MODE_OFF", 0)
                self._check(camera.MV_CC_SetEnumValue("TriggerMode", trigger_off), "关闭触发模式")
                exposure_auto_off = getattr(mvs, "MV_EXPOSURE_AUTO_MODE_OFF", 0)
                self._check(camera.MV_CC_SetEnumValue("ExposureAuto", exposure_auto_off),
                            "关闭自动曝光")
                gain_auto_off = getattr(mvs, "MV_GAIN_MODE_OFF", 0)
                self._check(camera.MV_CC_SetEnumValue("GainAuto", gain_auto_off),
                            "关闭自动增益")
                if self.target_fps > 0:
                    camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
                    camera.MV_CC_SetFloatValue("AcquisitionFrameRate", float(self.target_fps))
                else:
                    camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", False)
                self._check(camera.MV_CC_SetFloatValue("ExposureTime", float(self.exposure_us)),
                            "设置曝光时间")
                self._check(camera.MV_CC_SetFloatValue("Gain", float(self.gain_db)),
                            "设置增益")
                actual_exposure = mvs.MVCC_FLOATVALUE()
                actual_gain = mvs.MVCC_FLOATVALUE()
                self._check(camera.MV_CC_GetFloatValue("ExposureTime", actual_exposure),
                            "读取曝光时间")
                self._check(camera.MV_CC_GetFloatValue("Gain", actual_gain),
                            "读取增益")
                self._check(camera.MV_CC_StartGrabbing(), "开始取流")
                self._set_status(
                    True, "海康相机已连接 / "
                    f"曝光 {actual_exposure.fCurValue:.0f} μs / "
                    f"Gain {actual_gain.fCurValue:.2f} dB")
                self._capture_loop(mvs, camera)
            except (MvsError, OSError, AttributeError) as exc:
                self._set_status(False, str(exc))
            finally:
                if camera is not None:
                    try:
                        camera.MV_CC_StopGrabbing()
                        camera.MV_CC_CloseDevice()
                        camera.MV_CC_DestroyHandle()
                    except Exception:
                        pass
                self._reconnect.clear()
            if not self._stop.is_set():
                self._reconnect.wait(1.0)

    @staticmethod
    def _check(result: int, action: str) -> None:
        if result != 0:
            raise MvsError(f"{action}失败：0x{result & 0xFFFFFFFF:08X}")

    def _capture_loop(self, mvs, camera) -> None:
        frame_id = 0
        frame_times: list[float] = []
        destination = None
        destination_size = 0
        while not self._stop.is_set() and not self._reconnect.is_set():
            frame = mvs.MV_FRAME_OUT()
            result = camera.MV_CC_GetImageBuffer(frame, 500)
            if result != 0:
                raise MvsError(f"取图超时或相机断开：0x{result & 0xFFFFFFFF:08X}")
            try:
                width = int(frame.stFrameInfo.nWidth)
                height = int(frame.stFrameInfo.nHeight)
                required = width * height * 3
                if destination is None or required != destination_size:
                    destination = (ctypes.c_ubyte * required)()
                    destination_size = required
                params = mvs.MV_CC_PIXEL_CONVERT_PARAM_EX()
                params.nWidth = width
                params.nHeight = height
                params.pSrcData = frame.pBufAddr
                params.nSrcDataLen = frame.stFrameInfo.nFrameLen
                params.enSrcPixelType = frame.stFrameInfo.enPixelType
                params.enDstPixelType = mvs.PixelType_Gvsp_RGB8_Packed
                params.pDstBuffer = ctypes.cast(destination, ctypes.POINTER(ctypes.c_ubyte))
                params.nDstBufferSize = destination_size
                self._check(camera.MV_CC_ConvertPixelTypeEx(params), "像素转换")
                receive_time_ns = time.perf_counter_ns()
                capture_time_ns = receive_time_ns - int(self.exposure_us * 500.0)
                now = receive_time_ns * 1.0e-9
                frame_times.append(now)
                while frame_times and frame_times[0] < now - 1.0:
                    frame_times.pop(0)
                fps = float(len(frame_times))
                self.frames.put(VideoFrame(width, height, bytes(destination), fps, frame_id,
                                           capture_time_ns))
                frame_id += 1
            finally:
                camera.MV_CC_FreeImageBuffer(frame)
