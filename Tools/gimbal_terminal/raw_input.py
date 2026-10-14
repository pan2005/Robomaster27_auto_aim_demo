from __future__ import annotations

import ctypes
from ctypes import wintypes

WM_INPUT = 0x00FF
RID_INPUT = 0x10000003
RIDEV_INPUTSINK = 0x00000100
MOUSE_MOVE_ABSOLUTE = 0x0001
RI_MOUSE_LEFT_BUTTON_DOWN = 0x0001
RI_MOUSE_LEFT_BUTTON_UP = 0x0002
RI_MOUSE_RIGHT_BUTTON_DOWN = 0x0004
RI_MOUSE_RIGHT_BUTTON_UP = 0x0008


class RAWINPUTDEVICE(ctypes.Structure):
    _fields_ = [("usUsagePage", wintypes.USHORT), ("usUsage", wintypes.USHORT),
                ("dwFlags", wintypes.DWORD), ("hwndTarget", wintypes.HWND)]


class RAWINPUTHEADER(ctypes.Structure):
    _fields_ = [("dwType", wintypes.DWORD), ("dwSize", wintypes.DWORD),
                ("hDevice", wintypes.HANDLE), ("wParam", wintypes.WPARAM)]


class _BUTTONS_STRUCT(ctypes.Structure):
    _fields_ = [("usButtonFlags", wintypes.USHORT), ("usButtonData", wintypes.USHORT)]


class _BUTTONS_UNION(ctypes.Union):
    _anonymous_ = ("buttons",)
    _fields_ = [("ulButtons", wintypes.ULONG), ("buttons", _BUTTONS_STRUCT)]


class RAWMOUSE(ctypes.Structure):
    _anonymous_ = ("button_union",)
    _fields_ = [("usFlags", wintypes.USHORT), ("button_union", _BUTTONS_UNION),
                ("ulRawButtons", wintypes.ULONG), ("lLastX", wintypes.LONG),
                ("lLastY", wintypes.LONG), ("ulExtraInformation", wintypes.ULONG)]


class _RAW_UNION(ctypes.Union):
    _fields_ = [("mouse", RAWMOUSE), ("padding", ctypes.c_byte * 24)]


class RAWINPUT(ctypes.Structure):
    _fields_ = [("header", RAWINPUTHEADER), ("data", _RAW_UNION)]


class MSG(ctypes.Structure):
    _fields_ = [("hwnd", wintypes.HWND), ("message", wintypes.UINT),
                ("wParam", wintypes.WPARAM), ("lParam", wintypes.LPARAM),
                ("time", wintypes.DWORD), ("pt", wintypes.POINT)]


_register_raw_input_devices = ctypes.windll.user32.RegisterRawInputDevices
_register_raw_input_devices.argtypes = [ctypes.POINTER(RAWINPUTDEVICE), wintypes.UINT,
                                        wintypes.UINT]
_register_raw_input_devices.restype = wintypes.BOOL
_get_raw_input_data = ctypes.windll.user32.GetRawInputData
_get_raw_input_data.argtypes = [wintypes.HANDLE, wintypes.UINT, wintypes.LPVOID,
                                ctypes.POINTER(wintypes.UINT), wintypes.UINT]
_get_raw_input_data.restype = wintypes.UINT


def register_raw_mouse(hwnd: int) -> None:
    device = RAWINPUTDEVICE(1, 2, RIDEV_INPUTSINK, hwnd)
    ok = _register_raw_input_devices(
        ctypes.byref(device), 1, ctypes.sizeof(RAWINPUTDEVICE))
    if not ok:
        raise ctypes.WinError()


def read_raw_mouse(message_ptr: int) -> tuple[int, int, bool | None, bool | None] | None:
    msg = MSG.from_address(message_ptr)
    if msg.message != WM_INPUT:
        return None
    size = wintypes.UINT(0)
    header_size = ctypes.sizeof(RAWINPUTHEADER)
    raw_handle = wintypes.HANDLE(msg.lParam)
    _get_raw_input_data(raw_handle, RID_INPUT, None, ctypes.byref(size), header_size)
    if not size.value:
        return None
    buffer = ctypes.create_string_buffer(size.value)
    copied = _get_raw_input_data(
        raw_handle, RID_INPUT, buffer, ctypes.byref(size), header_size)
    if copied != size.value:
        return None
    raw = ctypes.cast(buffer, ctypes.POINTER(RAWINPUT)).contents
    mouse = raw.data.mouse
    if mouse.usFlags & MOUSE_MOVE_ABSOLUTE:
        dx = dy = 0
    else:
        dx, dy = int(mouse.lLastX), int(mouse.lLastY)
    left: bool | None = None
    right: bool | None = None
    if mouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_DOWN:
        left = True
    elif mouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_UP:
        left = False
    if mouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_DOWN:
        right = True
    elif mouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_UP:
        right = False
    return dx, dy, left, right
