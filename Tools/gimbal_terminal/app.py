from __future__ import annotations

import sys
import threading
import time
import math
from pathlib import Path

import numpy as np
import cv2
import yaml

from PyQt5.QtCore import QEvent, QObject, QPoint, QPointF, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from calibration import IntrinsicCalibrationSession
from apriltag import AprilTagDetector, load_camera_calibration
from auto_aim import AimCommand, AttitudeBuffer, AutoAimEngine
from extrinsic import ExtrinsicCalibrationSession
from mvs_camera import CameraWorker, LatestFrame, MvsError, enumerate_hikrobot_devices
from protocol import (
    GimbalStatus,
    STATUS_CONTROL_ENABLED,
    STATUS_FIRE_REQUESTED,
    STATUS_IMU_VALID,
    STATUS_LINK_ALIVE,
    STATUS_PITCH_ONLINE,
    STATUS_SAFE,
    STATUS_YAW_ONLINE,
)
from raw_input import read_raw_mouse, register_raw_mouse
from serial_link import SerialLink, available_ports, preferred_port
from vision_worker import VisionWorker
from aim_logger import AimLogger


class EventBridge(QObject):
    camera_status = pyqtSignal(bool, str)
    serial_status = pyqtSignal(bool, str)
    telemetry = pyqtSignal(object)


class VideoWidget(QLabel):
    control_requested = pyqtSignal()

    def __init__(self) -> None:
        super().__init__("等待海康相机画面\n左键点击画面进入控制，Esc 退出")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 480)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background:#080b10; color:#8190a5; border:1px solid #273247;")
        self._image: QImage | None = None
        self._armed = False
        self._rotation_degrees = 90
        self._tag_poses = ()
        self._predicted_point: tuple[float, float] | None = None
        self._planned_point: tuple[float, float] | None = None

    def set_frame(self, image: QImage) -> None:
        self._image = image
        self.update()

    def set_armed(self, armed: bool) -> None:
        self._armed = armed
        self.update()

    def set_rotation(self, degrees: int) -> None:
        self._rotation_degrees = degrees % 360
        self.update()

    def set_tag_poses(self, poses) -> None:
        self._tag_poses = tuple(poses)
        self.update()

    def set_aim_points(self, predicted, planned) -> None:
        self._predicted_point = predicted
        self._planned_point = planned
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.control_requested.emit()
        super().mousePressEvent(event)

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        painter = QPainter(self)
        if self._image is not None:
            source_width = self._image.width()
            source_height = self._image.height()
            rotated_width = source_height if self._rotation_degrees in (90, 270) else source_width
            rotated_height = source_width if self._rotation_degrees in (90, 270) else source_height
            scale = min(self.width() / rotated_width, self.height() / rotated_height)
            painter.save()
            painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
            painter.translate(self.width() / 2.0, self.height() / 2.0)
            painter.rotate(self._rotation_degrees)
            painter.scale(scale, scale)
            painter.drawImage(QPoint(-source_width // 2, -source_height // 2), self._image)
            painter.restore()
            if self._rotation_degrees == 90:
                painter.setPen(QPen(QColor("#40c4ff"), 2))
                offset_x = (self.width() - rotated_width * scale) * 0.5
                offset_y = (self.height() - rotated_height * scale) * 0.5
                for pose in self._tag_poses:
                    points = [QPointF(offset_x + float(p[0]) * scale,
                                     offset_y + float(p[1]) * scale)
                              for p in pose.corners]
                    for index in range(4):
                        painter.drawLine(points[index], points[(index + 1) % 4])
                    painter.drawText(points[0], f"ID {pose.tag_id}")
                for point, color, label in (
                        (self._predicted_point, "#ffca28", "预测"),
                        (self._planned_point, "#e040fb", "规划")):
                    if point is None:
                        continue
                    display = QPointF(offset_x + point[0] * scale,
                                      offset_y + point[1] * scale)
                    painter.setPen(QPen(QColor(color), 2))
                    painter.drawEllipse(display, 7, 7)
                    painter.drawText(display + QPointF(9, -9), label)
        center = self.rect().center()
        color = QColor("#ff5252" if self._armed else "#69f0ae")
        painter.setPen(QPen(color, 2))
        painter.drawLine(center.x() - 18, center.y(), center.x() - 5, center.y())
        painter.drawLine(center.x() + 5, center.y(), center.x() + 18, center.y())
        painter.drawLine(center.x(), center.y() - 18, center.x(), center.y() - 5)
        painter.drawLine(center.x(), center.y() + 5, center.x(), center.y() + 18)
        painter.drawEllipse(center, 2, 2)
        if self._armed:
            painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
            painter.drawText(12, 24, "CONTROL ACTIVE — ESC TO RELEASE")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("HIKROBOT Gimbal Terminal")
        self.resize(1280, 760)
        self.settings = QSettings("RM-Gimbal", "HikrobotTerminal")
        self._loading_settings = True
        self.bridge = EventBridge()
        self.bridge.camera_status.connect(self._on_camera_status)
        self.bridge.serial_status.connect(self._on_serial_status)
        self.bridge.telemetry.connect(self._on_telemetry)

        self.frames = LatestFrame()
        self.attitudes = AttitudeBuffer()
        self.camera = CameraWorker(self.frames, self.bridge.camera_status.emit)
        self.serial = SerialLink(self.bridge.telemetry.emit, self.bridge.serial_status.emit,
                                 self.attitudes.put)
        self.camera_online = False
        self.serial_online = False
        self.protocol_ready = False
        self.armed = False
        self.fire = False
        self.fire_authorized = False
        self.auto_mode = False
        self.auto_engine: AutoAimEngine | None = None
        self.vision_worker: VisionWorker | None = None
        self.last_aim_command = AimCommand(False)
        self.extrinsic_session: ExtrinsicCalibrationSession | None = None
        self._camera_matrix = self._distortion = None
        self.aim_logger = AimLogger(Path(__file__).resolve().parent / "aim_logs")
        self.raw_dx = 0
        self.raw_dy = 0
        self._mouse_lock = threading.Lock()
        self._last_frame_id = -1
        self._cursor_hidden = False
        self._last_status: GimbalStatus | None = None
        self.calibration_session: IntrinsicCalibrationSession | None = None

        self._build_ui()
        self._load_settings()
        self._restore_latest_calibration_session()
        self._init_auto_aim()
        self._restore_latest_extrinsic_session()
        self._refresh_ports()
        self._loading_settings = False

        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self._display_latest_frame)
        self.display_timer.start(8)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self._send_control)
        self.control_timer.start(10)

        selected_port = self.port_combo.currentData() or ""
        self.serial.start(selected_port)
        self.camera.start(self.camera_combo.currentData() or "", self._selected_fps(),
                          self.exposure_spin.value(), self.gain_spin.value())

    def _init_auto_aim(self) -> None:
        base = Path(__file__).resolve().parent / "calibration_data"
        calibrations = sorted(base.glob("*/camera_calibration.yaml"),
                              key=lambda path: path.stat().st_mtime, reverse=True)
        if not calibrations:
            self.auto_state.setText("缺少相机内参")
            return
        matrix, distortion, size = load_camera_calibration(calibrations[0])
        self._camera_matrix, self._distortion = matrix, distortion
        if size != (1080, 1440):
            self.auto_state.setText(f"内参分辨率不匹配：{size[0]}×{size[1]}")
            return
        detector = AprilTagDetector(matrix, distortion, 0.100)
        self.auto_engine = AutoAimEngine(self.attitudes, self.bullet_speed.value())
        extrinsics = sorted((Path(__file__).resolve().parent / "extrinsic_data").glob(
            "*/camera_to_gimbal.yaml"), key=lambda path: path.stat().st_mtime,
            reverse=True)
        if extrinsics:
            self.auto_engine.load_extrinsic(extrinsics[0])
        self.vision_worker = VisionWorker(self.frames, detector, self.auto_engine)
        self.vision_worker.start()
        self.auto_state.setText("MANUAL / AprilTag 36h11 100 mm")

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        self.video = VideoWidget()
        self.video.control_requested.connect(self._arm_control)
        layout.addWidget(self.video, 1)

        panel_widget = QWidget()
        panel = QVBoxLayout(panel_widget)
        panel.setSpacing(10)
        panel_scroll = QScrollArea()
        panel_scroll.setWidgetResizable(True)
        panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        panel_scroll.setMinimumWidth(390)
        panel_scroll.setWidget(panel_widget)
        layout.addWidget(panel_scroll)

        connection_group = QGroupBox("连接")
        connection_form = QGridLayout(connection_group)
        self.camera_combo = QComboBox()
        self.camera_combo.addItem("自动选择海康相机", "")
        self.camera_combo.currentIndexChanged.connect(self._reconfigure_camera)
        camera_refresh = QPushButton("刷新")
        camera_refresh.clicked.connect(self._refresh_cameras)
        self.fps_combo = QComboBox()
        self.fps_combo.addItem("60 FPS", 60.0)
        self.fps_combo.addItem("120 FPS", 120.0)
        self.fps_combo.addItem("相机上限", 0.0)
        self.fps_combo.currentIndexChanged.connect(self._reconfigure_camera)
        self.exposure_spin = QDoubleSpinBox()
        self.exposure_spin.setRange(10.0, 1000000.0)
        self.exposure_spin.setDecimals(0)
        self.exposure_spin.setSingleStep(100.0)
        self.exposure_spin.setSuffix(" μs")
        self.exposure_spin.setKeyboardTracking(False)
        self.exposure_spin.valueChanged.connect(self._reconfigure_camera)
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(0.0, 48.0)
        self.gain_spin.setDecimals(2)
        self.gain_spin.setSingleStep(0.10)
        self.gain_spin.setSuffix(" dB")
        self.gain_spin.setKeyboardTracking(False)
        self.gain_spin.valueChanged.connect(self._reconfigure_camera)
        self.rotation_combo = QComboBox()
        self.rotation_combo.addItem("不旋转", 0)
        self.rotation_combo.addItem("顺时针 90°", 90)
        self.rotation_combo.addItem("旋转 180°", 180)
        self.rotation_combo.addItem("顺时针 270°", 270)
        self.rotation_combo.currentIndexChanged.connect(self._change_rotation)
        self.port_combo = QComboBox()
        port_refresh = QPushButton("刷新")
        port_refresh.clicked.connect(self._refresh_ports)
        self.port_combo.currentIndexChanged.connect(self._change_port)
        connection_form.addWidget(QLabel("相机"), 0, 0)
        connection_form.addWidget(self.camera_combo, 0, 1)
        connection_form.addWidget(camera_refresh, 0, 2)
        connection_form.addWidget(QLabel("帧率"), 1, 0)
        connection_form.addWidget(self.fps_combo, 1, 1, 1, 2)
        connection_form.addWidget(QLabel("曝光"), 2, 0)
        connection_form.addWidget(self.exposure_spin, 2, 1, 1, 2)
        connection_form.addWidget(QLabel("增益"), 3, 0)
        connection_form.addWidget(self.gain_spin, 3, 1, 1, 2)
        connection_form.addWidget(QLabel("画面方向"), 4, 0)
        connection_form.addWidget(self.rotation_combo, 4, 1, 1, 2)
        connection_form.addWidget(QLabel("主控"), 5, 0)
        connection_form.addWidget(self.port_combo, 5, 1)
        connection_form.addWidget(port_refresh, 5, 2)
        panel.addWidget(connection_group)

        control_group = QGroupBox("鼠标控制")
        control_form = QFormLayout(control_group)
        self.x_sensitivity = QDoubleSpinBox()
        self.y_sensitivity = QDoubleSpinBox()
        for spin in (self.x_sensitivity, self.y_sensitivity):
            spin.setRange(0.001, 1.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.005)
            spin.setSuffix(" °/count")
            spin.valueChanged.connect(self._save_settings)
        self.invert_x = QCheckBox("反转水平轴")
        self.invert_y = QCheckBox("反转垂直轴")
        self.invert_x.toggled.connect(self._save_settings)
        self.invert_y.toggled.connect(self._save_settings)
        control_form.addRow("水平灵敏度", self.x_sensitivity)
        control_form.addRow("垂直灵敏度", self.y_sensitivity)
        control_form.addRow(self.invert_x)
        control_form.addRow(self.invert_y)
        panel.addWidget(control_group)

        telemetry_group = QGroupBox("状态遥测")
        telemetry_form = QFormLayout(telemetry_group)
        self.camera_state = QLabel("未连接")
        self.serial_state = QLabel("未连接")
        self.control_state = QLabel("未使能")
        self.fps_state = QLabel("0 FPS")
        self.angle_state = QLabel("Pitch -- / Yaw --")
        self.motor_state = QLabel("Pitch -- / Yaw --")
        self.protocol_state = QLabel("CRC/协议错误 --")
        telemetry_form.addRow("相机", self.camera_state)
        telemetry_form.addRow("主控", self.serial_state)
        telemetry_form.addRow("控制", self.control_state)
        telemetry_form.addRow("图像", self.fps_state)
        telemetry_form.addRow("角度", self.angle_state)
        telemetry_form.addRow("电机", self.motor_state)
        telemetry_form.addRow("通信", self.protocol_state)
        panel.addWidget(telemetry_group)

        calibration_group = QGroupBox("相机内参标定")
        calibration_layout = QGridLayout(calibration_group)
        self.pattern_cols = QSpinBox()
        self.pattern_cols.setRange(3, 30)
        self.pattern_cols.setValue(10)
        self.pattern_cols.valueChanged.connect(self._save_settings)
        self.pattern_rows = QSpinBox()
        self.pattern_rows.setRange(3, 30)
        self.pattern_rows.setValue(7)
        self.pattern_rows.valueChanged.connect(self._save_settings)
        self.pattern_spacing = QDoubleSpinBox()
        self.pattern_spacing.setRange(1.0, 200.0)
        self.pattern_spacing.setDecimals(2)
        self.pattern_spacing.setValue(40.0)
        self.pattern_spacing.setSuffix(" mm")
        self.pattern_spacing.valueChanged.connect(self._save_settings)
        self.capture_calibration_button = QPushButton("采集有效帧")
        self.capture_calibration_button.clicked.connect(self._capture_calibration_frame)
        self.solve_calibration_button = QPushButton("计算内参")
        self.solve_calibration_button.clicked.connect(self._solve_calibration)
        self.new_calibration_button = QPushButton("新建会话")
        self.new_calibration_button.clicked.connect(self._new_calibration_session)
        self.restore_calibration_button = QPushButton("恢复最近会话")
        self.restore_calibration_button.clicked.connect(
            self._restore_latest_calibration_session)
        self.calibration_state = QLabel("0 张；建议采集15～25张")
        self.calibration_state.setWordWrap(True)
        calibration_layout.addWidget(QLabel("圆点列×行"), 0, 0)
        board_size_layout = QHBoxLayout()
        board_size_layout.addWidget(self.pattern_cols)
        board_size_layout.addWidget(QLabel("×"))
        board_size_layout.addWidget(self.pattern_rows)
        calibration_layout.addLayout(board_size_layout, 0, 1, 1, 2)
        calibration_layout.addWidget(QLabel("圆心距"), 1, 0)
        calibration_layout.addWidget(self.pattern_spacing, 1, 1, 1, 2)
        calibration_layout.addWidget(self.capture_calibration_button, 2, 0)
        calibration_layout.addWidget(self.solve_calibration_button, 2, 1)
        calibration_layout.addWidget(self.new_calibration_button, 2, 2)
        calibration_layout.addWidget(self.restore_calibration_button, 3, 0, 1, 3)
        calibration_layout.addWidget(self.calibration_state, 4, 0, 1, 3)
        panel.addWidget(calibration_group)

        auto_group = QGroupBox("AprilTag 自动瞄准")
        auto_layout = QFormLayout(auto_group)
        self.auto_state = QLabel("初始化中")
        self.auto_state.setWordWrap(True)
        self.tag_state = QLabel("未发现目标")
        self.tag_state.setWordWrap(True)
        self.bullet_speed = QDoubleSpinBox()
        self.bullet_speed.setRange(1.0, 50.0)
        self.bullet_speed.setDecimals(1)
        self.bullet_speed.setValue(15.0)
        self.bullet_speed.setSuffix(" m/s")
        self.bullet_speed.valueChanged.connect(self._save_settings)
        auto_layout.addRow("状态", self.auto_state)
        auto_layout.addRow("目标", self.tag_state)
        auto_layout.addRow("估计弹速", self.bullet_speed)
        panel.addWidget(auto_group)

        extrinsic_group = QGroupBox("相机—云台外参")
        extrinsic_layout = QGridLayout(extrinsic_group)
        capture_extrinsic = QPushButton("采集静止姿态")
        capture_extrinsic.clicked.connect(self._capture_extrinsic)
        solve_extrinsic = QPushButton("计算外参")
        solve_extrinsic.clicked.connect(self._solve_extrinsic)
        self.extrinsic_state = QLabel("0 / 25；固定平整圆点板并改变云台姿态")
        self.extrinsic_state.setWordWrap(True)
        extrinsic_layout.addWidget(capture_extrinsic, 0, 0)
        extrinsic_layout.addWidget(solve_extrinsic, 0, 1)
        extrinsic_layout.addWidget(self.extrinsic_state, 1, 0, 1, 2)
        panel.addWidget(extrinsic_group)

        self.estop_button = QPushButton("急停 / 失能")
        self.estop_button.setMinimumHeight(48)
        self.estop_button.setStyleSheet(
            "QPushButton{background:#b71c1c;color:white;font-weight:bold;border-radius:4px;}"
            "QPushButton:hover{background:#d32f2f;}"
        )
        self.estop_button.clicked.connect(lambda: self._release_control(True))
        panel.addWidget(self.estop_button)
        help_label = QLabel("左键点击：进入控制；自动模式中按住为开火授权\n"
                            "右键：锁定/退出 AprilTag 自动瞄准\n"
                            "F8：保持使能并采集外参静止姿态\n"
                            "Esc / 失焦：退出并失能\nF11：切换全屏")
        help_label.setStyleSheet("color:#8b98ab;")
        panel.addWidget(help_label)
        panel.addStretch(1)

    def _load_settings(self) -> None:
        self.x_sensitivity.setValue(float(self.settings.value("sensitivity_x", 0.030)))
        self.y_sensitivity.setValue(float(self.settings.value("sensitivity_y", 0.030)))
        self.invert_x.setChecked(self.settings.value("invert_x", False, type=bool))
        self.invert_y.setChecked(self.settings.value("invert_y", False, type=bool))
        fps = float(self.settings.value("camera_fps", 120.0))
        index = self.fps_combo.findData(fps)
        self.fps_combo.setCurrentIndex(max(0, index))
        self.exposure_spin.setValue(float(self.settings.value("exposure_us", 8000.0)))
        self.gain_spin.setValue(float(self.settings.value("gain_db", 16.08)))
        rotation = int(self.settings.value("rotation_degrees", 90))
        rotation_index = self.rotation_combo.findData(rotation)
        self.rotation_combo.setCurrentIndex(max(0, rotation_index))
        self.video.set_rotation(int(self.rotation_combo.currentData()))
        self.pattern_cols.setValue(int(self.settings.value("pattern_cols", 10)))
        self.pattern_rows.setValue(int(self.settings.value("pattern_rows", 7)))
        # Existing sessions created before persistence used the supplied A4 board.
        self.pattern_spacing.setValue(float(
            self.settings.value("pattern_spacing_mm", 20.0)))
        self.bullet_speed.setValue(float(self.settings.value("bullet_speed_m_s", 15.0)))

    def _save_settings(self) -> None:
        if self._loading_settings:
            return
        self.settings.setValue("sensitivity_x", self.x_sensitivity.value())
        self.settings.setValue("sensitivity_y", self.y_sensitivity.value())
        self.settings.setValue("invert_x", self.invert_x.isChecked())
        self.settings.setValue("invert_y", self.invert_y.isChecked())
        self.settings.setValue("camera_fps", self._selected_fps())
        self.settings.setValue("exposure_us", self.exposure_spin.value())
        self.settings.setValue("gain_db", self.gain_spin.value())
        self.settings.setValue("rotation_degrees", self.rotation_combo.currentData())
        self.settings.setValue("serial_port", self.port_combo.currentData() or "")
        self.settings.setValue("pattern_cols", self.pattern_cols.value())
        self.settings.setValue("pattern_rows", self.pattern_rows.value())
        self.settings.setValue("pattern_spacing_mm", self.pattern_spacing.value())
        self.settings.setValue("bullet_speed_m_s", self.bullet_speed.value())

    def _selected_fps(self) -> float:
        return float(self.fps_combo.currentData() or 0.0)

    def _refresh_ports(self) -> None:
        configured = str(self.settings.value("serial_port", ""))
        current = self.port_combo.currentData() or configured
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        ports = available_ports()
        for device, label in ports:
            self.port_combo.addItem(label, device)
        selected = preferred_port(str(current))
        if selected:
            index = self.port_combo.findData(selected)
            self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)

    def _refresh_cameras(self) -> None:
        try:
            devices = enumerate_hikrobot_devices()
        except MvsError as exc:
            self.camera_state.setText(str(exc))
            return
        current = self.camera_combo.currentData() or ""
        self.camera_combo.blockSignals(True)
        self.camera_combo.clear()
        self.camera_combo.addItem("自动选择海康相机", "")
        for device in devices:
            self.camera_combo.addItem(device.label, device.serial)
        index = self.camera_combo.findData(current)
        self.camera_combo.setCurrentIndex(max(0, index))
        self.camera_combo.blockSignals(False)

    def _change_port(self) -> None:
        self._release_control()
        self.serial.set_port(self.port_combo.currentData() or "")
        self._save_settings()

    def _reconfigure_camera(self) -> None:
        if hasattr(self, "camera"):
            self.camera.configure(self.camera_combo.currentData() or "", self._selected_fps(),
                                  self.exposure_spin.value(), self.gain_spin.value())
        self._save_settings()

    def _change_rotation(self) -> None:
        self.video.set_rotation(int(self.rotation_combo.currentData()))
        self._save_settings()

    def _new_calibration_session(self) -> None:
        self.calibration_session = None
        self.pattern_cols.setEnabled(True)
        self.pattern_rows.setEnabled(True)
        self.pattern_spacing.setEnabled(True)
        self.rotation_combo.setEnabled(True)
        self.calibration_state.setText("0 张；建议采集15～25张")

    def _restore_latest_calibration_session(self) -> None:
        base = Path(__file__).resolve().parent / "calibration_data"
        candidates = sorted(
            (folder for folder in base.glob("*")
             if folder.is_dir() and any((folder / "images").glob("*.png"))),
            key=lambda folder: folder.stat().st_mtime,
            reverse=True)
        if not candidates:
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            session, skipped = IntrinsicCalibrationSession.load(
                candidates[0], self.pattern_cols.value(), self.pattern_rows.value(),
                self.pattern_spacing.value(), int(self.rotation_combo.currentData()))
        except (ValueError, OSError, yaml.YAMLError) as exc:
            self.calibration_state.setText(f"恢复标定会话失败：{exc}")
            return
        finally:
            QApplication.restoreOverrideCursor()
        self.calibration_session = session
        self.pattern_cols.setValue(session.columns)
        self.pattern_rows.setValue(session.rows)
        self.pattern_spacing.setValue(session.spacing_mm)
        rotation_index = self.rotation_combo.findData(session.rotation_degrees)
        if rotation_index >= 0:
            self.rotation_combo.setCurrentIndex(rotation_index)
        self.pattern_cols.setEnabled(False)
        self.pattern_rows.setEnabled(False)
        self.pattern_spacing.setEnabled(False)
        self.rotation_combo.setEnabled(False)
        suffix = f"；跳过 {skipped} 张" if skipped else ""
        self.calibration_state.setText(
            f"已恢复 {session.count} 张有效图{suffix}\n{session.folder}")
        self._save_settings()

    def _ensure_calibration_session(self) -> IntrinsicCalibrationSession:
        columns = self.pattern_cols.value()
        rows = self.pattern_rows.value()
        spacing = self.pattern_spacing.value()
        rotation = int(self.rotation_combo.currentData())
        if self.calibration_session is None:
            base = Path(__file__).resolve().parent / "calibration_data"
            self.calibration_session = IntrinsicCalibrationSession(
                base, columns, rows, spacing, rotation)
        else:
            self.calibration_session.configure(columns, rows, spacing, rotation)
        return self.calibration_session

    def _capture_calibration_frame(self) -> None:
        self._release_control()
        frame = self.frames.get(-1)
        if frame is None or not self.camera_online:
            self.calibration_state.setText("没有可用的海康相机图像")
            return
        try:
            session = self._ensure_calibration_session()
            raw = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(
                frame.height, frame.width, 3)
            result = session.capture(raw)
        except (ValueError, OSError) as exc:
            self.calibration_state.setText(str(exc))
            return
        if result.success:
            self.pattern_cols.setEnabled(False)
            self.pattern_rows.setEnabled(False)
            self.pattern_spacing.setEnabled(False)
            self.rotation_combo.setEnabled(False)
            self.calibration_state.setText(
                f"{session.count} 张有效图；{result.message}\n{session.folder}")
        else:
            self.calibration_state.setText(result.message)

    def _solve_calibration(self) -> None:
        self._release_control()
        if self.calibration_session is None:
            self.calibration_state.setText("尚未采集有效标定图片")
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            result = self.calibration_session.calibrate(minimum_images=12)
        except (ValueError, OSError) as exc:
            self.calibration_state.setText(str(exc))
            return
        finally:
            QApplication.restoreOverrideCursor()
        quality = ("优秀" if result.mean_reprojection_error < 0.30 else
                   "良好" if result.mean_reprojection_error < 0.50 else
                   "可用" if result.mean_reprojection_error < 1.00 else "建议重采")
        message = (f"标定完成：平均重投影误差 "
                   f"{result.mean_reprojection_error:.4f} px（{quality}）\n"
                   f"OpenCV RMS：{result.rms:.4f}\n{result.output_path}")
        self.calibration_state.setText(message)
        QMessageBox.information(self, "相机内参标定", message)

    def _capture_extrinsic(self) -> None:
        if self._camera_matrix is None or self._distortion is None:
            self.extrinsic_state.setText("缺少相机内参")
            return
        frame = self.frames.get(-1)
        if frame is None:
            self.extrinsic_state.setText("没有相机图像")
            return
        attitude, quality = self.attitudes.capture_quality(frame.capture_time_ns)
        if attitude is None:
            self.extrinsic_state.setText(f"外参样本被拒绝：{quality}")
            return
        if self.extrinsic_session is None:
            base = Path(__file__).resolve().parent / "extrinsic_data"
            self.extrinsic_session = ExtrinsicCalibrationSession(
                base, self._camera_matrix, self._distortion, 10, 7, 20.0)
        raw = np.frombuffer(frame.rgb, np.uint8).reshape(frame.height, frame.width, 3)
        if not self.extrinsic_session.capture(raw, attitude):
            self.extrinsic_state.setText(
                f"外参样本被拒绝：{self.extrinsic_session.last_error}")
            return
        self.extrinsic_state.setText(
            f"{self.extrinsic_session.count} / 25；{quality}\n"
            f"温度 {attitude.temperature_cdeg/100:.2f}℃ / "
            f"标定 {attitude.calibration_temperature_cdeg/100:.2f}℃\n"
            f"{self.extrinsic_session.folder}")

    def _restore_latest_extrinsic_session(self) -> None:
        if self._camera_matrix is None or self._distortion is None:
            return
        base = Path(__file__).resolve().parent / "extrinsic_data"
        candidates = sorted(
            (folder for folder in base.glob("*")
             if folder.is_dir() and any(folder.glob("*.npz")) and
             not (folder / "camera_to_gimbal.yaml").is_file()),
            key=lambda folder: folder.stat().st_mtime, reverse=True)
        if not candidates:
            return
        try:
            self.extrinsic_session = ExtrinsicCalibrationSession.restore(
                candidates[0], self._camera_matrix, self._distortion,
                10, 7, 20.0)
        except (OSError, ValueError, cv2.error):
            return
        self.extrinsic_state.setText(
            f"已恢复并统一圆点方向：{self.extrinsic_session.count} / 25\n"
            f"{self.extrinsic_session.folder}")

    def _solve_extrinsic(self) -> None:
        if self.extrinsic_session is None:
            self.extrinsic_state.setText("尚未采集外参姿态")
            return
        try:
            path = self.extrinsic_session.solve(25)
            if self.auto_engine is not None:
                self.auto_engine.load_extrinsic(path)
        except (ValueError, OSError, cv2.error) as exc:
            self.extrinsic_state.setText(str(exc))
            return
        self.extrinsic_state.setText(f"外参完成（开发参数，尚未通过最终0.5°验收）\n{path}")
        QMessageBox.information(
            self, "相机—云台外参",
            f"外参完成，可用于无发射器流程联调。\n"
            f"当前为开发参数，尚未通过最终0.5°最大误差验收。\n{path}")

    def showEvent(self, event) -> None:
        super().showEvent(event)
        try:
            register_raw_mouse(int(self.winId()))
        except OSError as exc:
            self.serial_state.setText(f"Raw Input 初始化失败：{exc}")

    def nativeEvent(self, event_type, message):
        raw = read_raw_mouse(int(message))
        if raw is not None:
            dx, dy, left, right = raw
            if left is not None and self.armed:
                self.fire_authorized = left
            if right is True and self.armed:
                self._toggle_auto_aim()
            if self.armed and self.camera_online and not self.auto_mode:
                with self._mouse_lock:
                    self.raw_dx += dx
                    self.raw_dy += dy
        return super().nativeEvent(event_type, message)

    def _toggle_auto_aim(self) -> None:
        if self.vision_worker is None:
            self.auto_state.setText("自动瞄准不可用：缺少有效内参")
            return
        if self.auto_mode:
            self.vision_worker.unlock()
            self.auto_mode = False
            self.fire_authorized = False
            self.auto_state.setText("MANUAL")
            return
        if int(self.rotation_combo.currentData()) != 90:
            self.auto_state.setText("自动瞄准要求画面保持顺时针 90°")
            return
        locked = self.vision_worker.lock_nearest((540.0, 720.0))
        if locked is None:
            self.auto_state.setText("准星附近没有合格 AprilTag")
            return
        self.auto_mode = True
        self.auto_state.setText(f"AUTO_TRACK / ID {locked}")

    def _arm_control(self) -> None:
        if not self.camera_online:
            self.statusBar().showMessage("相机未连接，不能进入鼠标控制", 3000)
            return
        if not self.serial_online or not self.protocol_ready:
            self.statusBar().showMessage("主控协议尚未握手", 3000)
            return
        if self._last_status is None or not self._last_status.has(STATUS_SAFE):
            self.statusBar().showMessage("IMU 或云台电机未就绪", 3000)
            return
        self.armed = True
        self.video.set_armed(True)
        self.control_state.setText("已使能（右键预留）")
        if not self._cursor_hidden:
            QApplication.setOverrideCursor(Qt.BlankCursor)
            self._cursor_hidden = True
        self.activateWindow()
        self.video.setFocus()

    def _release_control(self, emergency: bool = False) -> None:
        was_armed = self.armed
        self.armed = False
        self.fire = False
        self.fire_authorized = False
        self.auto_mode = False
        if self.vision_worker is not None:
            self.vision_worker.unlock()
        with self._mouse_lock:
            self.raw_dx = self.raw_dy = 0
        self.video.set_armed(False)
        self.control_state.setText("急停" if emergency else "未使能")
        if self._cursor_hidden:
            QApplication.restoreOverrideCursor()
            self._cursor_hidden = False
        if was_armed or emergency:
            self.serial.disarm(emergency)

    def _send_control(self) -> None:
        if not self.serial_online:
            return
        if self.auto_mode and self.auto_engine is not None:
            self.auto_engine.bullet_speed = self.bullet_speed.value()
            now_ns = time.perf_counter_ns()
            command = self.auto_engine.command(now_ns, self.fire_authorized)
            self.last_aim_command = command
            execute_at = self.serial.mcu_time_ms()
            valid = command.valid and execute_at is not None and self.camera_online
            self.serial.send_aim_target(
                valid, self.armed, command.fire if valid else False,
                ((execute_at or 0) + 10) & 0xFFFFFFFF,
                round(math.degrees(command.yaw) * 1000),
                round(math.degrees(command.pitch) * 1000),
                round(math.degrees(command.yaw_rate) * 1000),
                round(math.degrees(command.pitch_rate) * 1000),
                round(math.degrees(command.yaw_accel) * 1000),
                round(math.degrees(command.pitch_accel) * 1000))
            self.fire = command.fire if valid else False
            observation_age_ns = now_ns - self.auto_engine.last_observation_ns
            state = ("AUTO_ACQUIRE" if not command.valid else
                     "AUTO_COAST" if observation_age_ns > 30_000_000 else
                     "AUTO_TRACK")
            self.auto_state.setText(f"{state} / {command.gate_reason}")
            attitude = self.attitudes.latest()
            predicted_pixel = None
            planned_pixel = None
            if (attitude is not None and command.target_world is not None and
                    self._camera_matrix is not None):
                predicted_pixel = self.auto_engine.project_world_point(
                    command.target_world, attitude, self._camera_matrix,
                    self._distortion)
                cp = math.cos(command.pitch)
                planned_world = max(command.distance, 1.0) * np.array([
                    cp * math.cos(command.yaw), cp * math.sin(command.yaw),
                    math.sin(command.pitch)])
                planned_pixel = self.auto_engine.project_world_point(
                    planned_world, attitude, self._camera_matrix,
                    self._distortion)
            self.video.set_aim_points(predicted_pixel, planned_pixel)
            snap = self.vision_worker.snapshot() if self.vision_worker else None
            self.aim_logger.put({
                "host_time_ns": time.perf_counter_ns(), "mode": "AUTO",
                "tag_id": snap.locked_id if snap else "",
                "distance_m": command.distance,
                "vision_ms": snap.processing_ms if snap else "",
                "actual_yaw_deg": attitude.yaw_mdeg/1000 if attitude else "",
                "actual_pitch_deg": attitude.pitch_mdeg/1000 if attitude else "",
                "command_yaw_deg": math.degrees(command.yaw),
                "command_pitch_deg": math.degrees(command.pitch),
                "yaw_rate_deg_s": math.degrees(command.yaw_rate),
                "pitch_rate_deg_s": math.degrees(command.pitch_rate),
                "yaw_accel_deg_s2": math.degrees(command.yaw_accel),
                "pitch_accel_deg_s2": math.degrees(command.pitch_accel),
                "fire_authorized": int(self.fire_authorized), "fire": int(self.fire),
                "saturated": int(command.saturated),
                "aim_error_deg": math.degrees(command.aim_error_rad),
                "tracking_error_deg": math.degrees(command.tracking_error_rad),
                "target_world_x_m": command.target_world[0] if command.target_world is not None else "",
                "target_world_y_m": command.target_world[1] if command.target_world is not None else "",
                "target_world_z_m": command.target_world[2] if command.target_world is not None else "",
                "gate_reason": command.gate_reason})
            return
        self.video.set_aim_points(None, None)
        with self._mouse_lock:
            dx, dy = self.raw_dx, self.raw_dy
            self.raw_dx = self.raw_dy = 0
        if not self.armed or not self.camera_online:
            dx = dy = 0
        if self.invert_x.isChecked():
            dx = -dx
        if self.invert_y.isChecked():
            dy = -dy
        yaw_mdeg = round(dx * self.x_sensitivity.value() * 1000.0)
        # Windows raw Y is positive downward, while protocol Pitch is positive upward.
        pitch_mdeg = round(-dy * self.y_sensitivity.value() * 1000.0)
        self.serial.send_control(self.armed, self.fire and self.armed,
                                 yaw_mdeg, pitch_mdeg)

    def _display_latest_frame(self) -> None:
        frame = self.frames.get(self._last_frame_id)
        if frame is None:
            return
        self._last_frame_id = frame.frame_id
        image = QImage(frame.rgb, frame.width, frame.height,
                       frame.width * 3, QImage.Format_RGB888).copy()
        self.video.set_frame(image)
        self.fps_state.setText(f"{frame.fps:.0f} FPS / {frame.width}×{frame.height}")
        if self.vision_worker is not None:
            snap = self.vision_worker.snapshot()
            self.video.set_tag_poses(snap.poses)
            if snap.poses:
                pose = snap.poses[0]
                observation = (" / 姿态OK" if snap.observation_valid else
                               (f" / {snap.observation_reason}"
                                if snap.locked_id is not None else ""))
                self.tag_state.setText(
                    f"ID {pose.tag_id} / {np.linalg.norm(pose.tvec_m):.2f} m / "
                    f"重投影 {pose.reprojection_error_px:.2f}px / "
                    f"{snap.processing_ms:.1f}ms{observation}")
            else:
                self.tag_state.setText(f"未发现目标 / 检测 {snap.processing_ms:.1f}ms")

    def _on_camera_status(self, connected: bool, detail: str) -> None:
        self.camera_online = connected
        self.camera_state.setText(detail)
        self.camera_state.setStyleSheet("color:#69f0ae;" if connected else "color:#ff8a80;")
        if not connected:
            self.fire = False
            with self._mouse_lock:
                self.raw_dx = self.raw_dy = 0
            if self.armed:
                self.control_state.setText("画面掉线，保持最后角度")

    def _on_serial_status(self, connected: bool, detail: str) -> None:
        self.serial_online = connected
        self.serial_state.setText(detail)
        self.serial_state.setStyleSheet("color:#69f0ae;" if connected else "color:#ff8a80;")
        if not connected:
            self.protocol_ready = False
            self._release_control()

    def _on_telemetry(self, status: GimbalStatus) -> None:
        self._last_status = status
        self.protocol_ready = status.protocol_version == 1 and status.has(STATUS_LINK_ALIVE)
        self.angle_state.setText(
            f"P {status.pitch_mdeg / 1000:+.2f}° / Y {status.yaw_mdeg / 1000:+.2f}°"
        )
        pitch_ok = status.has(STATUS_PITCH_ONLINE)
        yaw_ok = status.has(STATUS_YAW_ONLINE)
        self.motor_state.setText(
            f"P {'OK' if pitch_ok else 'OFF'} {status.pitch_temperature}℃ / "
            f"Y {'OK' if yaw_ok else 'OFF'} {status.yaw_temperature}℃"
        )
        fire = " / FIRE预留=1" if status.has(STATUS_FIRE_REQUESTED) else ""
        self.protocol_state.setText(
            f"错误 {status.protocol_errors} / ACK {status.last_control_sequence}{fire}"
        )
        if status.protocol_version != 1:
            self.control_state.setText(f"协议版本不兼容：{status.protocol_version}")
            self._release_control()
        elif self.armed and not status.has(STATUS_CONTROL_ENABLED):
            self.control_state.setText("主控拒绝使能，请检查安全状态")

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Escape:
            self._release_control()
            event.accept()
            return
        if event.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            event.accept()
            return
        if event.key() == Qt.Key_F8:
            self._capture_extrinsic()
            event.accept()
            return
        super().keyPressEvent(event)

    def changeEvent(self, event) -> None:
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow() and self.armed:
            self._release_control()
        super().changeEvent(event)

    def closeEvent(self, event) -> None:
        self._release_control()
        self.camera.stop()
        if self.vision_worker is not None:
            self.vision_worker.stop()
        self.serial.stop()
        self.aim_logger.close()
        event.accept()


def main() -> int:
    if sys.platform != "win32":
        print("This terminal currently requires Windows Raw Input and HIKROBOT MVS for Windows.")
        return 2
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(
        "QWidget{background:#111720;color:#d9e2ef;font-family:'Segoe UI';font-size:10pt;}"
        "QGroupBox{border:1px solid #334158;border-radius:5px;margin-top:8px;padding-top:8px;}"
        "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}"
        "QComboBox,QDoubleSpinBox{background:#1b2635;border:1px solid #3b4c67;padding:4px;}"
        "QPushButton{background:#24344a;border:1px solid #405675;padding:5px;border-radius:3px;}"
        "QPushButton:hover{background:#304866;}"
    )
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
