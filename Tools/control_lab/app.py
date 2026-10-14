from __future__ import annotations

import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from control_lab.data_manager import ExperimentIndex
    from control_lab.metrics import velocity_metrics
else:
    from .data_manager import ExperimentIndex
    from .metrics import velocity_metrics


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOLS_ROOT = PROJECT_ROOT / "Tools"
EXPERIMENTS_ROOT = PROJECT_ROOT / "Experiments"
DEFAULT_SESSION = EXPERIMENTS_ROOT / "2026-10-01_mech-v2_yaw-sysid"
MATLAB = Path(r"E:\MATLAB\R2024a\bin\matlab.exe")
CMAKE = Path(r"C:\ST\STM32CubeCLT_1.18.0\CMake\bin\cmake.exe")
BUILD_DIR = PROJECT_ROOT / "cmake-build-debug-stm32"


DEFAULT_PARAMS = {
    "pid_kp": 80.0,
    "pid_ki": 300.0,
    "pid_integral_limit": 5.0,
    "positive_friction": 734.3,
    "positive_velocity": 102.1,
    "negative_friction": 486.4,
    "negative_velocity": 98.9,
    "friction_blend_rpm": 2.5,
    "friction_slew_per_s": 5000.0,
    "zero_target_rpm": 0.5,
    "integral_leak_tau_s": 0.1,
}

PARAM_LABELS = {
    "pid_kp": "PID Kp", "pid_ki": "PID Ki", "pid_integral_limit": "积分限幅",
    "positive_friction": "+摩擦", "positive_velocity": "+速度斜率",
    "negative_friction": "-摩擦", "negative_velocity": "-速度斜率",
    "friction_blend_rpm": "Blend rpm", "friction_slew_per_s": "摩擦斜率/s",
    "zero_target_rpm": "零速阈值", "integral_leak_tau_s": "泄放τ(s)",
}
PARAM_BOUNDS = {
    "pid_kp": (0, 500), "pid_ki": (0, 2000), "pid_integral_limit": (0, 20),
    "positive_friction": (0, 3000), "positive_velocity": (0, 300),
    "negative_friction": (0, 3000), "negative_velocity": (0, 300),
    "friction_blend_rpm": (.1, 10), "friction_slew_per_s": (100, 30000),
    "zero_target_rpm": (0, 3), "integral_leak_tau_s": (.01, 2),
}
TUNING_RANGES = {
    "pid_kp": (40, 140, 20), "pid_ki": (0, 600, 100),
    "pid_integral_limit": (2, 8, 1),
    "positive_friction": (600, 850, 50), "positive_velocity": (80, 130, 10),
    "negative_friction": (350, 650, 50), "negative_velocity": (80, 130, 10),
    "friction_blend_rpm": (1.0, 3.0, 0.5),
    "friction_slew_per_s": (2500, 7500, 1000),
    "zero_target_rpm": (0.25, 1.0, 0.25),
    "integral_leak_tau_s": (0.05, 0.25, 0.05),
}


class ControlLab(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Gimbal Control Lab — 系统辨识与自动调参")
        self.resize(1180, 780)
        self.process: QtCore.QProcess | None = None
        self.process_callback = None
        self.tuning_queue: list[tuple[dict, Path]] = []
        self.sweep_results: list[tuple[float, dict, Path, dict]] = []
        self.tune_dimensions: list[tuple[str, list[float]]] = []
        self.tune_dimension_index = 0
        self.tune_round = 0
        self.tune_base_params: dict[str, float] = {}
        self.dimension_results: list[tuple[float, dict, Path, dict]] = []
        self.tune_reference_pitch: float | None = None
        self.last_velocity_csv: Path | None = None
        self.settings = QtCore.QSettings("DM", "GimbalControlLab")
        self.index = ExperimentIndex(EXPERIMENTS_ROOT)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        session_row = QtWidgets.QHBoxLayout()
        session_row.addWidget(QtWidgets.QLabel("实验目录"))
        self.session_edit = QtWidgets.QLineEdit(
            self.settings.value("session", str(DEFAULT_SESSION)))
        session_row.addWidget(self.session_edit, 1)
        choose = QtWidgets.QPushButton("选择…")
        choose.clicked.connect(self.choose_session)
        session_row.addWidget(choose)
        self.port_edit = QtWidgets.QLineEdit(self.settings.value("port", "COM7"))
        self.port_edit.setMaximumWidth(100)
        session_row.addWidget(QtWidgets.QLabel("串口"))
        session_row.addWidget(self.port_edit)
        outer.addLayout(session_row)

        self.tabs = QtWidgets.QTabWidget()
        outer.addWidget(self.tabs, 1)
        self.tabs.addTab(self.make_sysid_tab(), "系统辨识")
        self.tabs.addTab(self.make_tuning_tab(), "控制调参")
        self.tabs.addTab(self.make_data_tab(), "数据管理")

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setPlaceholderText("采集、MATLAB和评分日志")
        outer.addWidget(self.log, 0)
        self.statusBar().showMessage("就绪")
        self.refresh_data()

    @staticmethod
    def spin(value: float, minimum: float, maximum: float, decimals: int = 2) -> QtWidgets.QDoubleSpinBox:
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def make_sysid_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        capture = QtWidgets.QGroupBox("采集")
        form = QtWidgets.QGridLayout(capture)
        self.sysid_axis = QtWidgets.QComboBox(); self.sysid_axis.addItems(["yaw", "pitch"])
        self.sysid_type = QtWidgets.QComboBox(); self.sysid_type.addItems(["prbs", "step"])
        self.sysid_amp = self.spin(1200, 1, 2000, 0)
        self.sysid_switch = self.spin(40, 5, 1000, 0)
        self.sysid_duration = self.spin(20000, 100, 60000, 0)
        self.sysid_seed = QtWidgets.QLineEdit("0x13572468")
        widgets = [("轴", self.sysid_axis), ("激励", self.sysid_type), ("幅值", self.sysid_amp),
                   ("切换周期 ms", self.sysid_switch), ("时长 ms", self.sysid_duration),
                   ("随机种子", self.sysid_seed)]
        for column, (label, widget) in enumerate(widgets):
            form.addWidget(QtWidgets.QLabel(label), 0, column)
            form.addWidget(widget, 1, column)
        button = QtWidgets.QPushButton("开始采集")
        button.clicked.connect(self.capture_sysid)
        form.addWidget(button, 1, len(widgets))
        build = QtWidgets.QPushButton("构建辨识固件")
        build.clicked.connect(lambda: self.build_firmware(True))
        form.addWidget(build, 0, len(widgets))
        self.sysid_auto_position = QtWidgets.QCheckBox("Pitch PRBS自动预定位")
        self.sysid_operating_angle = self.spin(0, -20, 20, 1)
        form.addWidget(self.sysid_auto_position, 2, 0, 1, 2)
        form.addWidget(QtWidgets.QLabel("工作角 °"), 2, 2)
        form.addWidget(self.sysid_operating_angle, 2, 3)
        layout.addWidget(capture)

        gravity = QtWidgets.QGroupBox("Pitch 重力矩扫描")
        gravity_form = QtWidgets.QGridLayout(gravity)
        self.gravity_min = self.spin(-30, -45, 44, 1)
        self.gravity_max = self.spin(30, -44, 45, 1)
        self.gravity_step = self.spin(15, 1, 45, 1)
        self.gravity_ramp = self.spin(1500, 200, 5000, 0)
        self.gravity_settle = self.spin(1500, 200, 5000, 0)
        self.gravity_repetitions = self.spin(1, 1, 5, 0)
        gravity_widgets = [
            ("最小角 °", self.gravity_min), ("最大角 °", self.gravity_max),
            ("步长 °", self.gravity_step), ("斜坡 ms", self.gravity_ramp),
            ("停稳 ms", self.gravity_settle), ("往返次数", self.gravity_repetitions),
        ]
        for column, (label, widget) in enumerate(gravity_widgets):
            gravity_form.addWidget(QtWidgets.QLabel(label), 0, column)
            gravity_form.addWidget(widget, 1, column)
        gravity_start = QtWidgets.QPushButton("开始重力扫描")
        gravity_start.clicked.connect(self.capture_pitch_gravity)
        gravity_form.addWidget(gravity_start, 1, len(gravity_widgets))
        self.gravity_csv_edit = QtWidgets.QLineEdit()
        gravity_form.addWidget(QtWidgets.QLabel("扫描CSV"), 2, 0)
        gravity_form.addWidget(self.gravity_csv_edit, 2, 1, 1, 4)
        gravity_browse = QtWidgets.QPushButton("浏览…")
        gravity_browse.clicked.connect(lambda: self.choose_csv(self.gravity_csv_edit))
        gravity_form.addWidget(gravity_browse, 2, 5)
        gravity_analyze = QtWidgets.QPushButton("拟合重力模型")
        gravity_analyze.clicked.connect(self.analyze_pitch_gravity)
        gravity_form.addWidget(gravity_analyze, 2, 6)
        self.gravity_model_label = QtWidgets.QLabel("尚未分析")
        self.gravity_model_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        gravity_form.addWidget(self.gravity_model_label, 3, 0, 1, 7)
        layout.addWidget(gravity)

        files = QtWidgets.QGroupBox("训练与验证数据")
        grid = QtWidgets.QGridLayout(files)
        self.train_edit = QtWidgets.QLineEdit()
        self.valid_edit = QtWidgets.QLineEdit()
        for row, (name, edit) in enumerate((("训练CSV", self.train_edit), ("验证CSV", self.valid_edit))):
            grid.addWidget(QtWidgets.QLabel(name), row, 0)
            grid.addWidget(edit, row, 1)
            browse = QtWidgets.QPushButton("浏览…")
            browse.clicked.connect(lambda _, target=edit: self.choose_csv(target))
            grid.addWidget(browse, row, 2)
        analyze = QtWidgets.QPushButton("MATLAB自动建模")
        analyze.clicked.connect(self.analyze_sysid)
        grid.addWidget(analyze, 0, 3)
        launch = QtWidgets.QPushButton("打开System Identification")
        launch.clicked.connect(self.open_matlab_sysid)
        grid.addWidget(launch, 1, 3)
        layout.addWidget(files)

        model = QtWidgets.QGroupBox("当前辨识模型")
        model_layout = QtWidgets.QFormLayout(model)
        self.tf_label = QtWidgets.QLabel("尚未分析")
        self.tf_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        font = QtGui.QFont("Consolas"); font.setPointSize(11)
        self.tf_label.setFont(font)
        self.fit_label = QtWidgets.QLabel("—")
        self.model_param_label = QtWidgets.QLabel("—")
        model_layout.addRow("系统函数", self.tf_label)
        model_layout.addRow("验证拟合率", self.fit_label)
        model_layout.addRow("关键参数", self.model_param_label)
        layout.addWidget(model)
        layout.addStretch()
        return tab

    def make_tuning_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        profile = QtWidgets.QGroupBox("速度测试")
        row = QtWidgets.QHBoxLayout(profile)
        self.target_rpm = self.spin(8, 1, 30, 0)
        self.ramp_ms = self.spin(500, 50, 2000, 0)
        self.hold_ms = self.spin(600, 100, 5000, 0)
        self.repetitions = self.spin(2, 1, 20, 0)
        for label, widget in (("目标 rpm", self.target_rpm), ("斜坡 ms", self.ramp_ms),
                              ("平台 ms", self.hold_ms), ("重复", self.repetitions)):
            row.addWidget(QtWidgets.QLabel(label)); row.addWidget(widget)
        build = QtWidgets.QPushButton("构建调参固件")
        build.clicked.connect(lambda: self.build_firmware(False))
        row.addWidget(build)
        layout.addWidget(profile)

        parameters = QtWidgets.QGroupBox("运行时参数（无需重新编译）")
        grid = QtWidgets.QGridLayout(parameters)
        self.param_widgets: dict[str, QtWidgets.QDoubleSpinBox] = {}
        for index, (name, value) in enumerate(DEFAULT_PARAMS.items()):
            lo, hi = PARAM_BOUNDS[name]
            widget = self.spin(value, lo, hi, 3 if value < 10 else 1)
            self.param_widgets[name] = widget
            r, c = divmod(index, 4)
            grid.addWidget(QtWidgets.QLabel(PARAM_LABELS[name]), r * 2, c)
            grid.addWidget(widget, r * 2 + 1, c)
        layout.addWidget(parameters)

        ranges = QtWidgets.QGroupBox("多参数坐标下降范围")
        range_layout = QtWidgets.QVBoxLayout(ranges)
        self.range_table = QtWidgets.QTableWidget(len(DEFAULT_PARAMS), 5)
        self.range_table.setHorizontalHeaderLabels(["启用", "参数", "最小值", "最大值", "步长"])
        self.range_table.verticalHeader().setVisible(False)
        self.range_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.range_table.setMaximumHeight(285)
        self.range_widgets: dict[str, tuple[QtWidgets.QCheckBox, QtWidgets.QDoubleSpinBox,
                                            QtWidgets.QDoubleSpinBox, QtWidgets.QDoubleSpinBox]] = {}
        default_enabled = {"friction_blend_rpm", "friction_slew_per_s", "integral_leak_tau_s"}
        for row_index, name in enumerate(DEFAULT_PARAMS):
            enabled = QtWidgets.QCheckBox(); enabled.setChecked(name in default_enabled)
            holder = QtWidgets.QWidget(); holder_layout = QtWidgets.QHBoxLayout(holder)
            holder_layout.setContentsMargins(0, 0, 0, 0); holder_layout.addWidget(enabled, alignment=QtCore.Qt.AlignCenter)
            self.range_table.setCellWidget(row_index, 0, holder)
            self.range_table.setItem(row_index, 1, QtWidgets.QTableWidgetItem(PARAM_LABELS[name]))
            lo_bound, hi_bound = PARAM_BOUNDS[name]
            lo, hi, step = TUNING_RANGES[name]
            lo_widget = self.spin(lo, lo_bound, hi_bound, 3 if hi_bound <= 20 else 1)
            hi_widget = self.spin(hi, lo_bound, hi_bound, 3 if hi_bound <= 20 else 1)
            step_widget = self.spin(step, 0.001, hi_bound - lo_bound or 1, 3 if hi_bound <= 20 else 1)
            self.range_table.setCellWidget(row_index, 2, lo_widget)
            self.range_table.setCellWidget(row_index, 3, hi_widget)
            self.range_table.setCellWidget(row_index, 4, step_widget)
            self.range_widgets[name] = (enabled, lo_widget, hi_widget, step_widget)
        range_layout.addWidget(self.range_table)
        range_controls = QtWidgets.QHBoxLayout()
        self.tune_rounds = self.spin(1, 1, 5, 0)
        range_controls.addWidget(QtWidgets.QLabel("坐标下降轮数")); range_controls.addWidget(self.tune_rounds)
        self.estimate_label = QtWidgets.QLabel("预计实验次数：—")
        range_controls.addWidget(self.estimate_label); range_controls.addStretch()
        estimate = QtWidgets.QPushButton("计算实验次数")
        estimate.clicked.connect(self.estimate_tuning_runs)
        range_controls.addWidget(estimate)
        range_layout.addLayout(range_controls)
        layout.addWidget(ranges)

        actions = QtWidgets.QHBoxLayout()
        save_preset = QtWidgets.QPushButton("保存参数方案")
        save_preset.clicked.connect(self.save_parameter_preset)
        actions.addWidget(save_preset)
        load_preset = QtWidgets.QPushButton("加载参数方案")
        load_preset.clicked.connect(self.load_parameter_preset)
        actions.addWidget(load_preset)
        run = QtWidgets.QPushButton("运行当前参数并评分")
        run.clicked.connect(self.run_velocity_once)
        actions.addWidget(run)
        self.sweep_values = QtWidgets.QLineEdit("1.0,1.5,2.0,2.5")
        actions.addWidget(QtWidgets.QLabel("Blend扫描")); actions.addWidget(self.sweep_values)
        sweep = QtWidgets.QPushButton("一键扫描并评分")
        sweep.clicked.connect(self.run_blend_sweep)
        actions.addWidget(sweep)
        multi = QtWidgets.QPushButton("多参数自动整定")
        multi.clicked.connect(self.run_multi_parameter_tuning)
        actions.addWidget(multi)
        stop = QtWidgets.QPushButton("停止当前进程")
        stop.clicked.connect(self.stop_process)
        actions.addWidget(stop)
        layout.addLayout(actions)

        summary = QtWidgets.QGroupBox("当前评分")
        summary_layout = QtWidgets.QHBoxLayout(summary)
        self.score_label = QtWidgets.QLabel("—")
        score_font = QtGui.QFont(); score_font.setPointSize(24); score_font.setBold(True)
        self.score_label.setFont(score_font)
        self.metric_label = QtWidgets.QLabel("尚无测试结果")
        summary_layout.addWidget(self.score_label)
        summary_layout.addWidget(self.metric_label, 1)
        layout.addWidget(summary)

        self.tuning_table = QtWidgets.QTableWidget(0, 8)
        self.tuning_table.setHorizontalHeaderLabels(
            ["候选", "Score↓", "RMSE", "最大误差", "零速峰值", "电流RMS", "TV", "有效"])
        self.tuning_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.tuning_table, 1)
        return tab

    def make_data_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        row = QtWidgets.QHBoxLayout()
        refresh = QtWidgets.QPushButton("刷新索引")
        refresh.clicked.connect(self.refresh_data)
        row.addWidget(refresh); row.addStretch()
        layout.addLayout(row)
        self.data_table = QtWidgets.QTableWidget(0, 9)
        self.data_table.setHorizontalHeaderLabels(
            ["类型", "名称", "时间", "样本", "状态", "原因", "评分", "有效", "路径"])
        self.data_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.data_table.horizontalHeader().setStretchLastSection(True)
        self.data_table.doubleClicked.connect(self.open_selected_data_folder)
        layout.addWidget(self.data_table)
        return tab

    def session(self) -> Path:
        path = Path(self.session_edit.text()).resolve()
        path.mkdir(parents=True, exist_ok=True)
        self.settings.setValue("session", str(path)); self.settings.setValue("port", self.port_edit.text())
        return path

    def append_log(self, text: str) -> None:
        self.log.appendPlainText(text.rstrip())

    def choose_session(self) -> None:
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "选择实验目录", self.session_edit.text())
        if path: self.session_edit.setText(path)

    def choose_csv(self, edit: QtWidgets.QLineEdit) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择CSV", str(self.session()), "CSV (*.csv)")
        if path: edit.setText(path)

    def start_process(self, title: str, program: str, arguments: list[str], callback=None) -> None:
        if self.process and self.process.state() != QtCore.QProcess.NotRunning:
            QtWidgets.QMessageBox.warning(self, "进程忙", "当前已有实验或分析正在运行。")
            return
        self.append_log(f"\n[{title}] {program} {' '.join(arguments)}")
        self.statusBar().showMessage(title)
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(PROJECT_ROOT))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self.append_log(bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")))
        process.finished.connect(self.process_finished)
        self.process = process
        self.process_callback = callback
        process.start(program, arguments)

    def process_finished(self, exit_code: int, _status) -> None:
        self.append_log(f"进程结束，exit={exit_code}")
        callback = self.process_callback
        self.process_callback = None
        self.statusBar().showMessage("完成" if exit_code == 0 else "失败")
        if callback: callback(exit_code)
        self.refresh_data()

    def stop_process(self) -> None:
        if self.process and self.process.state() != QtCore.QProcess.NotRunning:
            self.process.terminate()
            QtCore.QTimer.singleShot(2000, self.process.kill)

    def build_firmware(self, sysid: bool) -> None:
        args = ["-S", str(PROJECT_ROOT), "-B", str(BUILD_DIR),
                f"-DSYSTEM_IDENTIFICATION_MODE={'ON' if sysid else 'OFF'}",
                f"-DVELOCITY_CONTROL_TEST_MODE={'OFF' if sysid else 'ON'}"]
        command = f"& '{CMAKE}' {' '.join(args)}; if ($LASTEXITCODE -eq 0) {{ & '{CMAKE}' --build '{BUILD_DIR}' --target Project1 -j 14 }}"
        self.start_process("构建固件", "powershell", ["-NoProfile", "-Command", command])

    def capture_sysid(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        kind = self.sysid_type.currentText()
        axis = self.sysid_axis.currentText()
        output = self.session() / "raw" / "sysid_gui" / f"{axis}_{kind}_{stamp}"
        args = [str(TOOLS_ROOT / "capture_sysid.py"), "--port", self.port_edit.text(),
                "--output", str(output), "--axis", axis, "--type", kind,
                "--amplitude", str(int(self.sysid_amp.value())),
                "--duration-ms", str(int(self.sysid_duration.value()))]
        if kind == "prbs":
            args += ["--switch-ms", str(int(self.sysid_switch.value())), "--seed", self.sysid_seed.text()]
        if axis == "pitch" and kind == "prbs" and self.sysid_auto_position.isChecked():
            args += ["--operating-angle-deg", str(self.sysid_operating_angle.value()),
                     "--move-ms", "2500", "--settle-ms", "1000"]
        self.start_process("系统辨识采集", sys.executable, args)

    def capture_pitch_gravity(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = self.session() / "raw" / "pitch" / "gravity" / f"pitch_gravity_{stamp}"
        args = [str(TOOLS_ROOT / "capture_pitch_gravity.py"),
                "--port", self.port_edit.text(), "--output", str(output),
                "--min-deg", str(self.gravity_min.value()),
                "--max-deg", str(self.gravity_max.value()),
                "--step-deg", str(self.gravity_step.value()),
                "--ramp-ms", str(int(self.gravity_ramp.value())),
                "--settle-ms", str(int(self.gravity_settle.value())),
                "--repetitions", str(int(self.gravity_repetitions.value()))]
        csv_path = output.with_suffix(".csv")
        self.start_process(
            "Pitch重力矩扫描", sys.executable, args,
            lambda code: self._gravity_capture_finished(code, csv_path))

    def _gravity_capture_finished(self, exit_code: int, csv_path: Path) -> None:
        if exit_code == 0:
            self.gravity_csv_edit.setText(str(csv_path))
            self.analyze_pitch_gravity()

    def analyze_pitch_gravity(self) -> None:
        source = Path(self.gravity_csv_edit.text())
        if not source.exists():
            QtWidgets.QMessageBox.warning(self, "缺少数据", "请选择重力扫描CSV。")
            return
        prefix = self.session() / "models" / "pitch_gravity_latest"
        args = [str(TOOLS_ROOT / "analyze_pitch_gravity.py"),
                "--input", str(source), "--output-prefix", str(prefix)]
        summary = prefix.with_name(prefix.name + "_summary.json")
        self.start_process(
            "Pitch重力模型拟合", sys.executable, args,
            lambda code: self.load_gravity_summary(summary) if code == 0 else None)

    def load_gravity_summary(self, path: Path) -> None:
        summary = json.loads(path.read_text(encoding="utf-8"))
        if "recommended_gravity_sin_coefficient" in summary:
            self.gravity_model_label.setText(
                f"推荐：u_ff={summary['recommended_gravity_sin_coefficient']:.2f}·sin(θ) "
                f"{summary['recommended_bias_current']:+.2f} | "
                f"摩擦估计={summary['recommended_friction_current']:.2f}, "
                f"配对RMSE={summary['recommended_fit_rmse_current']:.2f}")
        else:
            self.gravity_model_label.setText(
                f"u={summary['sin_coefficient']:.2f}·sin(θ) + "
                f"{summary['cos_coefficient']:.2f}·cos(θ) + "
                f"{summary['bias_current']:.2f} + "
                f"{summary['coulomb_friction_current']:.2f}·direction | "
                f"RMSE={summary['rmse_current']:.2f}, R²={summary['r_squared']:.4f}")

    def analyze_sysid(self) -> None:
        train, valid = Path(self.train_edit.text()), Path(self.valid_edit.text())
        if not train.exists() or not valid.exists():
            QtWidgets.QMessageBox.warning(self, "缺少数据", "请选择训练和验证CSV。")
            return
        output = self.session() / "models" / "gui_latest"
        output.mkdir(parents=True, exist_ok=True)
        axis = self.sysid_axis_for_file(train)
        cmd = (f"addpath('{TOOLS_ROOT.as_posix()}');"
               f"run_sysid_export('{train.as_posix()}','{valid.as_posix()}','{output.as_posix()}','{axis}');")
        self.start_process("MATLAB系统辨识", str(MATLAB), ["-batch", cmd],
                           lambda code: self.load_model_summary(output) if code == 0 else None)

    def load_model_summary(self, output: Path) -> None:
        summary = json.loads((output / "latest_sysid_summary.json").read_text(encoding="utf-8"))
        self.tf_label.setText(summary["transfer_function"])
        self.fit_label.setText(
            f"一阶 {summary['fit_first_order_percent']:.2f}% | 二阶 {summary['fit_second_order_percent']:.2f}% | 状态空间 {summary['fit_state_space_2_percent']:.2f}%")
        self.model_param_label.setText(
            f"axis={summary.get('axis', 'yaw')}, output={summary.get('output_signal', 'speed_rpm')}, "
            f"K={summary.get('first_order_gain', summary['first_order_gain_rpm_per_command']):.6g}, "
            f"τ={summary['first_order_time_constant_s']*1000:.3f} ms, "
            f"Td={summary['first_order_delay_s']*1000:.3f} ms")

    def open_matlab_sysid(self) -> None:
        train, valid = Path(self.train_edit.text()), Path(self.valid_edit.text())
        if not train.exists() or not valid.exists():
            QtWidgets.QMessageBox.warning(self, "缺少数据", "请选择训练和验证CSV。")
            return
        axis = self.sysid_axis_for_file(train)
        command = (f"addpath('{TOOLS_ROOT.as_posix()}');"
                   f"open_sysid_app('{train.as_posix()}','{valid.as_posix()}','{axis}');")
        subprocess.Popen([str(MATLAB), "-r", command], cwd=str(PROJECT_ROOT))
        self.append_log("已启动MATLAB；工作区变量为 sysid_training_data 和 sysid_validation_data。")

    def sysid_axis_for_file(self, csv_path: Path) -> str:
        metadata_path = csv_path.with_suffix(".json")
        if metadata_path.exists():
            try:
                axis = json.loads(metadata_path.read_text(encoding="utf-8")).get("axis")
                if axis in ("yaw", "pitch"): return axis
            except (OSError, json.JSONDecodeError):
                pass
        return self.sysid_axis.currentText()

    def current_params(self) -> dict[str, float]:
        return {name: widget.value() for name, widget in self.param_widgets.items()}

    def save_parameter_preset(self) -> None:
        preset_dir = self.session() / "presets"
        preset_dir.mkdir(parents=True, exist_ok=True)
        default_name = preset_dir / f"controller_{datetime.now():%Y%m%d_%H%M%S}.json"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存参数方案", str(default_name), "JSON (*.json)")
        if not path: return
        payload = {
            "schema": "gimbal-controller-params-v1",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "params": self.current_params(),
        }
        Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        self.append_log(f"已保存参数方案：{path}")

    def load_parameter_preset(self) -> None:
        preset_dir = self.session() / "presets"
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载参数方案", str(preset_dir), "JSON (*.json)")
        if not path: return
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            params = payload.get("params", payload)
            missing = [name for name in DEFAULT_PARAMS if name not in params]
            if missing: raise ValueError("缺少参数：" + ", ".join(missing))
            for name in DEFAULT_PARAMS:
                value = float(params[name])
                low, high = PARAM_BOUNDS[name]
                if not low <= value <= high:
                    raise ValueError(f"{name}={value}超出[{low}, {high}]")
                self.param_widgets[name].setValue(value)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            QtWidgets.QMessageBox.warning(self, "参数方案无效", str(exc)); return
        self.append_log(f"已加载参数方案：{path}")

    def run_velocity(self, params: dict[str, float], output: Path, callback=None) -> None:
        param_dir = self.session() / "notes" / "tuning_params"
        param_dir.mkdir(parents=True, exist_ok=True)
        param_path = param_dir / f"{output.name}.json"
        param_path.write_text(json.dumps(params, indent=2), encoding="utf-8")
        args = [str(TOOLS_ROOT / "capture_velocity_test.py"), "--port", self.port_edit.text(),
                "--output", str(output), "--mode", "full-ff",
                "--target-rpm", str(int(self.target_rpm.value())),
                "--ramp-ms", str(int(self.ramp_ms.value())),
                "--hold-ms", str(int(self.hold_ms.value())),
                "--repetitions", str(int(self.repetitions.value())),
                "--params-json", str(param_path)]
        self.last_velocity_csv = output.with_suffix(".csv")
        self.start_process("速度控制实验", sys.executable, args,
                           callback or (lambda code: self.score_velocity(self.last_velocity_csv) if code == 0 else None))

    def run_velocity_once(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = self.session() / "raw" / "tuning" / f"velocity_candidate_{stamp}"
        self.run_velocity(self.current_params(), output)

    def score_velocity(self, csv_path: Path, candidate: str = "current") -> dict | None:
        try:
            metrics = velocity_metrics(csv_path)
        except Exception as exc:
            self.append_log(f"评分失败: {exc}")
            return None
        self.score_label.setText(f"{metrics['score']:.2f}")
        self.metric_label.setText(
            f"RMSE {metrics['rmse_rpm']:.3f} rpm | 最大误差 {metrics['max_error_rpm']:.3f} rpm | "
            f"零速峰值 {metrics['zero_peak_rpm']:.2f} rpm | 电流RMS {metrics['current_rms']:.1f} | "
            f"Pitch范围 {metrics['pitch_range_deg']:.3f}°")
        row = self.tuning_table.rowCount(); self.tuning_table.insertRow(row)
        values = [candidate, f"{metrics['score']:.3f}", f"{metrics['rmse_rpm']:.3f}",
                  f"{metrics['max_error_rpm']:.3f}", f"{metrics['zero_peak_rpm']:.2f}",
                  f"{metrics['current_rms']:.1f}", f"{metrics['control_tv']:.0f}", str(metrics['valid'])]
        for column, value in enumerate(values): self.tuning_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.append_log(f"评分 {candidate}: {json.dumps(metrics, ensure_ascii=False)}")
        return metrics

    @staticmethod
    def range_values(minimum: float, maximum: float, step: float) -> list[float]:
        if maximum < minimum or step <= 0:
            raise ValueError("范围必须满足 max >= min 且 step > 0")
        count = int((maximum - minimum) / step + 1e-9) + 1
        if count > 100:
            raise ValueError("单个参数候选数不能超过100")
        values = [minimum + index * step for index in range(count)]
        if values[-1] < maximum - step * 1e-6:
            values.append(maximum)
        return values

    def selected_tuning_dimensions(self) -> list[tuple[str, list[float]]]:
        dimensions = []
        for name, (enabled, lo, hi, step) in self.range_widgets.items():
            if enabled.isChecked():
                dimensions.append((name, self.range_values(lo.value(), hi.value(), step.value())))
        return dimensions

    def estimate_tuning_runs(self) -> None:
        try:
            dimensions = self.selected_tuning_dimensions()
            count = sum(len(values) for _, values in dimensions) * int(self.tune_rounds.value())
            duration = 4 * (self.ramp_ms.value() + self.hold_ms.value()) * self.repetitions.value() / 1000
            self.estimate_label.setText(
                f"预计实验次数：{count}，纯采集约 {count * duration / 60:.1f} min")
        except ValueError as exc:
            self.estimate_label.setText(str(exc))

    def run_multi_parameter_tuning(self) -> None:
        try:
            dimensions = self.selected_tuning_dimensions()
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "范围错误", str(exc)); return
        if not dimensions:
            QtWidgets.QMessageBox.warning(self, "未选择参数", "至少勾选一个待整定参数。")
            return
        count = sum(len(values) for _, values in dimensions) * int(self.tune_rounds.value())
        if count > 80:
            answer = QtWidgets.QMessageBox.question(
                self, "实验次数较多", f"坐标下降预计需要{count}次实验，是否继续？")
            if answer != QtWidgets.QMessageBox.Yes: return
        self.tune_dimensions = dimensions
        self.tune_dimension_index = 0
        self.tune_round = 0
        self.tune_base_params = self.current_params()
        self.sweep_results.clear()
        self.dimension_results.clear()
        self.tune_reference_pitch = None
        self.append_log(f"开始多参数坐标下降：{[name for name, _ in dimensions]}，共{int(self.tune_rounds.value())}轮。")
        self.begin_tuning_dimension()

    def begin_tuning_dimension(self) -> None:
        if self.tune_round >= int(self.tune_rounds.value()):
            self.finish_multi_tuning()
            return
        if self.tune_dimension_index >= len(self.tune_dimensions):
            self.tune_round += 1
            self.tune_dimension_index = 0
            self.begin_tuning_dimension()
            return
        name, values = self.tune_dimensions[self.tune_dimension_index]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dimension_results.clear()
        self.tuning_queue.clear()
        for value in values:
            params = dict(self.tune_base_params); params[name] = value
            label = str(round(value, 6)).replace(".", "p")
            output = (self.session() / "raw" / "tuning" /
                      f"cd_r{self.tune_round + 1}_{name}_{label}_{stamp}")
            self.tuning_queue.append((params, output))
        self.append_log(f"第{self.tune_round + 1}轮：扫描 {name}，候选={values}")
        self.run_next_multi_candidate()

    def run_next_multi_candidate(self) -> None:
        if not self.tuning_queue:
            valid = [item for item in self.dimension_results if item[3].get("valid")]
            if not valid:
                self.append_log("当前维度没有有效候选，自动整定停止。")
                self.tune_dimensions = []
                return
            best = min(valid, key=lambda item: item[0])
            self.tune_base_params = dict(best[1])
            name, _ = self.tune_dimensions[self.tune_dimension_index]
            self.append_log(f"{name}本轮最优={self.tune_base_params[name]:.6g}，Score={best[0]:.3f}")
            self.tune_dimension_index += 1
            QtCore.QTimer.singleShot(1000, self.begin_tuning_dimension)
            return

        params, output = self.tuning_queue.pop(0)
        name, _ = self.tune_dimensions[self.tune_dimension_index]
        label = f"R{self.tune_round + 1} {name}={params[name]:.6g}"

        def finished(code: int) -> None:
            if code != 0:
                self.tuning_queue.clear(); self.tune_dimensions = []
                self.append_log("采集进程失败，自动整定停止。")
                return
            metrics = self.score_velocity(output.with_suffix(".csv"), label)
            if metrics is None or not metrics.get("valid"):
                self.tuning_queue.clear(); self.tune_dimensions = []
                self.append_log("候选触发安全/数据完整性门限，自动整定停止。")
                return
            pitch = float(metrics["initial_pitch_deg"])
            if self.tune_reference_pitch is None:
                self.tune_reference_pitch = pitch
            if abs(pitch - self.tune_reference_pitch) > 0.5:
                self.tuning_queue.clear(); self.tune_dimensions = []
                self.append_log(
                    f"Pitch初始角偏差{pitch-self.tune_reference_pitch:+.3f}°超过±0.5°，请复位机械后重试。")
                return
            if abs(float(metrics["yaw_net_deg"])) > 10.0:
                self.tuning_queue.clear(); self.tune_dimensions = []
                self.append_log("Yaw单次净漂移超过10°，为防止多轮累计行程，自动整定停止。")
                return
            item = (float(metrics["score"]), params, output.with_suffix(".csv"), metrics)
            self.dimension_results.append(item); self.sweep_results.append(item)
            QtCore.QTimer.singleShot(1000, self.run_next_multi_candidate)

        self.run_velocity(params, output, finished)

    def finish_multi_tuning(self) -> None:
        for name, value in self.tune_base_params.items():
            self.param_widgets[name].setValue(value)
        result_dir = self.session() / "results"; result_dir.mkdir(parents=True, exist_ok=True)
        history = [{"score": score, "csv": str(path), "params": params,
                    "metrics": metrics}
                   for score, params, path, metrics in self.sweep_results]
        result = {"algorithm": "bounded_coordinate_descent",
                  "rounds": int(self.tune_rounds.value()),
                  "best_params": self.tune_base_params, "history": history}
        (result_dir / "tuning_best_params.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        self.append_log("多参数自动整定完成，最优参数已回填并保存。请使用独立轨迹进行最终验证。")
        self.tune_dimensions = []

    def run_blend_sweep(self) -> None:
        try:
            values = [float(item.strip()) for item in self.sweep_values.text().split(",") if item.strip()]
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "参数错误", "Blend扫描列表格式错误。")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.tuning_queue.clear()
        self.sweep_results.clear()
        for value in values:
            params = self.current_params(); params["friction_blend_rpm"] = value
            label = str(value).replace(".", "p")
            output = self.session() / "raw" / "tuning" / f"blend_{label}_{stamp}"
            self.tuning_queue.append((params, output))
        self.run_next_candidate()

    def run_next_candidate(self) -> None:
        if not self.tuning_queue:
            valid = [item for item in self.sweep_results if item[3].get("valid")]
            if valid:
                score, params, csv_path, _ = min(valid, key=lambda item: item[0])
                for name, value in params.items():
                    self.param_widgets[name].setValue(value)
                result_dir = self.session() / "results"
                result_dir.mkdir(parents=True, exist_ok=True)
                best = {"score": score, "csv": str(csv_path), "params": params}
                (result_dir / "tuning_best_params.json").write_text(
                    json.dumps(best, indent=2), encoding="utf-8")
                self.append_log(f"参数扫描完成；最优Score={score:.3f}，已回填界面并保存。")
            else:
                self.append_log("参数扫描完成，但没有通过安全与数据完整性检查的候选。")
            return
        params, output = self.tuning_queue.pop(0)
        label = f"blend={params['friction_blend_rpm']:.3g}"
        def finished(code: int) -> None:
            if code == 0:
                metrics = self.score_velocity(output.with_suffix(".csv"), label)
                if metrics is not None:
                    self.sweep_results.append((float(metrics["score"]), params,
                                               output.with_suffix(".csv"), metrics))
                QtCore.QTimer.singleShot(1000, self.run_next_candidate)
            else:
                self.tuning_queue.clear()
                self.append_log("扫描因实验失败停止。")
        self.run_velocity(params, output, finished)

    def refresh_data(self) -> None:
        try:
            self.index.sync(); rows = self.index.rows()
        except Exception as exc:
            self.append_log(f"索引刷新失败: {exc}") if hasattr(self, "log") else None
            return
        self.data_table.setRowCount(0)
        for values in rows:
            row = self.data_table.rowCount(); self.data_table.insertRow(row)
            for column, value in enumerate(values):
                self.data_table.setItem(row, column, QtWidgets.QTableWidgetItem("" if value is None else str(value)))

    def open_selected_data_folder(self) -> None:
        row = self.data_table.currentRow()
        if row < 0: return
        path = Path(self.data_table.item(row, 8).text()).parent
        os.startfile(path)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = ControlLab()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
