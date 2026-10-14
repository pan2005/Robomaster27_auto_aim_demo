# Robomaster27 Auto Aim Demo

面向 RoboMaster 云台的实验性高动态 AprilTag 自瞄项目。项目包含 STM32H723 云台固件和 Windows 图传终端，使用海康机器人 `MV-CS016-10UC` 相机、MVS SDK、USB CDC 和 GM6020 电机，支持鼠标手动操控、视觉测量、目标预测与约束轨迹规划。

> 当前版本只计算、显示并记录 `fire` 判定，不会驱动发射 GPIO。请先在无弹、断开发射机构的条件下完成全部测试。

## 已实现功能

- 海康 MVS SDK 实时取流，默认曝光 `8000 us`、增益 `16.08 dB`，画面顺时针旋转 90°，只消费最新帧以限制延迟。
- Windows Raw Input 鼠标相对位移控制；失焦、Esc、串口断开和 200 ms 心跳超时安全失能。
- 带同步字、版本、序号、长度和 CRC16 的 USB CDC 二进制协议，以及状态、姿态和时钟同步回传。
- 两轴机械/软件角度限制均为 ±90°；手动模式保留固件轨迹生成、PID 和前馈控制。
- IMU 预热后采集 2000 个静止样本估计陀螺零偏，并在姿态报文中提供诊断数据。
- 圆点板相机内参及相机—云台 Robot-World/Hand-Eye 外参标定，包含数据质量检查和离群样本诊断。
- AprilTag 36h11 检测、亚像素角点、IPPE Square PnP、预测 ROI 和观测门控。
- 世界系 9 状态位置/速度/加速度滤波，延迟与重力补偿，以及 10 ms 步长、1 s 预测窗的两轴 MPC 轨迹规划。
- `MANUAL → AUTO_ACQUIRE → AUTO_TRACK → AUTO_COAST` 人机协作状态机、可视化叠加和 CSV 诊断日志。

## 目录结构

```text
Core/ Drivers/ Middlewares/ USB_DEVICE/  STM32CubeH7 生成及基础驱动
BSP/ Modules/ Tasks/ Algorithm/           云台、IMU、控制、协议与算法代码
Tools/gimbal_terminal/                    PyQt5 海康图传及自瞄终端
Tools/gimbal_terminal/tests/              PC 协议、视觉、预测和规划测试
Docs/                                     通信协议和工程说明
Experiments/                              系统辨识/速度实验脚本（不含原始大数据）
```

## 硬件与软件

- STM32H723 控制板，通过 ST USB CDC 虚拟串口连接 Windows 主机。
- 海康机器人 `MV-CS016-10UC`，通过独立 USB3 接口连接主机。
- Pitch/Yaw GM6020 云台；当前固件按两轴 ±90° 限位配置。
- Windows 10/11、海康机器人 MVS（含 USB3 Vision 驱动及开发组件）。
- ARM GNU Toolchain、CMake 和 Ninja，用于固件构建。
- Python 3。首次启动脚本会建立本地虚拟环境并安装终端依赖。

首先用 MVS 客户端确认相机能稳定取流；设备管理器错误码 28 表示驱动尚未正确安装。本项目不包含也不会静默安装海康驱动。

## 快速开始

### 1. 构建并烧录固件

在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成的 ELF 位于 `build/Debug/Project1.elf`。使用现有调试器/烧录工具下载后，确认 STM32 枚举为 CDC 串口。

### 2. 启动终端

```powershell
Tools\gimbal_terminal\run_terminal.bat
```

脚本会检查 MVS Python SDK、创建 `.venv` 并安装 `PyQt5`、`pyserial`、`numpy`、`opencv-python` 和 `PyYAML`。相机和串口均可在界面中选择。

### 3. 操作

- 左键点击画面：捕获鼠标并进入手动云台控制。
- 移动鼠标：改变 Yaw/Pitch 目标；停止移动后保持目标角度。
- 右键：锁定准星附近最近的合格 AprilTag；再次右键退出自动跟踪。
- 自动跟踪时按住左键：仅授予 `fire` 判定权限，当前版本不执行真实发射。
- `F8`：云台完全静止后采集一组外参样本。
- Esc、窗口失焦或急停：释放鼠标并立即失能。

完整的相机、内外参标定与 AprilTag 操作说明见 [终端文档](Tools/gimbal_terminal/README.md)，线协议见 [USB 协议](Docs/USB_Protocol.md)。仓库附带的标定 YAML 只对应开发样机；改变镜头、对焦、分辨率、相机安装或 IMU/云台结构后必须重新标定。

## 测试

PC 端：

```powershell
Tools\gimbal_terminal\.venv\Scripts\python.exe -m unittest discover -s Tools/gimbal_terminal/tests -v
```

固件除默认模式外还支持系统辨识和速度测试：

```powershell
cmake -S . -B build/SysId -G Ninja "-DCMAKE_BUILD_TYPE=Debug" "-DSYSTEM_IDENTIFICATION_MODE=ON" "-DVELOCITY_CONTROL_TEST_MODE=OFF" "-DCMAKE_TOOLCHAIN_FILE=$((Resolve-Path cmake/gcc-arm-none-eabi.cmake).Path)"
cmake --build build/SysId

cmake -S . -B build/Velocity -G Ninja "-DCMAKE_BUILD_TYPE=Debug" "-DSYSTEM_IDENTIFICATION_MODE=OFF" "-DVELOCITY_CONTROL_TEST_MODE=ON" "-DCMAKE_TOOLCHAIN_FILE=$((Resolve-Path cmake/gcc-arm-none-eabi.cmake).Path)"
cmake --build build/Velocity
```

辨识/速度测试模式不用于运行图传终端协议。

## 当前边界与已知问题

- 当前目标是 AprilTag 36h11，不包含装甲板神经网络识别。
- 自动模式的固定弹速默认值为 15 m/s，暂不建模空气阻力；实弹前必须测量弹速、发射延迟和枪口外参。
- `fire` 只进入 UI、日志和协议状态，STM32 发射 GPIO 保持禁用。
- 当前协议在自动跟踪出现瞬时无效视觉/姿态目标时会触发失能并重置控制器；复杂场景可能表现为云台轻微顿挫。后续应加入短时 HOLD 状态，使无效观测只撤销 `fire`，并保持安全轨迹参考。
- 项目是研究/联调用演示，不是经过功能安全认证的武器控制系统。任何发射机构接入前都应增加独立硬件急停、低有效安全回路和现场风险评估。

## 致谢与第三方代码

视觉、预测和轨迹规划路线参考了 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)。STM32 HAL/CMSIS、海康 MVS SDK 及 Python 依赖分别遵循各自上游许可；本仓库不再分发 MVS SDK 二进制文件。
