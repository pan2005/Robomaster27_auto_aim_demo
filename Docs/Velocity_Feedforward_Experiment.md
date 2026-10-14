# Yaw速度前馈对比实验

本实验比较三种控制结构，三者使用完全相同的速度PID参数和目标轨迹：

1. 纯速度PID；
2. 速度前馈 `Kv * target_rpm` 加PID；
3. 摩擦、速度、加速度前馈加PID。

核心前馈代码位于`Tasks/velocity_control_test_task.c`的`VTestFeedforward()`。每个物理分量独立计算并独立记录：

```c
velocity_ff = Kv * target_rpm;
friction_ff = Uf * tanh(target_rpm / blend_rpm);
acceleration_ff = Ka * target_accel_rpm_s;
output = clamp(pid_output + friction_ff + velocity_ff + acceleration_ff);
```

`tanh`用于让摩擦补偿在零速附近平滑过渡，避免直接使用`sign()`造成正负516电流的跳变。当前辨识初值为：

- `Uf = 516`；
- `Kv = 65.8 command/rpm`；
- `Ka = 0.184 command/(rpm/s)`。

固定PID参数为`Kp=80、Ki=300、Kd=0`。三种模式均采用同一组参数，才能公平判断性能改善是否真正来自前馈。

## 构建与烧录

CLion构建目录已配置为：

```text
SYSTEM_IDENTIFICATION_MODE=OFF
VELOCITY_CONTROL_TEST_MODE=ON
```

烧录`cmake-build-debug-stm32/Project1.elf`。

## 三组实验

将`COM7`替换成实际端口，依次运行：

```powershell
python Tools\capture_velocity_test.py --port COM7 --output velocity_pid_long --mode pid --target-rpm 15 --ramp-ms 400 --hold-ms 1500 --repetitions 1
python Tools\capture_velocity_test.py --port COM7 --output velocity_vff_long --mode velocity-ff --target-rpm 15 --ramp-ms 400 --hold-ms 1500 --repetitions 1
python Tools\capture_velocity_test.py --port COM7 --output velocity_fullff_long --mode full-ff --target-rpm 15 --ramp-ms 400 --hold-ms 1500 --repetitions 1
```

一轮实验持续7.6秒，轨迹依次为：斜坡到+15 rpm、保持、回零、保持、斜坡到-15 rpm、保持、回零、保持。单方向目标理论位移约153°，在±180°安全限位内保留约27°制动和模型误差余量。

## MATLAB比较

```matlab
addpath("Tools")
metrics = compare_velocity_control("velocity_pid.csv", ...
    "velocity_vff.csv", "velocity_fullff.csv");
```

脚本输出速度跟踪曲线、误差、电流曲线，以及RMSE、IAE、最大误差、电流RMS、峰值电流、控制量总变差和饱和比例。

实验安全限制为电流±2000、速度±60 rpm、相对启动角±180°。如果任一组触发保护，不应直接比较综合指标，应先检查机械起始位置和终止原因。
