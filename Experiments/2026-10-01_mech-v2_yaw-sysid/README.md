# Mechanical Version 2 — Yaw System Identification

## 机械与传感器变更

- 日期：2026-10-01；
- 相机、上部结构和负载重新设计；
- IMU安装位置发生变化；
- 旧结构辨识模型和前馈参数全部视为失效，仅保留作对照；
- 机械照片：`notes/mechanical_design.jpg`。

实验前补充：

- [ ] 记录IMU PCB箭头或XYZ丝印相对枪管的方向；
- [ ] 记录相机、仿真枪/波箱和其他配重是否全部安装；
- [ ] 记录线缆布置，确保运动时不会拉扯；
- [ ] 确认Yaw相对实验起点的两侧至少有90°安全空间；
- [ ] 上电后静止至少100 ms完成IMU零偏采样。

## 本轮固件

- 应用模式：`SYSTEM_IDENTIFICATION_MODE=ON`；
- 二进制协议：v4，支持Yaw/Pitch双轴辨识；
- 采样率：1 kHz；
- Yaw电机：CAN1，反馈ID `0x206`；
- Yaw硬限制：电流±2000、速度±60 rpm、相对启动角±90°；
- Pitch在实验开始时锁定当前`imu.pitch`，保持电流限制±5000；
- Pitch速度超过60 rpm，或IMU/编码器相对起点偏离超过5°时立即中止；
- v4记录IMU yaw/pitch/roll、gyro X/Y/Z及支撑轴保持电流。日志中的电机编码器、累计角度、RPM和反馈电流统一指当前被辨识轴。
- Pitch辨识时Yaw保持起始姿态；Pitch相对起点限制±30°，注入电流叠加在Pitch闭环承重电流上，`output_current`为电机实际总指令。

### Pitch首次低幅值检查

将Pitch放在接近水平且两侧有足够空间的位置，先执行：

```powershell
python Tools\capture_sysid.py --port COM7 --axis pitch --output Experiments\2026-10-01_mech-v2-yaw-sysid\raw\pitch\axis-check\pitch_step_amp0200_run01 --type step --amplitude 200 --duration-ms 4000
```

验收`gyro_y_rad_s`是主要响应轴、Yaw姿态保持稳定、Pitch没有下坠或持续振荡。该组只用于方向和安全验证，不进入正式模型。

### Pitch重力矩扫描

将Pitch放在接近IMU 0°的位置，确认物理行程覆盖-30°到+30°，然后运行：

```powershell
python Tools\capture_pitch_gravity.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\gravity\pitch_gravity_m30_p30_step15_run01 --min-deg -30 --max-deg 30 --step-deg 15 --ramp-ms 1500 --settle-ms 1500 --repetitions 1
```

首轮完成后检查没有振荡、撞限位、线缆牵拉和保护中止。拟合命令：

```powershell
python Tools\analyze_pitch_gravity.py --input Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\gravity\pitch_gravity_m30_p30_step15_run01.csv --output-prefix Experiments\2026-10-01_mech-v2_yaw-sysid\models\pitch_gravity_run01
```

若首轮有效，可用不同文件名和`--repetitions 2`复测；默认参数每轮30秒，两轮总时长60秒。每轮最后会返回实验启动角再关闭电流。不要覆盖run01。

本轮重力扫描的推荐局部前馈为：

```text
u_ff = -2298.8448 * sin(imu_pitch_rad) - 99.3683
```

这个推荐式来自-15°、0°、+15°处正反方向维持电流的配对平均，适用范围暂定为实测的±30°。方向摩擦估计约670.9，但首轮控制对比不加入摩擦前馈，以保证只比较重力补偿这一项。采集PID和PID+重力前馈两组的命令见`Tools/SYSID_README.md`。

### Pitch PRBS动态辨识

Pitch PRBS使用重力补偿和位置反馈维持工作点，再叠加零均值PRBS。首轮在IMU Pitch接近0°处采集幅值1200、切换周期30 ms、时长15秒的训练集和验证集，随机种子必须不同。文件放入`raw/pitch/prbs/`；命令见`Tools/SYSID_README.md`。动态模型使用实际总电机指令`output_current`作为输入、`gyro_y_rad_s`作为输出，并明确标记为闭环局部辨识结果。

非零工作点必须使用`--operating-angle-deg`自动预定位，禁止用手估计角度。固件在PRBS前自动匀速移动并停稳，采集结束后返回实验启动角；移动、停稳和返回阶段不进入CSV采样。首轮0°数据保留为局部预实验，简单模型最高验证拟合约52%，暂不用于生成动态前馈。

## 实验顺序

### 1. 坐标系与低电流检查

先将Yaw放在行程中间，执行：

```powershell
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\axis-check\yaw_step_amp0300_run01 --type step --amplitude 300 --duration-ms 4000
```

如果运动太小，可以依次使用500和800，但不得跳过低幅值检查。检查电机角度、IMU三轴姿态和三轴角速度的符号及相关性，确认哪个IMU轴对应物理Yaw；同时确认Pitch闭环没有沿错误方向运动。

### 2. 阶跃测试

每组前将Yaw放回安全中位，不覆盖旧文件：

```powershell
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\step\yaw_step_amp0600_run01 --type step --amplitude 600 --duration-ms 4000
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\step\yaw_step_amp1000_run01 --type step --amplitude 1000 --duration-ms 4000
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\step\yaw_step_amp1400_run01 --type step --amplitude 1400 --duration-ms 4000
```

若任一组触发保护或出现机械撞击趋势，不再提高幅值。

### 3. PRBS训练与验证

先从1000幅值开始；确认安全和运动充分后再进行1400幅值：

```powershell
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\prbs\yaw_prbs_amp1000_sw20ms_train_run01 --type prbs --amplitude 1000 --switch-ms 20 --duration-ms 10000 --seed 0x13572468
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\prbs\yaw_prbs_amp1000_sw20ms_valid_run01 --type prbs --amplitude 1000 --switch-ms 20 --duration-ms 10000 --seed 0x24681357
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\prbs\yaw_prbs_amp1400_sw40ms_train_run01 --type prbs --amplitude 1400 --switch-ms 40 --duration-ms 10000 --seed 0x31415926
python Tools\capture_sysid.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\prbs\yaw_prbs_amp1400_sw40ms_valid_run01 --type prbs --amplitude 1400 --switch-ms 40 --duration-ms 10000 --seed 0x27182818
```

## 数据验收

每个JSON必须满足：

- `sample_count`等于预期采样数；
- `host_detected_missing_samples = 0`；
- `crc_errors = 0`；
- 最终状态为`COMPLETE`且reason为0；
- CSV中`safety_flags`全部为0；
- 电机RPM和至少一个IMU角速度轴与运动明显相关。

不满足条件的数据保留，但在文件名后增加`_invalid`或在`notes`中记录原因，禁止静默删除或混入训练集。

## MATLAB建模与结果保存

采集完成后，在MATLAB中从项目根目录运行（按实际文件名修改）：

```matlab
session = fullfile("Experiments", "2026-10-01_mech-v2_yaw-sysid");
trainCsv = fullfile(session, "raw", "prbs", "yaw_prbs_amp1000_sw20ms_train_run01.csv");
validCsv = fullfile(session, "raw", "prbs", "yaw_prbs_amp1000_sw20ms_valid_run01.csv");

result = analyze_sysid(trainCsv, validCsv);
save(fullfile(session, "models", "yaw_model_amp1000_sw20ms.mat"), "result");
```

导出的图片放入`plots/`，人工结论、异常现象和最终采用的模型参数记录在`notes/`。旧机械版本的数据只能用于前后对比，不与本次数据混合训练。

## 速度控制对比实验

速度测试使用独立固件模式`VELOCITY_CONTROL_TEST_MODE=ON`和USB协议v3。v3增加运行时控制参数设置与回读，确保每次实验的实际参数写入JSON元数据。Pitch继续闭环保持实验开始姿态，Yaw相对启动位置限制±90°。首轮统一使用目标速度8 rpm、500 ms斜坡、600 ms平台和2次重复：

```powershell
python Tools\capture_velocity_test.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\velocity\yaw_vel_08rpm_pid_run01 --mode pid --target-rpm 8 --ramp-ms 500 --hold-ms 600 --repetitions 2
python Tools\capture_velocity_test.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\velocity\yaw_vel_08rpm_linearff_run01 --mode velocity-ff --target-rpm 8 --ramp-ms 500 --hold-ms 600 --repetitions 2
python Tools\capture_velocity_test.py --port COM7 --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\velocity\yaw_vel_08rpm_directionalff_run01 --mode full-ff --target-rpm 8 --ramp-ms 500 --hold-ms 600 --repetitions 2
```

三组之间将Yaw重新放回中位，机械状态和Pitch初始角保持一致。模式3中的`full-ff`名称为协议兼容保留，当前含义是“方向相关的摩擦补偿 + 速度比例前馈”，不包含加速度前馈。
