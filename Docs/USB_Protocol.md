# 云台 USB CDC 二进制协议

PC 端权威实现位于 `Tools/gimbal_terminal/protocol.py`，STM32 端实现位于 `Tasks/gimbal_protocol.[ch]`。所有多字节整数均采用小端序。

## 帧格式

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | Sync | 固定 `A5 5A` |
| 2 | 1 | Version | 当前为 `1` |
| 3 | 1 | Type | 消息类型 |
| 4 | 1 | Length | 载荷长度，最大 48 字节 |
| 5 | 2 | Sequence | 16 位循环序号 |
| 7 | N | Payload | 消息载荷 |
| 7+N | 2 | CRC | CRC-16/CCITT-FALSE |

CRC 覆盖从 `Version` 到载荷末尾，不包含同步字和 CRC 本身；多项式 `0x1021`，初值 `0xFFFF`，不反射、不异或输出。完整帧长度为 `9 + N` 字节。解析器能从半帧、粘包、随机噪声、非法长度和 CRC 错误中重新同步。

## 消息类型

| 类型 | 名称 | 方向 | 周期/用途 |
|---:|---|---|---|
| `0x01` | HELLO | PC → MCU | 协议握手 |
| `0x02` | TIME_SYNC_REQ | PC → MCU | PC 单调时钟同步请求 |
| `0x10` | CONTROL | PC → MCU | 100 Hz 手动增量及心跳 |
| `0x11` | DISARM | PC → MCU | 立即失能 |
| `0x12` | ESTOP | PC → MCU | 急停并失能 |
| `0x13` | AIM_TARGET | PC → MCU | 100 Hz 自动绝对轨迹参考 |
| `0x80` | STATUS | MCU → PC | 20 Hz 云台状态 |
| `0x81` | ATTITUDE | MCU → PC | 100 Hz 时间对齐姿态 |
| `0x82` | TIME_SYNC_RESP | MCU → PC | 时钟同步响应 |

## PC 到 MCU 载荷

### CONTROL (`<Bhh>`, 5 字节)

| 字段 | 类型 | 单位 |
|---|---|---|
| flags | `uint8` | bit0 armed，bit1 fire |
| yaw_delta | `int16` | mdeg |
| pitch_delta | `int16` | mdeg |

Yaw 正方向为向左，Pitch 正方向为向上。只有 CRC 正确且序号更新的增量才累加一次。零增量有效帧用于保持最后目标和刷新心跳。

### AIM_TARGET (`<BBIiiiiii>`, 30 字节)

| 字段 | 类型 | 单位/说明 |
|---|---|---|
| flags | `uint8` | bit0 valid，bit1 armed，bit2 fire |
| mode | `uint8` | 当前固定为 1 |
| execute_at | `uint32` | MCU 时域 ms |
| yaw, pitch | `int32 × 2` | mdeg，绝对目标 |
| yaw_rate, pitch_rate | `int32 × 2` | mdeg/s |
| yaw_accel, pitch_accel | `int32 × 2` | mdeg/s² |

自动模式直接使用位置、速度和加速度参考，避免再次经过本地梯形规划器。固件按执行时刻进行短时外推，并始终执行两轴 ±90° 限位。

### TIME_SYNC_REQ (`<Q>`, 8 字节)

载荷为 PC 单调时钟微秒值。MCU 在响应中原样回显，以便 PC 用往返时间估计时钟偏移。

DISARM 与 ESTOP 无载荷。HELLO 的具体版本握手以两端源码为准。

## MCU 到 PC 载荷

### TIME_SYNC_RESP (`<QI>`, 12 字节)

依次为回显的 PC 单调时钟 `uint64 us` 和 MCU `uint32 ms` 时钟。

### ATTITUDE (`<IHhhhhiiiihhiii>`, 46 字节)

| 字段 | 类型 | 说明 |
|---|---|---|
| mcu_time | `uint32` | MCU ms 时钟 |
| flags | `uint16` | IMU、电机、控制与安全状态 |
| quaternion | `int16 × 4` | w,x,y,z，除以 32767 得归一化值 |
| pitch, yaw | `int32 × 2` | mdeg |
| pitch_rate, yaw_rate | `int32 × 2` | mdeg/s |
| temperature | `int16` | 0.01 °C |
| calibration_temperature | `int16` | 0.01 °C |
| gyro_offset | `int32 × 3` | µrad/s |

解码器也兼容不带温度与零偏诊断数据的旧 30 字节 ATTITUDE。

### STATUS (`<HiiiihhBBHHB>`, 29 字节)

依次包含状态位、实际 Pitch/Yaw、目标 Pitch/Yaw（均为 mdeg）、两轴转速 rpm、两轴温度、协议错误计数、最后控制序号和协议版本。

## 安全语义

- CONTROL 与 AIM_TARGET 同时承担心跳职责；超过约 200 ms 无有效控制帧时清零输出并失能。
- 时间异常、目标越过 ±90°、CRC 错误或未显式 armed 时不会执行目标，并强制清除 fire。
- 重复及乱序序号不会重复执行角度增量。
- 正常重连需要重新握手并显式 armed。
- fire 当前仅被解析、回显和记录，不驱动任何发射硬件。
- STATUS 发送遇到 USB CDC busy 时可丢弃当周期，不阻塞 1 kHz 控制环。

