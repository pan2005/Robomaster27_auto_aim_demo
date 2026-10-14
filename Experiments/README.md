# 实验数据目录规范

每次机械结构、负载、IMU安装或固件协议发生变化，都新建一个独立会话目录，禁止把CSV、BIN、JSON和MAT文件直接放在工程根目录。

目录命名：

```text
YYYY-MM-DD_mech-vN_axis-purpose/
├─ README.md          # 结构、负载、固件、接线、实验矩阵和结论
├─ raw/               # 采集脚本直接输出的 .bin/.csv/.json
├─ models/            # MATLAB辨识结果 .mat
├─ plots/             # 图像和报告导出
└─ notes/             # 机械照片、安装说明和异常记录
```

原始数据文件命名：

```text
轴_信号_幅值_切换周期_用途_重复编号
yaw_step_amp0800_run01
yaw_prbs_amp1200_sw20ms_train_run01
yaw_prbs_amp1200_sw20ms_valid_run01
```

采集脚本会自动创建`--output`所指定的父目录。不得覆盖原始实验；重复实验必须递增`runNN`。

旧结构数据保存在`2026-09-09_mech-v1_yaw`，新结构数据保存在`2026-10-01_mech-v2_yaw-sysid`。不同机械版本的数据不能混合用于同一次参数辨识。
