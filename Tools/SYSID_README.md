# Dual-axis system-identification mode

`SYSTEM_IDENTIFICATION_MODE=ON` creates only the independent identification controller; it never starts `GimbalTask`. Protocol v4 supports `--axis yaw` and `--axis pitch`.

```powershell
cmake -S . -B cmake-build-debug-stm32 -DSYSTEM_IDENTIFICATION_MODE=ON -DVELOCITY_CONTROL_TEST_MODE=OFF
cmake --build cmake-build-debug-stm32 --target Project1 -j 14
```

Conservative Pitch direction/safety check:

```powershell
python Tools\capture_sysid.py --port COM7 --axis pitch `
  --output Experiments\2026-10-01_mech-v2-yaw-sysid\raw\pitch\axis-check\pitch_step_amp0200_run01 `
  --type step --amplitude 200 --duration-ms 4000
```

- Yaw identification holds Pitch at its starting IMU attitude and injects current into Yaw.
- Pitch identification holds Yaw, keeps Pitch locally supported around its starting attitude, and adds the requested excitation to the Pitch motor command.
- `excitation_current` is the injected identification signal.
- `output_current` is the actual total current sent to the identified motor.
- `support_hold_current` is the other axis's holding current.
- Motor encoder/angle/RPM/current columns always refer to the identified axis.
- Pitch model fitting uses `gyro_y_rad_s`; Yaw currently uses motor `speed_rpm`.

For Pitch PRBS, the motor command is `position-feedback + gravity-feedforward + excitation`. The CSV keeps `excitation_current` and the actual `output_current` separately. Plant fitting uses the actual total motor command; the external-injection response can also be studied with `excitation_current`. Because the experiment is closed loop, accept a model only after validation with a different PRBS seed.

Initial Pitch dynamic-identification pair around IMU Pitch 0°:

```powershell
python Tools\capture_sysid.py --port COM7 --axis pitch --type prbs `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\prbs\pitch_prbs_amp1200_sw30ms_train_run01 `
  --amplitude 1200 --switch-ms 30 --duration-ms 15000 --seed 0x13572468

python Tools\capture_sysid.py --port COM7 --axis pitch --type prbs `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\prbs\pitch_prbs_amp1200_sw30ms_valid_run01 `
  --amplitude 1200 --switch-ms 30 --duration-ms 15000 --seed 0x24681357
```

Keep Pitch near 0° before both runs and do not change payload, cable routing, supply voltage, or controller firmware between training and validation. These two runs identify the local dynamics around 0°; they do not prove that the same dynamic model applies across the full Pitch travel.

For nonzero Pitch operating points, never position the axis by hand. Add `--operating-angle-deg`; firmware then executes `start angle -> requested angle -> settle -> PRBS -> start angle -> settle -> current off`. Only the PRBS interval is written as sample frames, so positioning and return motion cannot contaminate identification data. Example:

```powershell
python Tools\capture_sysid.py --port COM7 --axis pitch --type prbs `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\prbs\pitch_prbs_opp15_amp1200_sw30ms_train_run01 `
  --operating-angle-deg 15 --move-ms 2500 --settle-ms 1000 `
  --amplitude 1200 --switch-ms 30 --duration-ms 15000 --seed 0x51A1B2C3
```

The metadata records requested angle, measured mean/range during PRBS, move/settle times, and total sequence duration. The requested operating point is limited to ±20°, and firmware rejects starts more than 25° away from it.

Hard limits include injected current ±2000, actual motor command ±5000, speed ±60 rpm, Yaw relative travel ±90°, Pitch relative travel ±30°, support-axis deviation ±5°, and duration 60 seconds. Motor/IMU/CAN faults abort on the next 1 ms control cycle.

## Pitch gravity-torque scan

The gravity scan commands a slow absolute Pitch angle trajectory while Yaw holds its starting attitude. It records both travel directions so gravity, bias, and direction-dependent friction can be separated, then returns to the experiment-start Pitch angle before disabling current. Start with Pitch near 0°; the scan has an independent ±40° relative-travel guard.

```powershell
python Tools\capture_pitch_gravity.py --port COM7 `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\gravity\pitch_gravity_m30_p30_step15_run01 `
  --min-deg -30 --max-deg 30 --step-deg 15 `
  --ramp-ms 1500 --settle-ms 1500 --repetitions 1

python Tools\analyze_pitch_gravity.py `
  --input Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\gravity\pitch_gravity_m30_p30_step15_run01.csv `
  --output-prefix Experiments\2026-10-01_mech-v2_yaw-sysid\models\pitch_gravity_run01
```

The fitted model is `u = a*sin(theta) + b*cos(theta) + c + f*direction`. `theta` is the measured IMU Pitch angle, not the requested angle. The fit uses only the latter half of each settle period. Raw `.bin`, decoded `.csv`, acquisition `.json`, fitted point `.csv`, and model summary `.json` are kept separately.

For the first controller A/B test, use the direction-paired local model exported as `recommended_gravity_sin_coefficient` and `recommended_bias_current`. The current firmware uses `u_ff = -2298.8448*sin(imu.pitch) - 99.3683`; this model is commissioned only over the measured ±30° interval. Motion-friction feedforward is intentionally disabled so the A/B test changes only gravity compensation.

After flashing the updated identification firmware, put Pitch within 10° of the requested center and collect both runs without changing the mechanical configuration:

```powershell
python Tools\capture_pitch_control.py --port COM7 --mode pid `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_pid_run01 `
  --center-deg 0 --amplitude-deg 15 --ramp-ms 1000 --hold-ms 1500 --repetitions 2

python Tools\capture_pitch_control.py --port COM7 --mode gravity-ff `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_gravityff_run01 `
  --center-deg 0 --amplitude-deg 15 --ramp-ms 1000 --hold-ms 1500 --repetitions 2

python Tools\compare_pitch_control.py `
  --pid Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_pid_run01.csv `
  --feedforward Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_gravityff_run01.csv `
  --output-prefix Experiments\2026-10-01_mech-v2_yaw-sysid\analysis\pitch_gravityff_ab_run01
```

The profile is `center -> +amplitude -> -amplitude -> center`, repeated twice. Both modes use exactly the same cascaded PID gains. The comparison score uses tracking errors only; current is reported diagnostically but is not part of the score.

### Pitch dynamic feedforward candidate

Cross-validation between the automatically positioned +15° and -15° PRBS runs produced `Kv=2040.76, Ka=49.28` and `Kv=1620.06, Ka=51.95`. Firmware uses their midpoint `Kv=1830.4092`, `Ka=50.6137`, with a runtime scale. Because cross-angle validation fit is only about 47-60%, the first test must use scale 0.5. `dynamic-ff` always includes gravity compensation as well.

All Pitch control modes now use the same half-cosine ramp, providing finite reference velocity and acceleration:

```powershell
python Tools\capture_pitch_control.py --port COM7 --mode gravity-ff `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_gravityff_cosine_run01 `
  --center-deg 0 --amplitude-deg 15 --ramp-ms 1000 --hold-ms 1500 --repetitions 2

python Tools\capture_pitch_control.py --port COM7 --mode dynamic-ff `
  --output Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_dynamicff_s050_run01 `
  --center-deg 0 --amplitude-deg 15 --ramp-ms 1000 --hold-ms 1500 --repetitions 2 `
  --dynamic-ff-scale 0.5

python Tools\compare_pitch_control.py `
  --baseline Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_gravityff_cosine_run01.csv `
  --candidate Experiments\2026-10-01_mech-v2_yaw-sysid\raw\pitch\control\pitch_pos15_dynamicff_s050_run01.csv `
  --output-prefix Experiments\2026-10-01_mech-v2_yaw-sysid\analysis\pitch_dynamicff_s050_ab_run01
```

Do not compare against an older linear-ramp control log; the baseline and candidate must both be acquired with this firmware so their reference trajectories are identical.

Binary frames use little-endian fields: header `uint16 magic=0xA55A`, `uint8 version=4`, `uint8 type`, `uint16 payload_length`, `uint32 sequence`; CRC-16/CCITT (initial value 0xFFFF) follows the payload. Message types are CONFIG=0x01, START=0x02, STOP=0x03, QUERY=0x04, GRAVITY_CONFIG=0x05, PITCH_CONTROL_CONFIG=0x06, PITCH_PRBS_CONFIG=0x07, ACK=0x81, STATUS=0x82, SAMPLE=0x83, GRAVITY_SAMPLE=0x84, and PITCH_CONTROL_SAMPLE=0x85.
