# Experiment Log

## 2026-10-01

### `axis-check/yaw_step_amp0300_run01`

- Result: axis and communication check passed.
- Yaw motor travel: about 1.36 deg.
- IMU Yaw range: about 1.33 deg.
- Physical Yaw still corresponds to IMU yaw / gyro Z.
- Not used for model fitting because excitation and travel are too small.

### `step/yaw_step_amp0800_run01`

- Result: invalid for fixed-configuration Yaw identification.
- Completed 4000 samples without a firmware safety fault.
- Yaw motor travel: about 23.82 deg.
- IMU Pitch changed about 14.81 deg because the Pitch motor was not holding position.
- The moving Pitch assembly changes the Yaw inertia and introduces axis coupling during the run.

### `step/yaw_step_amp1200_run01`

- Result: invalid and aborted.
- Aborted after 747 samples with reason `0x0010` (`SYSID_FAULT_ANGLE_LIMIT`).
- Yaw motor travel reached about 44.96 deg, i.e. the firmware relative-angle limit.
- IMU Pitch changed about 36.40 deg during the run.
- Do not mix this run into model training or validation data.

## Required correction before formal acquisition

Pitch must remain at a defined angle during every Yaw identification run. Either mechanically lock the Pitch axis or add an independent Pitch holding loop in `system_identification_task`; record the chosen Pitch angle and holding method as experiment metadata.

### Firmware correction

- Selected the active Pitch-holding solution rather than a mechanical lock.
- Protocol upgraded to v3.
- Pitch motor `0x205` holds the experiment-start `imu.pitch` value with a cascaded position/speed controller.
- Added ±5 deg IMU/encoder deviation protection for initial commissioning, ±60 rpm Pitch speed protection, and ±5000 Pitch holding-current limit.
- Added `pitch_hold_current` to every sample.

### `axis-check/yaw_step_amp0800_pitchhold_run01`

- Result: Pitch holding validation passed; 4000/4000 samples, no dropped samples and no safety fault.
- Yaw motor travel: about 24.08 deg; IMU Yaw range: about 24.01 deg.
- IMU Pitch range: about 0.135 deg, reduced from 14.81 deg in the unpowered-Pitch 800-current run (about 99.1% reduction).
- Pitch holding command: peak 93, 99th percentile 11, far below the ±5000 limit.
- Pitch started near 3.582 deg and ended near 3.670 deg (about 0.088 deg net drift).
- The Pitch feedback direction is correct and this holding configuration is accepted for formal Yaw acquisition.

### Safety-limit update

- After an on-site clearance check, the Yaw relative-angle protection was increased from ±45 deg to ±90 deg for system-identification mode only.
- Yaw current remains limited to ±2000 and Yaw speed remains limited to ±60 rpm.

### Formal Pitch-held step tests

All three runs completed with 4000/4000 samples, no host-detected missing samples, no firmware-dropped samples, and no safety fault.

| Run | Yaw travel | Peak Yaw speed | IMU Pitch range | Peak Pitch command |
|---|---:|---:|---:|---:|
| `step/yaw_step_amp0800_pitchhold_run01` | 27.77 deg | 9 rpm | 0.131 deg | 213 |
| `step/yaw_step_amp1200_pitchhold_run01` | 58.89 deg | 18 rpm | 0.290 deg | 200 |
| `step/yaw_step_amp1600_pitchhold_run01` | 85.47 deg | 21 rpm | 0.381 deg | 378 |

These runs are accepted as formal step-identification data. The plant remained within the ±90 deg Yaw, ±60 rpm Yaw, and ±5 deg Pitch limits.

### Pitch commissioning run with corrected filename

- Canonical file: `raw/pitch/axis-check/pitch_step_amp1600_run01`.
- It was originally captured into the accidental sibling session directory with the misleading stem `pitch_step_amp0200_run01`; the three raw artifacts were moved and renamed together without changing their contents.
- The command line and acquisition metadata specify `--amplitude 1600`; therefore the true excitation amplitude is 1600, not 200 and not 1200.
- Result: completed 4000/4000 samples with no dropped samples, CRC errors, or safety fault.
- IMU Pitch range was about 3.69 deg; Yaw range about 0.25 deg; peak Pitch motor speed was 8 rpm.
- Keep the raw file for traceability, but do not select it by filename as a 200-current run. Prefer renaming future runs before acquisition; metadata is authoritative.

### Pitch gravity scan `pitch_gravity_m30_p30_step15_run01`

- Result: complete, 30000/30000 samples, no host loss, CRC error, firmware drop, or safety fault.
- The unrestricted four-parameter sine/cosine/bias/friction fit has condition number 35.45 over only ±30°; its apparent amplitude 5268.65 is not used directly as feedforward.
- Opposite-direction pairing at -15°, 0°, and +15° cancels most static friction and gives `u_ff = -2298.8448*sin(theta) - 99.3683` with paired-point RMSE 16.55 current units and condition number 4.45.
- Estimated direction-dependent static friction is about 670.90 current units. It remains disabled in the first gravity-feedforward A/B test.
- This is a local model limited to the measured ±30° interval. Wider-angle use requires additional safe data.

### Pitch PRBS 0 deg preliminary pair

- Both 1200-current, 30 ms-switch runs completed with 15000/15000 samples and no transport or safety fault.
- Pitch peak-to-peak motion was 2.61 deg (training) and 2.37 deg (validation), while gyro-Y RMS was about 0.27 rad/s and 1-30 Hz coherence was generally above 0.80.
- The data is informative as a local small-signal experiment, but first/second-order and low-order state-space validation fits were only about 29-52%, with correlated residuals. Do not derive final dynamic feedforward from this pair.
- The earlier suggestion to create ±15 deg datasets by manually positioning Pitch was rejected as inaccurate. Protocol message 0x07 now provides automatic move, settle, PRBS-only logging, return, and final settle.

### Pitch PRBS automatic +15/-15 working points

- Both runs completed with 15000/15000 samples and no transport or safety fault.
- Measured operating means were +15.189 deg and -15.258 deg; peak-to-peak PRBS motion was 2.410 deg and 2.627 deg.
- +15-trained model: `K=0.000490014`, `tau=24.149 ms`, delay 7.298 ms; validation on -15 gave 57.41% first-order and 59.78% second-order fit.
- -15-trained model: `K=0.000617261`, `tau=32.064 ms`, delay 7.000 ms; validation on +15 gave 47.39% first-order and 54.75% second-order fit.
- Inverse coefficients are `Kv=2040.76, Ka=49.28` and `Kv=1620.06, Ka=51.95`. `Ka` is consistent, while `Kv` changes about 26%, so the first dynamic-feedforward A/B run uses midpoint coefficients at scale 0.5.

### Pitch dynamic feedforward scale 0.50 A/B

- Baseline was gravity feedforward only; candidate added `0.5*(1830.4092*omega_ref + 50.6137*alpha_ref)`. Both used the same half-cosine trajectory.
- Ramp RMSE improved from 2.5357 deg to 1.9668 deg (22.44%); overall tracking RMSE improved 19.81%.
- Ramp P95 error improved from 4.7180 deg to 3.6713 deg, and maximum ramp error from 4.8783 deg to 3.7600 deg.
- Upward/downward ramp RMSE improved from 1.9376/3.4324 deg to 1.4944/2.6719 deg, so the benefit is present in both directions.
- Hold RMSE was effectively unchanged at 0.6974 vs 0.7001 deg. No new target crossing or overshoot was observed at the hold transitions.
- PID-current RMS fell from 615.45 to 461.04, while total-current RMS also fell from 610.37 to 593.49. Peak total command was 1895, far below the ±5000 limit.
- Scale 0.50 is accepted as safe and beneficial. Test scale 0.75 next; keep the existing baseline file.
