# Pitch Identification and Tuning Plan

## Control structure

Use IMU attitude and angular rate for control:

```text
Pitch angle target
  -> IMU-pitch position controller
  -> gyro-Y angular-rate target
  -> gyro-Y rate controller
  -> gravity + friction + velocity/acceleration feedforward
  -> Pitch motor current
```

The motor encoder remains an independent mechanical-limit and plausibility sensor. Existing free-fall data gives a 0.9994 correlation between the derivative of IMU Pitch and `gyro_y`, confirming the new physical Pitch-rate channel. Integer motor RPM is too coarse for the final high-performance inner loop.

## Identification stages

1. Hold Yaw at its starting attitude.
2. Identify Pitch current-to-gyro-Y dynamics locally around several operating angles.
3. Measure static holding current at `-30, -15, 0, +15, +30 deg`.
4. Fit gravity feedforward as `u_g(theta) = A*sin(theta + phi) + b` (or the equivalent cosine form selected by the fitted phase).
5. Estimate positive/negative friction and dynamic velocity/acceleration terms.
6. Use the identified values as the initial controller, then run bounded parameter tuning.

## Tracking-only Pitch score

No soft current or control-variation penalty is used. Candidate ranking should include:

- angle RMSE;
- maximum angle error;
- positive/negative overshoot;
- 2% settling time;
- steady-state error;
- high-frequency attitude/rate vibration;
- repeated-move consistency.

Current, temperature, speed, encoder travel, CAN/IMU health, and instability remain hard rejection gates. “No competition power limit” does not remove motor, driver, thermal, or mechanical safety limits.

## Validation trajectories

- Small steps: ±3 deg and ±5 deg.
- Normal tracking: ±10 deg trapezoids with multiple ramp rates.
- Larger unseen validation: ±15 deg.
- Separate disturbance test with the camera and launcher in their competition configuration.

Tune on the small/normal set and reserve the ±15 deg and disturbance records for final validation.
