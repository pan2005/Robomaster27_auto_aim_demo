# Yaw Velocity-Control Comparison

## Test configuration

- Target profile: `0 -> +8 rpm -> 0 -> -8 rpm -> 0`, repeated twice.
- Ramp: 500 ms; hold: 600 ms.
- Same feedback PID in every run.
- Pitch attitude holding enabled.
- No dropped samples, CRC errors, safety flags, or current saturation in any run.

## Overall metrics

| Controller | RMSE | IAE | Max error | Current RMS | Current peak | PID-output RMS |
|---|---:|---:|---:|---:|---:|---:|
| PID only | 3.177 rpm | 20.566 rpm*s | 8.000 rpm | 772.27 | 1395 | 772.27 |
| Linear velocity FF + PID | 2.097 rpm | 14.594 rpm*s | 4.656 rpm | 933.62 | 1816 | 669.24 |
| Directional friction + velocity FF + PID | 1.401 rpm | 8.955 rpm*s | 5.016 rpm | 875.99 | 1737 | 270.09 |

Relative to PID only, the directional feedforward reduced total RMSE by about 55.9%, IAE by about 56.5%, and PID-output RMS by about 65.0%.

## Constant-speed plateaus

| Controller | Mean at +8 rpm | Mean error | Mean at -8 rpm | Mean error |
|---|---:|---:|---:|---:|
| PID only | 4.159 rpm | +3.841 rpm | -3.375 rpm | -4.625 rpm |
| Linear velocity FF + PID | 5.434 rpm | +2.566 rpm | -5.409 rpm | -2.591 rpm |
| Directional FF + PID | 7.413 rpm | +0.587 rpm | -7.516 rpm | -0.484 rpm |

The directional static map is clearly the best of the three at overcoming the dead zone and reducing directional asymmetry.

## Remaining issue

The directional controller has the highest control total variation and a larger transient maximum error than the linear-FF controller. The plot shows overshoot around ramp endings and zero crossings. The likely causes are the friction term transitioning too aggressively and the feedback integrator retaining charge while the feedforward contribution changes rapidly.

Before increasing test speed, add bumpless zero-crossing handling or integral leakage/reset near zero target, then repeat at 8 rpm. After that, validate at 4 rpm and 12 rpm to check whether the fitted static map generalizes beyond the identification points.

## Zero-crossing revision for run 02

- Increased total `PID + feedforward` output limit from ±2000 to ±5000; the feedback PID itself remains limited to ±2000.
- Added a 100 ms integral-leak time constant while `|target| < 0.5 rpm`.
- Added a friction-command slew limit of 5000 command/s (5 command per 1 ms cycle).
- Overspeed, ±90 deg Yaw travel, Pitch deviation, motor-offline, IMU, and CAN protections remain unchanged.

## Directional-FF run 02 result

| Metric | Run 01 | Run 02 |
|---|---:|---:|
| Total RMSE | 1.401 rpm | 1.421 rpm |
| IAE | 8.955 rpm*s | 8.985 rpm*s |
| Maximum error | 5.016 rpm | 5.952 rpm |
| Zero-target maximum speed | 5 rpm | 3 rpm |
| Current peak | 1737 | 1610 |
| Control total variation | 103785 | 100574 |
| PID-output RMS | 270.09 | 211.76 |
| Pitch range | 0.377 deg | 0.234 deg |

The revision reduced the zero-return peak, current peak, control variation, and PID workload. Overall RMSE changed only slightly (+1.5%). The remaining maximum error moved to the negative-direction startup: at target `-2.048 rpm`, actual speed reached `-8 rpm` while the friction command was already `-470`. Thus the next parameter to tune is the low-speed friction blend width, not the plateau friction or velocity slope.

## Directional-FF run 03 (`blend_rpm = 2.5`)

| Metric | Run 02 (`blend=1.0`) | Run 03 (`blend=2.5`) |
|---|---:|---:|
| Total RMSE | 1.421 rpm | 1.445 rpm |
| IAE | 8.985 rpm*s | 9.317 rpm*s |
| Maximum error | 5.952 rpm | 5.232 rpm |
| Zero-target mean absolute speed | 0.389 rpm | 0.464 rpm |
| Zero-target maximum speed | 3 rpm | 3 rpm |
| Current peak | 1610 | 1708 |
| Control total variation | 100574 | 108886 |
| PID-output RMS | 211.76 | 216.64 |

The wider blend reduced the negative-start peak error by about 12.1%, confirming the diagnosis, but slightly worsened aggregate tracking, zero-speed settling, and control variation. Both repeated cycles show the same trend, so it is not a single transient outlier. A blend between 1.0 and 2.5 (initially 1.5) is the next sensible candidate if optimizing by hand. Future sweeps should record the initial Pitch angle because runs 01/02/03 started near +2.40, -0.37, and -2.01 deg respectively.
