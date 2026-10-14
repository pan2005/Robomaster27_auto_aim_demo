# Yaw PRBS Model Analysis

## Data acceptance

All four 20 s PRBS records contain 20,000 consecutive 1 kHz samples. CRC errors, host-detected missing samples, firmware-dropped samples, and safety flags are all zero. Positive and negative excitation counts are approximately balanced.

| Dataset | Yaw travel | Peak Yaw speed | Pitch range | Peak Pitch command |
|---|---:|---:|---:|---:|
| 40 ms train | 55.99 deg | 16 rpm | 0.819 deg | 832 |
| 40 ms validation | 42.06 deg | 16 rpm | 1.556 deg | 1212 |
| 20 ms train | 26.85 deg | 14 rpm | 0.582 deg | 816 |
| 20 ms validation | 25.18 deg | 14 rpm | 0.580 deg | 962 |

## Independent-validation fit

| Dataset pair | First order + delay | Second-order TF | Second-order state space |
|---|---:|---:|---:|
| 40 ms | 59.86% | 56.68% | 57.82% |
| 20 ms | 57.12% | 56.45% | 56.37% |

The first-order-plus-delay model has the best validation fit in both cases:

- 40 ms: gain `0.0065 rpm/command`, time constant `0.0017 s`, delay `0.0056 s`.
- 20 ms: gain `0.0059 rpm/command`, time constant `0.0029 s`, delay `0.0039 s`.

The two estimates are reasonably consistent, but the residuals remain strongly autocorrelated and correlated with the input. A single linear model therefore does not explain all observed behavior.

## Nonlinearity seen in step data

Approximate late-plateau speeds show a substantial dead zone and directional asymmetry:

| Command | Positive speed | Negative speed |
|---:|---:|---:|
| 800 | 0.50 rpm | -2.99 rpm |
| 1200 | 4.88 rpm | -7.64 rpm |
| 1600 | 8.30 rpm | -11.01 rpm |

This indicates that a useful velocity feedforward should include direction-dependent Coulomb/static-friction compensation plus a linear velocity term. Using only the inverse linear gain (`command = speed / K`) will be inaccurate near zero speed and will differ by direction.

## Recommendation

Use the first-order-plus-delay model as the initial dynamic approximation, but identify a piecewise static feedforward map separately for positive and negative motion. Validate the resulting feedforward on a velocity-reference experiment before tuning the feedback PID around it.
