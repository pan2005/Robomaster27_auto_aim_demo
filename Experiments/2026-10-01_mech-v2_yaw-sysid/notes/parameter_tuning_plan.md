# Controller Parameter Fine-Tuning Plan

## Goal

System identification supplies a physically meaningful initial controller. Fine tuning then minimizes measured tracking error without sacrificing safety, smoothness, or actuator margin. Tuning and final validation must use different trajectories.

## Tunable parameters

- Positive/negative Coulomb-friction commands.
- Positive/negative velocity-feedforward slopes.
- Friction blend width and friction slew rate.
- Velocity PID `Kp`, `Ki`, integral limit, and zero-target leak time constant.
- Acceleration feedforward only after the static map and zero crossing are stable.

## Firmware interface to add

Extend the velocity-test USB protocol with `SET_PARAMS` and `GET_PARAMS` messages. Parameters are RAM-only during experiments, bounded by immutable firmware safety limits, echoed in the ACK, and written into every run's JSON metadata. No test parameter may change the motor-offline, overspeed, angle, Pitch-deviation, or CAN-fault protections.

## Objective function

Use a normalized tracking-only score rather than RMSE alone:

```text
score = 0.40 * moving_RMSE
      + 0.20 * maximum_tracking_error
      + 0.20 * steady_state_error
      + 0.10 * zero_crossing_peak_error
      + 0.10 * zero_target_residual_speed
```

This is score version `tracking_only_v2`. Current RMS, current peak, saturation ratio, and control total variation remain mandatory diagnostics and hard safety gates, but they carry no soft score penalty because the RoboMaster gimbal is not power-limited. Historical scores produced by the earlier objective must not be compared numerically with v2 scores.

Any safety fault, dropped sample, current saturation above the allowed ratio, or unstable oscillation invalidates the candidate instead of merely adding a small penalty.

## Experimental stages

1. Tune zero-crossing behavior at 8 rpm: integral leak, friction slew, and blend width.
2. Tune positive and negative static-map parameters with 4, 8, and 12 rpm plateaus.
3. Tune feedback `Kp/Ki` while holding the accepted feedforward map fixed.
4. Optionally add acceleration feedforward using multiple ramp rates.
5. Freeze parameters and validate on unseen speeds, ramp rates, and direction sequences.

Begin with bounded coordinate/grid sweeps so parameter effects remain interpretable. After the score and safety gates are trusted, the same interface can support Bayesian optimization or another sample-efficient optimizer. Preserve every candidate run and its metadata; never tune on the final validation set.

## Implemented platform

- PyQt5 entry point: `Tools/run_control_lab.py`.
- Manual mode supports editing all parameters, saving/loading parameter presets, and running one exact parameter set for an immediate score without invoking an optimizer.
- Velocity USB protocol v3 supports bounded `SET_PARAMS` and `GET_PARAMS` messages for all 11 parameters listed above.
- Candidate parameter snapshots are stored under `notes/tuning_params`; the device-echoed values are stored in each run JSON.
- Multi-parameter coordinate-descent sweeps run sequentially, reject invalid data, score each candidate, update one dimension at a time, refill the GUI with the best valid parameter set, and save the complete history to `results/tuning_best_params.json`.
- MATLAB batch identification exports a JSON transfer-function summary for display in the GUI.
- MATLAB System Identification can be launched with training and validation `iddata` variables prepared in its base workspace.
- The Data tab maintains the cross-session SQLite index `Experiments/experiment_index.sqlite3`.
