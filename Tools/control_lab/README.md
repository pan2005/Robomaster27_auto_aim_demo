# Gimbal Control Lab

Run from the project root:

```powershell
python Tools\run_control_lab.py
```

The installed environment currently provides PyQt5, pandas, NumPy, pyserial, and MATLAB R2024a.

## System identification

- Select Yaw or Pitch and configure step/PRBS acquisition. Pitch identification holds Yaw and uses gyro Y as the physical-rate output.
- Select independent training and validation CSV files.
- Run MATLAB fitting and display the first-order-plus-delay transfer function, parameters, and validation fits in the GUI.
- Launch MATLAB System Identification with `sysid_training_data` and `sysid_validation_data` already present in the base workspace. In the app, import these variables from Workspace.

System-identification acquisition requires a firmware build with `SYSTEM_IDENTIFICATION_MODE=ON` and a corresponding flash operation. The GUI builds the correct ELF but does not flash the target automatically.

## Controller tuning

- Set 11 bounded controller parameters at runtime through velocity protocol v3.
- Manually enter a complete parameter set, save/load reusable JSON presets, and run that exact set once for scoring.
- Select any subset of the 11 parameters, define min/max/step ranges, and run bounded multi-round coordinate-descent tuning.
- A quick single-parameter friction-blend scan remains available for diagnostic experiments.
- Display the current aggregate score and key tracking, zero-crossing, current, and Pitch metrics.
- Store the requested and device-echoed parameters in `notes/tuning_params` and the captured run JSON.

Controller tuning requires a firmware build with `VELOCITY_CONTROL_TEST_MODE=ON`. Flash the generated ELF once; subsequent parameter candidates do not require recompilation or reflashing.

## Data management

The Data tab scans all experiment JSON files and maintains `Experiments/experiment_index.sqlite3`. Raw `.bin`, decoded `.csv`, metadata `.json`, parameter snapshots, MATLAB models, and plots remain under their session directory. Double-click a row to open its folder.

Lower tuning scores are better. The `tracking_only_v2` score contains no current-RMS or control-variation penalty; those quantities remain visible diagnostics and hard safety gates. A safety fault or discontinuous/missing data invalidates a candidate regardless of score. Scores from different score versions must not be compared directly.
