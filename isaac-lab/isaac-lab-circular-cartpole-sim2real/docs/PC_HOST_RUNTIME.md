# PC Host Runtime

## 1. Create or update the conda environment

```powershell
& "E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\scripts\host_runtime\create_windows_host_env.ps1"
```

The environment will be created at:

```text
E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\.conda\cartpole-s2r-host
```

## 2. Activate the environment

```powershell
conda activate E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\.conda\cartpole-s2r-host
```

## 3. Monitor raw MC02 USB CDC messages

Use this first to verify that the board is sending `0x0201`, `0x0202`, or `0x0203`.

```powershell
$env:PYTHONPATH="E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real"
python -m host_runtime mc02-monitor --port COM7 --max-frames 20
```

## 4. Enable the motor

```powershell
$env:PYTHONPATH="E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real"
python -m host_runtime enable --port COM7 --motor-id 1 --clear-fault
```

## 5. Run a sine-wave position-delta test

Use this before RL policy control.

```powershell
$env:PYTHONPATH="E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real"
python -m host_runtime mc02-sine --port COM7 --motor-id 1 --auto-enable --amplitude 0.08 --frequency-hz 0.5 --kp 8.0 --kd 0.2
```

## 6. Run the exported ONNX policy

`policy.onnx` should come from Isaac Lab `play.py`.

```powershell
$env:PYTHONPATH="E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real"
python -m host_runtime mc02-policy --port COM7 --motor-id 1 --auto-enable --policy E:\path\to\policy.onnx --kp 8.0 --kd 0.2 --action-limit-rad 0.12
```

## Notes

- `mc02-monitor`, `mc02-sine`, and `mc02-policy` directly match the current MC02 firmware USB A5 frame format.
- `HostRuntime`, observation history, and ONNX inference still use the original sim2real contract:
  `base_pos, flex1_pos, base_vel, flex1_vel, last_action`, with `history_length = 3`.
- If the board sends `0x0203`, the runtime uses it directly.
- If the board only sends `0x0201` and `0x0202`, the runtime will synthesize one `SensorFrameV1` from the latest motor and encoder states.
