# L1 Real Deployment Contract

This document fixes the host/runtime contract for the first real-hardware bring-up.

## Scope

- Target task: `RK-Circular-Cartpole-S2R-L1-REAL-V0`
- Training baseline: `RK-Circular-Cartpole-S2R-L1-V1`
- Runtime topology: Ubuntu host policy inference -> USB CDC -> MC02 -> FDCAN -> DM4310
- Sensor topology: DM4310 feedback for base joint, MT6701 for flex joint
- Control loop: 250 Hz

## Training vs. deployment

`L1 real` is intentionally a thin wrapper around `L1 v1`.

- Rewards stay the same.
- Domain randomization stays the same.
- Observation terms stay the same.
- The only semantic change is the action term:
  policy output is interpreted as a relative position command for `base_to_fixed`.

This keeps comparisons clean:

- If the upstream `sim2real` weight works, the chain is aligned.
- If it does not, the mismatch is likely in latency, sign, zeroing, or actuation details.

## Observation contract

Per-frame feature order is fixed:

1. `base_pos_rad`
2. `flex1_pos_rad`
3. `base_vel_rad_s`
4. `flex1_vel_rad_s`
5. `last_action_delta_rad`

History length is fixed at `3`.

The flattened policy input is term-major, oldest to newest:

1. `base_pos[t-2], base_pos[t-1], base_pos[t]`
2. `flex1_pos[t-2], flex1_pos[t-1], flex1_pos[t]`
3. `base_vel[t-2], base_vel[t-1], base_vel[t]`
4. `flex1_vel[t-2], flex1_vel[t-1], flex1_vel[t]`
5. `last_action[t-2], last_action[t-1], last_action[t]`

If history is not full, each term bucket is left-padded with zeros.

## Binary protocol

All frames are little-endian and protected by `CRC-16/CCITT-FALSE`.

### `SensorFrameV1`

- Magic: `SEN1`
- Version: `1`
- Kind: `1`
- Length: `46` bytes
- Fields:
  - `seq: uint32`
  - `t_us: uint64`
  - `base_pos_rad: float32`
  - `base_vel_rad_s: float32`
  - `flex1_pos_rad: float32`
  - `flex1_vel_rad_s: float32`
  - `motor_temp_c: float32`
  - `status_flags: uint32`
  - `crc16: uint16`

### `ActionFrameV1`

- Magic: `ACT1`
- Version: `1`
- Kind: `2`
- Length: `42` bytes
- Fields:
  - `seq: uint32`
  - `t_us: uint64`
  - `base_pos_delta_rad: float32`
  - `kp: float32`
  - `kd: float32`
  - `watchdog_ms: uint32`
  - `mode_flags: uint32`
  - `crc16: uint16`

## Host responsibilities

- Receive `SensorFrameV1`
- Maintain the 3-frame history buffer
- Run `policy.onnx` or `policy.pt`
- Clip the action if needed
- Send `ActionFrameV1`
- Write JSONL logs for replay and consistency checks

## MC02 responsibilities

- Read MT6701 over SPI
- Read/command DM4310 over FDCAN
- Normalize sign, zero position, and wrap handling
- Estimate velocity locally
- Apply watchdog and fault handling

The board does not build policy history and does not run network inference in the first milestone.

## Bring-up commands

Offline sine reference:

```powershell
python -m host_runtime sine --steps 2000 --amplitude 0.05 --frequency-hz 0.5 --log logs\sine.jsonl
```

Offline policy smoke test:

```powershell
python -m host_runtime policy --policy path\to\policy.onnx --steps 1000 --log logs\policy.jsonl
```

Replay validation:

```powershell
python -m host_runtime replay --log logs\policy.jsonl --no-policy-check
```

## Reuse of upstream weights

If the upstream author's hardware and mechanism match yours closely, use the upstream `sim2real` weight first for:

- protocol smoke testing
- sign and zero validation
- tethered `L1` verification

Retraining or fine-tuning is still recommended when:

- your observation contract changes
- your action semantics change
- host/board latency differs materially
- zero offsets or joint directions differ
- the policy can balance but is not robust
