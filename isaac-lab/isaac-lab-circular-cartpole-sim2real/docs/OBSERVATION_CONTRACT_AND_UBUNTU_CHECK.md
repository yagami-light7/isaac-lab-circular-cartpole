# Observation Contract And Ubuntu Check

This note fixes the current observation contract for the circular-cartpole sim2real path and gives a direct Ubuntu-side Isaac Lab inspection command.

## Current contract

- Contract version: `l1_policy_v1`
- Control loop: `250 Hz`
- History length: `3`
- Policy input dimension: `15`
- History layout: `term-major, oldest to newest`

Per-frame feature order is fixed to:

1. `base_pos`
2. `flex_pos`
3. `base_vel`
4. `flex_vel`
5. `last_action`

More explicitly:

- `base_pos` = relative angle of `base_to_fixed` against its default joint position
- `flex_pos` = relative angle of `fixed_to_flex_1` against its default joint position
- `base_vel` = relative velocity of `base_to_fixed`
- `flex_vel` = relative velocity of `fixed_to_flex_1`
- `last_action` = previous host-side position-delta command sent to the actuator

The current code path keeps `flex_pos` as the raw joint-relative value in the policy path. It does not apply host-side unwrap by default.

## What the Ubuntu check script verifies

`scripts/check_observation_contract.py` prints, for one selected environment:

- the policy observation length
- the five term buckets after history flattening
- raw and wrapped joint angles from Isaac Lab
- raw joint velocities from Isaac Lab

This is the quickest way to verify:

- observation order really is `[base_pos, flex_pos, base_vel, flex_vel, last_action]`
- the flattened observation really is term-major
- `flex_pos` is currently entering the policy as raw relative joint position rather than an unwrapped continuous angle

## Ubuntu command

Run this from the extension root on Ubuntu:

```bash
./isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-V1 \
  --num_envs 1 \
  --steps 8 \
  --seed 42 \
  --disable-observation-corruption \
  --headless
```

If you want the flex joint to sweep enough to expose wrap behavior, use a sine action:

```bash
./isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-V1 \
  --num_envs 1 \
  --steps 24 \
  --seed 42 \
  --disable-observation-corruption \
  --action-mode sine \
  --sine-amplitude 0.15 \
  --sine-frequency-hz 0.5 \
  --headless
```

If you want a machine-readable log for later comparison:

```bash
./isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-V1 \
  --num_envs 1 \
  --steps 24 \
  --seed 42 \
  --disable-observation-corruption \
  --action-mode sine \
  --jsonl logs/obs_contract_check.jsonl \
  --headless
```

## How to interpret the output

- `obs_dim` should stay at `15`.
- Each printed bucket should have exactly `3` values.
- With `--disable-observation-corruption`, `base_pos` and `flex_pos` buckets should align with the latest raw joint-relative values reported on the same step.
- If `flex_pos_raw` and `flex_pos_wrap` diverge once the joint moves across `+-pi`, then the script has confirmed the wrap boundary. At that point you can decide whether training and deployment should stay on raw relative angle or switch together to an explicit unwrap path.
