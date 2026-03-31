# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Check observation semantics for the circular cartpole Isaac Lab tasks."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Inspect policy observation order and joint semantics in Isaac Lab."
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument("--task", type=str, default="RK-Circular-Cartpole-S2R-L1-V1")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=8)
parser.add_argument("--env-index", type=int, default=0)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument(
    "--action-mode",
    choices=("zero", "sine"),
    default="zero",
    help="Action pattern used while sampling observations.",
)
parser.add_argument("--sine-amplitude", type=float, default=0.15)
parser.add_argument("--sine-frequency-hz", type=float, default=0.5)
parser.add_argument(
    "--jsonl",
    type=str,
    default=None,
    help="Optional JSONL output path for offline inspection.",
)
parser.add_argument(
    "--disable-observation-corruption",
    action="store_true",
    default=False,
    help="Disable observation corruption/noise before creating the environment.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

import circular_cartpole_sim2real.tasks  # noqa: F401
from circular_cartpole_sim2real.observation_contract import (
    BASE_POSITION_SEMANTICS,
    CONTROL_DT_S,
    CONTROL_FREQUENCY_HZ,
    FLEX_POSITION_SEMANTICS,
    LAST_ACTION_SEMANTICS,
    OBSERVATION_CONTRACT_VERSION,
    POLICY_HISTORY_LAYOUT,
    POLICY_HISTORY_LENGTH,
    POLICY_INPUT_DIM,
    POLICY_TERM_NAMES,
    split_history_buckets,
)


def _wrap_to_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _normalize_scalar(value: Any) -> float:
    if hasattr(value, "item"):
        return float(value.item())
    return float(value)


def _extract_policy_observation(observation: Any) -> Any:
    if isinstance(observation, dict):
        if "policy" in observation:
            return observation["policy"]
        if len(observation) == 1:
            return next(iter(observation.values()))
    return observation


def _row_to_list(value: Any, row_index: int) -> list[float]:
    row = value[row_index]
    if hasattr(row, "detach"):
        row = row.detach().cpu()
    if hasattr(row, "tolist"):
        return [float(item) for item in row.tolist()]
    return [float(item) for item in row]


def _tensor_joint_value(tensor: Any, env_index: int, joint_index: int) -> float:
    value = tensor
    if hasattr(value, "detach"):
        value = value.detach()
    if value.ndim == 1:
        return _normalize_scalar(value[joint_index])
    return _normalize_scalar(value[env_index, joint_index])


def _normalize_index(value: Any) -> int:
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError("empty joint index result")
        return _normalize_index(value[0])
    if hasattr(value, "ndim") and getattr(value, "ndim") == 0:
        return int(value.item())
    if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
        try:
            if len(value) > 0:
                first = value[0]
                return _normalize_index(first)
        except TypeError:
            pass
    if hasattr(value, "item"):
        return int(value.item())
    return int(value)


def _find_joint_index(robot: Any, joint_name: str) -> int:
    if hasattr(robot, "find_joints"):
        result = robot.find_joints(joint_name)
        if isinstance(result, tuple) and result:
            return _normalize_index(result[0])
        return _normalize_index(result)

    joint_names = getattr(robot, "joint_names", None)
    if joint_names is None and hasattr(robot, "data"):
        joint_names = getattr(robot.data, "joint_names", None)
    if joint_names is None:
        raise AttributeError("Unable to determine joint name list from Isaac Lab robot")
    return int(joint_names.index(joint_name))


def _snapshot_robot(env: Any, env_index: int) -> dict[str, float]:
    robot = env.unwrapped.scene["robot"]
    base_joint_index = _find_joint_index(robot, "base_to_fixed")
    flex_joint_index = _find_joint_index(robot, "fixed_to_flex_1")

    base_pos = _tensor_joint_value(robot.data.joint_pos, env_index, base_joint_index) - _tensor_joint_value(
        robot.data.default_joint_pos, env_index, base_joint_index
    )
    flex_pos = _tensor_joint_value(robot.data.joint_pos, env_index, flex_joint_index) - _tensor_joint_value(
        robot.data.default_joint_pos, env_index, flex_joint_index
    )
    base_vel = _tensor_joint_value(robot.data.joint_vel, env_index, base_joint_index) - _tensor_joint_value(
        robot.data.default_joint_vel, env_index, base_joint_index
    )
    flex_vel = _tensor_joint_value(robot.data.joint_vel, env_index, flex_joint_index) - _tensor_joint_value(
        robot.data.default_joint_vel, env_index, flex_joint_index
    )

    return {
        "base_pos_raw": base_pos,
        "base_pos_wrap": _wrap_to_pi(base_pos),
        "flex_pos_raw": flex_pos,
        "flex_pos_wrap": _wrap_to_pi(flex_pos),
        "base_vel_raw": base_vel,
        "flex_vel_raw": flex_vel,
    }


def _make_actions(env: Any, step_index: int) -> torch.Tensor:
    if args_cli.action_mode == "zero":
        return torch.zeros(env.action_space.shape, device=env.unwrapped.device)

    value = float(args_cli.sine_amplitude) * math.sin(
        2.0 * math.pi * float(args_cli.sine_frequency_hz) * float(step_index) * CONTROL_DT_S
    )
    return torch.full(env.action_space.shape, fill_value=value, device=env.unwrapped.device)


def _build_record(step_index: int, observation: Any, env: Any) -> dict[str, Any]:
    policy_observation = _extract_policy_observation(observation)
    observation_row = _row_to_list(policy_observation, args_cli.env_index)
    buckets = split_history_buckets(observation_row)
    raw_state = _snapshot_robot(env, args_cli.env_index)

    return {
        "step": int(step_index),
        "task": args_cli.task,
        "contract_version": OBSERVATION_CONTRACT_VERSION,
        "control_frequency_hz": CONTROL_FREQUENCY_HZ,
        "history_length": POLICY_HISTORY_LENGTH,
        "policy_input_dim": len(observation_row),
        "history_layout": POLICY_HISTORY_LAYOUT,
        "term_names": list(POLICY_TERM_NAMES),
        "raw_state": raw_state,
        "policy_observation": observation_row,
        "policy_buckets": buckets,
    }


def _print_header() -> None:
    print(f"[contract] version={OBSERVATION_CONTRACT_VERSION}")
    print(
        "[contract] "
        f"task={args_cli.task} history={POLICY_HISTORY_LENGTH} dim={POLICY_INPUT_DIM} "
        f"layout={POLICY_HISTORY_LAYOUT}"
    )
    print(f"[contract] base_pos={BASE_POSITION_SEMANTICS}")
    print(f"[contract] flex_pos={FLEX_POSITION_SEMANTICS}")
    print(f"[contract] last_action={LAST_ACTION_SEMANTICS}")


def _print_record(record: dict[str, Any]) -> None:
    raw_state = record["raw_state"]
    print(
        "[step {step}] obs_dim={obs_dim} "
        "base_raw={base:+.6f} flex_raw={flex:+.6f} flex_wrap={flex_wrap:+.6f} "
        "base_vel={base_vel:+.6f} flex_vel={flex_vel:+.6f}".format(
            step=record["step"],
            obs_dim=record["policy_input_dim"],
            base=raw_state["base_pos_raw"],
            flex=raw_state["flex_pos_raw"],
            flex_wrap=raw_state["flex_pos_wrap"],
            base_vel=raw_state["base_vel_raw"],
            flex_vel=raw_state["flex_vel_raw"],
        )
    )
    for term_name in POLICY_TERM_NAMES:
        print(f"  {term_name}: {record['policy_buckets'][term_name]}")


def main() -> None:
    jsonl_path = Path(args_cli.jsonl) if args_cli.jsonl else None
    writer = None
    if jsonl_path is not None:
        jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        writer = jsonl_path.open("w", encoding="utf-8")

    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    env_cfg.seed = int(args_cli.seed)
    if (
        args_cli.disable_observation_corruption
        and hasattr(env_cfg, "observations")
        and hasattr(env_cfg.observations, "policy")
        and hasattr(env_cfg.observations.policy, "enable_corruption")
    ):
        env_cfg.observations.policy.enable_corruption = False
    env = gym.make(args_cli.task, cfg=env_cfg)

    try:
        _print_header()
        observation, _info = env.reset()
        for step_index in range(args_cli.steps):
            with torch.inference_mode():
                record = _build_record(step_index, observation, env)
                _print_record(record)
                if writer is not None:
                    writer.write(json.dumps(record, ensure_ascii=False) + "\n")
                    writer.flush()

                actions = _make_actions(env, step_index)
                observation, _reward, _terminated, _truncated, _info = env.step(actions)
    finally:
        if writer is not None:
            writer.close()
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
