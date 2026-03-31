# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi
from ..mdp import joint_vel_l2

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

pi_square = torch.pi**2
_joint_index_cache = {}
_alignment_error_cache = {}
_last_cached_step = -1


def joint_pos_target_l2(
    env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)


def rk_timeout_pos_reward(
    env: ManagerBasedRLEnv,
    target: float,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Return joint position L2 distance from target when timeout occurs.

    The time_outs tensor is a boolean (num_envs,) that's converted to 0.0/1.0 in arithmetic.
    Returns the L2 distance only for environments that timed out, 0 otherwise.
    Use negative weight in config to penalize distance, positive to reward proximity.
    """
    time_outs = env.termination_manager.time_outs
    pos_l2 = joint_pos_target_l2(env, target, asset_cfg)
    return pos_l2 * time_outs


def rk_timeout_vel_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Return joint velocity L1 norm when timeout occurs.

    The time_outs tensor is a boolean (num_envs,) that's converted to 0.0/1.0 in arithmetic.
    Returns the L1 norm of joint velocities only for environments that timed out, 0 otherwise.
    Use negative weight in config to penalize high velocities at timeout.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    time_outs = env.termination_manager.time_outs
    joint_vel_l2_val = joint_vel_l2(env, asset_cfg)
    return joint_vel_l2_val * time_outs


def _compute_alignment_errors(
    env: ManagerBasedRLEnv,
    asset: Articulation,
    flex_target: float,
    fixed_target: float,
    fixed_pole_joint: str,
    flex_pole_joint: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Helper function to compute normalized alignment errors for both poles.

    Results are cached per step to avoid redundant calculations when multiple
    reward functions need the same alignment errors.

    Returns:
        Tuple of (fixed_pos_error_normalized, flex_pos_error_normalized)
    """
    global _last_cached_step

    # Create simplified cache key (env_id removed as there's only one env instance)
    current_step = env.common_step_counter
    cache_key = (current_step, flex_target, fixed_target)

    # Return cached result if available for this step
    if cache_key in _alignment_error_cache:
        return _alignment_error_cache[cache_key]

    # Compute joint indices (cached separately for reuse)
    asset_id = id(asset)
    if asset_id not in _joint_index_cache:
        _joint_index_cache[asset_id] = {
            "fixed": asset.find_joints(fixed_pole_joint)[0][0],
            "flex": asset.find_joints(flex_pole_joint)[0][0],
        }
    indices = _joint_index_cache[asset_id]

    # Get joint positions and wrap to [-pi, pi]
    fixed_joint_pos = wrap_to_pi(asset.data.joint_pos[:, indices["fixed"]])
    flex_joint_pos = wrap_to_pi(asset.data.joint_pos[:, indices["flex"]])

    # Calculate errors with wrapping for shortest angular distance
    fixed_pos_error = torch.square(wrap_to_pi(fixed_joint_pos - fixed_target))
    flex_pos_error = torch.square(wrap_to_pi(flex_joint_pos - flex_target))
    fixed_pos_error_normalized = torch.clamp(fixed_pos_error / pi_square, 0.0, 1.0)
    flex_pos_error_normalized = torch.clamp(flex_pos_error / pi_square, 0.0, 1.0)

    # Cache the result for this step
    result = (fixed_pos_error_normalized, flex_pos_error_normalized)
    _alignment_error_cache[cache_key] = result

    # Clean up old cache entries more efficiently - only when step changes
    if _last_cached_step != current_step and _last_cached_step >= 0:
        # Delete entries from 2+ steps ago
        old_key = (_last_cached_step, flex_target, fixed_target)
        _alignment_error_cache.pop(old_key, None)

    _last_cached_step = current_step

    return result


def rk_alignment_pos_reward(
    env: ManagerBasedRLEnv,
    flex_target: float,
    fixed_target: float,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    additive_weight: float = 5.0,
    multiplicative_weight: float = 5.0,
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]

    fixed_pos_error_normalized, flex_pos_error_normalized = _compute_alignment_errors(
        env, asset, flex_target, fixed_target, fixed_pole_joint, flex_pole_joint
    )

    additive = ((1 - flex_pos_error_normalized) + (1 - fixed_pos_error_normalized)) / 2
    multiplicative = (
        torch.exp((1 - flex_pos_error_normalized) * (1 - fixed_pos_error_normalized))
        - 1
    ) / (torch.e - 1)
    reward = additive_weight * additive + multiplicative_weight * multiplicative
    return reward


def rk_timeout_alignment_pos_reward(
    env: ManagerBasedRLEnv,
    flex_target: float,
    fixed_target: float,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    additive_weight: float = 5.0,
    multiplicative_weight: float = 5.0,
) -> torch.Tensor:
    """Return alignment position error when timeout occurs.

    Returns the sum of normalized position errors for both poles only when timeout occurs, 0 otherwise.
    Use negative weight in config to penalize misalignment at timeout.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    time_outs = env.termination_manager.time_outs

    fixed_pos_error_normalized, flex_pos_error_normalized = _compute_alignment_errors(
        env, asset, flex_target, fixed_target, fixed_pole_joint, flex_pole_joint
    )

    additive = ((1 - flex_pos_error_normalized) + (1 - fixed_pos_error_normalized)) / 2
    multiplicative = (
        torch.exp((1 - flex_pos_error_normalized) * (1 - fixed_pos_error_normalized))
        - 1
    ) / (torch.e - 1)
    reward = additive_weight * additive + multiplicative_weight * multiplicative
    return reward * time_outs


def rk_action_rate_l2(
    env: ManagerBasedRLEnv,
    flex_target: float,
    fixed_target: float,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    high_weight: float = 1.0,
    low_weight: float = 0.5,
) -> torch.Tensor:
    """Penalize the rate of change of the actions using L2 squared kernel.

    Weight is dynamically adjusted based on alignment errors:
    - Small alignment errors (well-aligned) → higher weight (smoother control)
    - Large alignment errors (misaligned) → lower weight (allow aggressive corrections)
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Compute alignment errors
    fixed_pos_error_normalized, flex_pos_error_normalized = _compute_alignment_errors(
        env, asset, flex_target, fixed_target, fixed_pole_joint, flex_pole_joint
    )

    # Average alignment error (0 = well-aligned, 1 = poorly aligned)
    alignment_error = (fixed_pos_error_normalized + flex_pos_error_normalized) / 2

    # Dynamic weight with exponential decay: high when aligned, low when misaligned
    decay_factor = torch.exp(-1.0 * alignment_error)
    weight_multiplier = low_weight + (high_weight - low_weight) * decay_factor

    # Compute action rate penalty
    action_rate_penalty = torch.sum(
        torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1
    )

    return weight_multiplier * action_rate_penalty


def rk_action_l2(
    env: ManagerBasedRLEnv,
    flex_target: float,
    fixed_target: float,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    high_weight: float = 1.0,
    low_weight: float = 0.5,
) -> torch.Tensor:
    """Penalize the actions using L2 squared kernel.

    Weight is dynamically adjusted based on alignment errors:
    - Small alignment errors (well-aligned) → higher weight (encourage smaller actions)
    - Large alignment errors (misaligned) → lower weight (allow larger corrective actions)
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Compute alignment errors
    fixed_pos_error_normalized, flex_pos_error_normalized = _compute_alignment_errors(
        env, asset, flex_target, fixed_target, fixed_pole_joint, flex_pole_joint
    )

    # Average alignment error (0 = well-aligned, 1 = poorly aligned)
    alignment_error = (fixed_pos_error_normalized + flex_pos_error_normalized) / 2

    # Dynamic weight with exponential decay: high when aligned, low when misaligned
    decay_factor = torch.exp(-1.0 * alignment_error)
    weight_multiplier = low_weight + (high_weight - low_weight) * decay_factor

    # Compute action penalty
    action_penalty = torch.sum(torch.square(env.action_manager.action), dim=1)

    return weight_multiplier * action_penalty


def rk_flex1_absolute_alignment(
    env: ManagerBasedRLEnv,
    target: float,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
) -> torch.Tensor:
    """Reward for absolute alignment of flex_1 in world coordinates.

    Since flex_1 is a child of the fixed pole, its absolute position is the sum
    of both joint positions. Both joints have range [-2π, 2π].

    Args:
        env: The RL environment
        target: Target absolute position for flex_1 (e.g., 0.0 for upright)
        asset_cfg: Configuration for the robot asset
        fixed_pole_joint: Name of the fixed pole joint
        flex_pole_joint: Name of the flex_1 pole joint

    Returns:
        Normalized alignment error (0 = perfectly aligned, 1 = maximally misaligned)
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Get or cache joint indices
    asset_id = id(asset)
    if asset_id not in _joint_index_cache:
        _joint_index_cache[asset_id] = {
            "fixed": asset.find_joints(fixed_pole_joint)[0][0],
            "flex": asset.find_joints(flex_pole_joint)[0][0],
        }
    indices = _joint_index_cache[asset_id]

    # Get joint positions
    fixed_joint_pos = asset.data.joint_pos[:, indices["fixed"]]
    flex_joint_pos = asset.data.joint_pos[:, indices["flex"]]

    # Compute absolute position of flex_1 (sum of both joints)
    flex1_absolute_pos = wrap_to_pi(fixed_joint_pos + flex_joint_pos)

    # Calculate error with wrapping for shortest angular distance
    pos_error = torch.square(wrap_to_pi(flex1_absolute_pos - target))
    pos_error_normalized = torch.clamp(pos_error / pi_square, 0.0, 1.0)

    # Return as reward (1 - error so that 1 is best, 0 is worst)
    return 1.0 - pos_error_normalized


def rk_fixed_joint_alignment(
    env: ManagerBasedRLEnv,
    target: float,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
) -> torch.Tensor:
    """Reward for alignment of the fixed pole joint to a target position.

    Args:
        env: The RL environment
        target: Target position for the fixed pole joint (e.g., 0.0 for upright)
        asset_cfg: Configuration for the robot asset
        fixed_pole_joint: Name of the fixed pole joint

    Returns:
        Normalized alignment reward (1 = perfectly aligned, 0 = maximally misaligned)
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Get or cache joint index
    asset_id = id(asset)
    if asset_id not in _joint_index_cache:
        _joint_index_cache[asset_id] = {}
    if "fixed" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["fixed"] = asset.find_joints(fixed_pole_joint)[0][
            0
        ]

    fixed_joint_idx = _joint_index_cache[asset_id]["fixed"]

    # Get joint position and wrap to [-pi, pi]
    fixed_joint_pos = wrap_to_pi(asset.data.joint_pos[:, fixed_joint_idx])

    # Calculate error with wrapping for shortest angular distance
    pos_error = torch.square(wrap_to_pi(fixed_joint_pos - target))
    pos_error_normalized = torch.clamp(pos_error / pi_square, 0.0, 1.0)

    # Return as reward (1 - error so that 1 is best, 0 is worst)
    return 1.0 - pos_error_normalized


def rk_flex1_joint_limit_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    flex_pole_joint: str,
) -> torch.Tensor:
    """Penalize when flex_1 joint position is outside [-pi, pi] range.

    This function returns a penalty value that increases as the joint position
    moves outside the [-pi, pi] range. The penalty is 0 when within limits.

    Args:
        env: The RL environment
        asset_cfg: Configuration for the robot asset
        flex_pole_joint: Name of the flex_1 pole joint

    Returns:
        Penalty value (0 = within limits, >0 = outside limits).
        Use with negative weight to penalize violations.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Get or cache joint index
    asset_id = id(asset)
    if asset_id not in _joint_index_cache:
        _joint_index_cache[asset_id] = {}
    if "flex" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["flex"] = asset.find_joints(flex_pole_joint)[0][0]

    flex_joint_idx = _joint_index_cache[asset_id]["flex"]

    # Get joint position (raw, not wrapped)
    flex_joint_pos = asset.data.joint_pos[:, flex_joint_idx]

    # Calculate how much the joint exceeds the [-pi, pi] limits
    # If within limits, violation is 0
    lower_violation = torch.clamp(-torch.pi - flex_joint_pos, min=0.0)
    upper_violation = torch.clamp(flex_joint_pos - torch.pi, min=0.0)

    # Total violation (squared to make penalty more significant)
    violation = torch.square(lower_violation + upper_violation)

    return violation
