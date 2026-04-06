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
        _joint_index_cache[asset_id] = {}
    if "fixed" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["fixed"] = asset.find_joints(fixed_pole_joint)[0][0]
    if "flex" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["flex"] = asset.find_joints(flex_pole_joint)[0][0]
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


def _get_effective_action_delta_pair(
    env: ManagerBasedRLEnv,
    action_name: str = "motor_pos",
) -> tuple[torch.Tensor, torch.Tensor]:
    """读取当前步和上一步“真正执行的动作增量”。

    旧任务里很多 reward 默认拿的是 raw action。
    新训练合同里我们明确改成：reward 和 last_action 都围绕真实执行 delta 建模。
    """

    term = env.action_manager.get_term(action_name)
    if hasattr(term, "processed_actions") and hasattr(term, "prev_applied_actions"):
        return term.processed_actions, term.prev_applied_actions
    return env.action_manager.action, env.action_manager.prev_action


def _compute_upright_metrics(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """计算近直立控制最关心的四个量。

    返回值依次为：
    - fixed 杆绝对角
    - 活动杆尖端绝对角
    - fixed 杆角速度
    - 活动杆尖端角速度
    """

    asset: Articulation = env.scene[asset_cfg.name]

    asset_id = id(asset)
    if asset_id not in _joint_index_cache:
        _joint_index_cache[asset_id] = {}
    if "fixed" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["fixed"] = asset.find_joints(fixed_pole_joint)[0][0]
    if "flex" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["flex"] = asset.find_joints(flex_pole_joint)[0][0]
    indices = _joint_index_cache[asset_id]

    fixed_pos = asset.data.joint_pos[:, indices["fixed"]]
    flex_pos = asset.data.joint_pos[:, indices["flex"]]
    fixed_vel = asset.data.joint_vel[:, indices["fixed"]]
    flex_vel = asset.data.joint_vel[:, indices["flex"]]

    base_abs = wrap_to_pi(fixed_pos)
    tip_abs = wrap_to_pi(fixed_pos + flex_pos)
    tip_vel = fixed_vel + flex_vel

    return base_abs, tip_abs, fixed_vel, tip_vel


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

    current_delta, previous_delta = _get_effective_action_delta_pair(env)
    action_rate_penalty = torch.sum(torch.square(current_delta - previous_delta), dim=1)

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

    current_delta, _ = _get_effective_action_delta_pair(env)
    action_penalty = torch.sum(torch.square(current_delta), dim=1)

    return weight_multiplier * action_penalty


def rk_upright_hold_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    base_angle_window: float = 0.12,
    tip_angle_window: float = 0.12,
    base_vel_window: float = 1.2,
    tip_vel_window: float = 1.5,
) -> torch.Tensor:
    """在近直立、低速度时提供持续保持奖励。

    这项奖励的目的不是再奖励一次“对齐”，而是明确告诉策略：
    进入直立窗口之后，继续稳稳待在里面，本身就是高价值行为。
    """

    base_abs, tip_abs, base_vel, tip_vel = _compute_upright_metrics(
        env, asset_cfg, fixed_pole_joint, flex_pole_joint
    )

    hold_mask = (
        (torch.abs(base_abs) <= base_angle_window)
        & (torch.abs(tip_abs) <= tip_angle_window)
        & (torch.abs(base_vel) <= base_vel_window)
        & (torch.abs(tip_vel) <= tip_vel_window)
    )
    return hold_mask.to(dtype=torch.float32)


def rk_upright_hold_soft_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    base_angle_window: float = 0.12,
    tip_angle_window: float = 0.12,
    base_vel_window: float = 1.2,
    tip_vel_window: float = 1.5,
    angle_softness: float = 0.4,
    vel_softness: float = 0.4,
) -> torch.Tensor:
    """在近直立、低速度时提供平滑保持奖励。

    与二值 hold 奖励相比，本函数不会在阈值边缘发生 0/1 跳变，
    而是通过指数核平滑衰减，减少课程切换阶段的训练震荡。

    数学形式（每个并行环境一条样本）：

    设
    - b_a = |base_abs|, t_a = |tip_abs|
    - b_v = |base_vel|, t_v = |tip_vel|
    - w_ba, w_ta, w_bv, w_tv 分别是角度/速度窗口
    - s_a, s_v 分别是 angle_softness、vel_softness

    则四个子项分别为高斯核：
    - r_ba = exp(-(b_a / (w_ba * s_a))^2)
    - r_ta = exp(-(t_a / (w_ta * s_a))^2)
    - r_bv = exp(-(b_v / (w_bv * s_v))^2)
    - r_tv = exp(-(t_v / (w_tv * s_v))^2)

    为了避免“角度已经基本到位，但速度项把奖励整体压到接近 0”的问题，
    最终不再直接使用四项连乘，而是改成：
    - angle_score = sqrt(r_ba * r_ta)
    - vel_score = 0.5 * (r_bv + r_tv)
    - r = angle_score * (0.25 + 0.75 * vel_score)

    这样仍然要求角度先接近直立，但对速度的要求从“硬门控”改成“平滑调制”，
    让策略在靠近稳住窗口时更早收到正反馈。
    """

    base_abs, tip_abs, base_vel, tip_vel = _compute_upright_metrics(
        env, asset_cfg, fixed_pole_joint, flex_pole_joint
    )

    # 角度窗口和速度窗口经过 softness 缩放后，作为高斯核的“有效标准化尺度”。
    # softness 越小，曲线越尖锐（更像硬阈值）；softness 越大，曲线越平缓。
    base_angle_term = torch.exp(
        -torch.square(base_abs / (base_angle_window * angle_softness + 1e-6))
    )
    tip_angle_term = torch.exp(
        -torch.square(tip_abs / (tip_angle_window * angle_softness + 1e-6))
    )

    # 速度项同理：速度越接近 0，奖励越接近 1；速度偏大时按平方指数衰减。
    base_vel_term = torch.exp(
        -torch.square(base_vel / (base_vel_window * vel_softness + 1e-6))
    )
    tip_vel_term = torch.exp(
        -torch.square(tip_vel / (tip_vel_window * vel_softness + 1e-6))
    )

    # 角度仍然是主要门控项，但不再让速度项把总奖励直接压成接近 0。
    angle_score = torch.sqrt(base_angle_term * tip_angle_term)
    vel_score = 0.5 * (base_vel_term + tip_vel_term)
    return angle_score * (0.25 + 0.75 * vel_score)


def rk_near_upright_velocity_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    fixed_pole_joint: str,
    flex_pole_joint: str,
    base_angle_window: float = 0.20,
    tip_angle_window: float = 0.20,
) -> torch.Tensor:
    """在近直立区域单独加强速度惩罚。"""

    base_abs, tip_abs, base_vel, tip_vel = _compute_upright_metrics(
        env, asset_cfg, fixed_pole_joint, flex_pole_joint
    )

    near_mask = (
        (torch.abs(base_abs) <= base_angle_window)
        & (torch.abs(tip_abs) <= tip_angle_window)
    ).to(dtype=torch.float32)
    velocity_penalty = torch.square(base_vel) + torch.square(tip_vel)
    return near_mask * velocity_penalty


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
        _joint_index_cache[asset_id] = {}
    if "fixed" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["fixed"] = asset.find_joints(fixed_pole_joint)[0][0]
    if "flex" not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id]["flex"] = asset.find_joints(flex_pole_joint)[0][0]
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

def rk_get_joint_angle_error(
    env: ManagerBasedRLEnv,
    target_angle: float,
    asset_cfg: SceneEntityCfg,
    joint_name: str | None = None,
    joint_index: int | None = None,
) -> torch.Tensor:
    """
    计算指定关节的“绝对角度”与目标角度之间的最短角距离（绝对误差）。

    Args:
        env: RL环境实例
        joint_name: 关节名称（兼容旧接口）
        joint_index: 关节索引（推荐，新接口）
        target_angle: 目标角度（弧度）
        asset_cfg: 机器人资产配置

    Returns:
        torch.Tensor: 形状为 (N, 1) 的误差张量，值域为 [0, π]
    """
    if joint_index is None and joint_name is None:
        raise ValueError("Either joint_index or joint_name must be provided.")

    asset: Articulation = env.scene[asset_cfg.name]

    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    joint_pos = torch.nan_to_num(joint_pos, nan=0.0, posinf=0.0, neginf=0.0)
    joint_pos = torch.clamp(joint_pos, -4 * torch.pi, 4 * torch.pi)
    abs_angles = torch.cumsum(joint_pos, dim=-1)

    if joint_index is None:
        asset_id = id(asset)
        if asset_id not in _joint_index_cache:
            _joint_index_cache[asset_id] = {}
        if joint_name not in _joint_index_cache[asset_id]:
            _joint_index_cache[asset_id][joint_name] = asset.find_joints(joint_name)[0][0]

        target_joint_id = _joint_index_cache[asset_id][joint_name]
        joint_ids_list = list(asset_cfg.joint_ids)
        if target_joint_id not in joint_ids_list:
            raise ValueError(
                f"joint_name={joint_name} (id={target_joint_id}) not found in asset_cfg.joint_ids={joint_ids_list}."
            )
        joint_index = joint_ids_list.index(target_joint_id)

    pole_abs_angle = abs_angles[:, joint_index]

    error = torch.abs(wrap_to_pi(pole_abs_angle - target_angle))
    error = torch.clamp(error, min=0.0, max=torch.pi)
    return error.unsqueeze(-1)

def rk_joint_angle_alignment_reward(
    env: ManagerBasedRLEnv,
    linear_weight: float,
    exp_weight: float,
    target_angle: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_name: str | None = None,
    joint_index: int | None = None,
) -> torch.Tensor:
    """
    基于单个关节相对角度的对齐奖励。
    结合了线性奖励和指数奖励，使得网络在远离目标时有固定梯度，接近目标时有更强的微调激励。

    Args:
        env: RL环境实例
        linear_weight: 线性奖励的权重比例
        exp_weight: 指数奖励的权重比例
        joint_name: 关节名称（兼容旧接口）
        joint_index: 关节索引（推荐，新接口）
        target_angle: 目标角度（弧度）
        asset_cfg: 机器人资产配置
    """
    error = rk_get_joint_angle_error(
        env=env,
        target_angle=target_angle,
        asset_cfg=asset_cfg,
        joint_name=joint_name,
        joint_index=joint_index,
    )  # (N, 1) 范围：[0, π]

    # 线性奖励部分：误差越大奖励越小，范围 [0, 1]
    linear_reward = (torch.pi - error) / torch.pi 
    # 指数奖励部分：在靠近0误差时会产生强烈的尖峰，范围 (0, 1]
    exp_reward = torch.exp(-1.0 * error) 

    # 组合两部分奖励
    combined_reward = linear_weight * linear_reward + exp_weight * exp_reward
    
    # 过滤 NaN 或无穷大值
    combined_reward = torch.nan_to_num(combined_reward, nan=0.0, posinf=0.0, neginf=0.0)

    return combined_reward.squeeze(-1) # 返回形状 (N,)




def rk_joint_angle_timeout_reward(
    env: ManagerBasedRLEnv,
    linear_weight: float,
    exp_weight: float,
    target_angle: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """
    终止时刻奖励（角度版本）。
    只在 episode 超时时，对所有杆的角度接近程度求平均奖励。
    所有杆使用同一个 target_angle。
    """
    time_outs = env.termination_manager.time_outs
    if not torch.any(time_outs):
        return torch.zeros_like(time_outs, dtype=torch.float32)

    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    # NaN + 极端值保护
    joint_pos = torch.nan_to_num(joint_pos, nan=0.0, posinf=0.0, neginf=0.0)
    joint_pos = torch.clamp(joint_pos, -4 * torch.pi, 4 * torch.pi)
    abs_angles = torch.cumsum(joint_pos, dim=-1)  # (N, num_joints)

    angle_errors = torch.abs(wrap_to_pi(abs_angles - target_angle))
    angle_errors = torch.clamp(angle_errors, 0.0, torch.pi)

    linear_rewards = (torch.pi - angle_errors) / torch.pi
    exp_rewards = torch.exp(-1.0 * angle_errors)

    combined = linear_weight * linear_rewards + exp_weight * exp_rewards
    avg_reward = torch.mean(combined, dim=-1)  # (N,)
    # NaN 保护
    avg_reward = torch.nan_to_num(avg_reward, nan=0.0, posinf=0.0, neginf=0.0)

    return torch.where(time_outs, avg_reward, torch.zeros_like(avg_reward))
