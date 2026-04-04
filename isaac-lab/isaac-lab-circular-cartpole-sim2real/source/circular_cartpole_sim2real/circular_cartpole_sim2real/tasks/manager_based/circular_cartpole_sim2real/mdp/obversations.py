from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

# MT6701 magnetic encoder: 14-bit resolution, 16384 counts per full revolution
_MT6701_LSB = 2.0 * math.pi / 16384  # rad per count (~3.835e-4 rad)
_MT6701_RATE_HZ = 200.0  # encoder read rate used for velocity estimation
_POLICY_RATE_HZ = 250.0
_POLICY_DT_S = 1.0 / _POLICY_RATE_HZ
_ENCODER_DT_S = 1.0 / _MT6701_RATE_HZ
_MT6701_VEL_EMA_ALPHA = 0.65

from isaaclab.utils.math import wrap_to_pi
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers.manager_base import ManagerTermBase
from isaaclab.managers.manager_term_cfg import ObservationTermCfg
from isaaclab.sensors import Camera, Imu, RayCaster, RayCasterCamera, TiledCamera

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv

from isaaclab.envs.utils.io_descriptors import (
    generic_io_descriptor,
    record_dtype,
    record_joint_names,
    record_joint_pos_offsets,
    record_joint_vel_offsets,
    record_shape,
)


@generic_io_descriptor(
    observation_type="JointState",
    on_inspect=[
        record_joint_names,
        record_dtype,
        record_shape,
        record_joint_pos_offsets,
    ],
    units="rad",
)
def rk_joint_pos_rel(
    env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """The joint positions of the asset w.r.t. the default joint positions.

    Replicates MT6701 encoder noise: position is quantized to 14-bit resolution
    (1 LSB = 2π/16384 rad) and may randomly read the adjacent count level (±1 LSB).
    The sampled jitter is stored on the env object so that rk_joint_vel_rel can derive
    correlated velocity noise as (jitter_current − jitter_previous) × LSB × rate.

    Note: Only the joints configured in :attr:`asset_cfg.joint_ids` will have their positions returned.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    pos = (
        asset.data.joint_pos[:, asset_cfg.joint_ids]
        - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    )
    # Quantize to nearest encoder count
    pos_quantized = torch.round(pos / _MT6701_LSB) * _MT6701_LSB
    # Random ±1 LSB jitter: encoder can read the current or an adjacent count level
    jitter = torch.randint(-1, 2, pos.shape, device=pos.device).float()
    # Store jitter for correlated velocity noise (roll previous → prev, current → curr)
    # Key includes joint IDs to avoid collision when the same asset has multiple ObsTerms
    _jid = (
        tuple(asset_cfg.joint_ids)
        if hasattr(asset_cfg.joint_ids, "__iter__")
        else (asset_cfg.joint_ids,)
    )
    _key_curr = f"_mt6701_jitter_{asset_cfg.name}_{_jid}_curr"
    _key_prev = f"_mt6701_jitter_{asset_cfg.name}_{_jid}_prev"
    if not hasattr(env, _key_curr):
        setattr(env, _key_prev, torch.zeros_like(jitter))
        setattr(env, _key_curr, torch.zeros_like(jitter))
    setattr(env, _key_prev, getattr(env, _key_curr))
    setattr(env, _key_curr, jitter)
    return pos_quantized + jitter * _MT6701_LSB


@generic_io_descriptor(
    observation_type="JointState",
    on_inspect=[
        record_joint_names,
        record_dtype,
        record_shape,
        record_joint_vel_offsets,
    ],
    units="rad/s",
)
def rk_joint_vel_rel(
    env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
):
    """The joint velocities of the asset w.r.t. the default joint velocities.

    Replicates MT6701 encoder-derived velocity noise. Velocity is estimated from
    consecutive position deltas at 200 Hz, so the noise is correlated with the position
    jitter stored by rk_joint_pos_rel:

        vel_noise = (jitter_current − jitter_previous) × LSB × rate

    This can produce spikes of up to ±2 LSB × 200 Hz ≈ ±0.153 rad/s when the position
    reading flips between adjacent counts on consecutive steps.

    Note: Only the joints configured in :attr:`asset_cfg.joint_ids` will have their velocities returned.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    vel = (
        asset.data.joint_vel[:, asset_cfg.joint_ids]
        - asset.data.default_joint_vel[:, asset_cfg.joint_ids]
    )
    # Derive velocity noise from the difference of correlated position jitter samples
    _jid = (
        tuple(asset_cfg.joint_ids)
        if hasattr(asset_cfg.joint_ids, "__iter__")
        else (asset_cfg.joint_ids,)
    )
    _key_curr = f"_mt6701_jitter_{asset_cfg.name}_{_jid}_curr"
    _key_prev = f"_mt6701_jitter_{asset_cfg.name}_{_jid}_prev"
    if not hasattr(env, _key_curr):
        return vel
    jitter_curr = getattr(env, _key_curr)
    jitter_prev = getattr(env, _key_prev)
    return vel + (jitter_curr - jitter_prev) * (_MT6701_LSB * _MT6701_RATE_HZ)


def _get_joint_state_rel(
    env: ManagerBasedEnv, asset_cfg: SceneEntityCfg
) -> tuple[Articulation, torch.Tensor, torch.Tensor]:
    """读取相对默认位姿的关节位置和速度。"""

    asset: Articulation = env.scene[asset_cfg.name]
    pos = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    vel = asset.data.joint_vel[:, asset_cfg.joint_ids] - asset.data.default_joint_vel[:, asset_cfg.joint_ids]
    return asset, pos, vel


def _ensure_env_tensor_state(
    env: ManagerBasedEnv,
    key: str,
    like: torch.Tensor,
    *,
    dtype: torch.dtype | None = None,
    fill_value: float = 0.0,
) -> torch.Tensor:
    """在 env 对象上创建或复用一个与给定 tensor 同形状的状态缓存。"""

    state = getattr(env, key, None)
    target_dtype = dtype if dtype is not None else like.dtype
    if (
        state is None
        or not isinstance(state, torch.Tensor)
        or state.shape != like.shape
        or state.device != like.device
        or state.dtype != target_dtype
    ):
        state = torch.full(like.shape, fill_value, device=like.device, dtype=target_dtype)
        setattr(env, key, state)
    return state


def _get_reset_mask(env: ManagerBasedEnv, reference: torch.Tensor) -> torch.Tensor:
    """返回哪些环境刚刚 reset。

    Isaac Lab 在 reset 后通常会把这些环境的 `episode_length_buf` 置为 0。
    这里利用这个行为，把 sample-and-hold 和 EMA 的内部状态也一起重置。
    """

    if hasattr(env, "episode_length_buf"):
        reset_mask = env.episode_length_buf == 0
        if reset_mask.ndim == 0:
            reset_mask = reset_mask.unsqueeze(0)
        return reset_mask.to(device=reference.device)
    return torch.zeros(reference.shape[0], dtype=torch.bool, device=reference.device)


def _update_wrapped_flex_encoder_model(
    env: ManagerBasedEnv,
    raw_pos: torch.Tensor,
    raw_vel: torch.Tensor,
    *,
    state_key: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    """模拟活动杆编码器的真实观测链。

    这里刻意不用简单的白噪声，而是模拟更贴近真机的结构化误差：
    1. 角度先 wrap 到 [-pi, pi]
    2. 角度量化到 MT6701 的 14-bit 分辨率
    3. 传感器只按 200 Hz 更新，控制仍按 250 Hz 读取，因此存在 sample-and-hold
    4. 角速度采用一阶 EMA，模拟板端估计链的滞后
    """

    wrapped_pos = wrap_to_pi(raw_pos)
    quantized_pos = torch.round(wrapped_pos / _MT6701_LSB) * _MT6701_LSB

    reset_mask = _get_reset_mask(env, quantized_pos)

    held_pos_key = f"{state_key}_held_pos"
    held_vel_key = f"{state_key}_held_vel"
    phase_key = f"{state_key}_sensor_phase"
    sample_idx_key = f"{state_key}_sample_idx"

    held_pos = _ensure_env_tensor_state(env, held_pos_key, quantized_pos)
    held_vel = _ensure_env_tensor_state(env, held_vel_key, raw_vel)
    sensor_phase = _ensure_env_tensor_state(env, phase_key, quantized_pos)
    last_sample_idx = _ensure_env_tensor_state(
        env, sample_idx_key, quantized_pos, dtype=torch.long, fill_value=-1.0
    )

    if torch.any(reset_mask):
        # 每个 env 在 reset 时重新抽一个编码器相位，避免所有环境都在完全相同的采样相位上。
        sensor_phase[reset_mask] = torch.rand_like(sensor_phase[reset_mask]) * _ENCODER_DT_S
        held_pos[reset_mask] = quantized_pos[reset_mask]
        held_vel[reset_mask] = raw_vel[reset_mask]
        last_sample_idx[reset_mask] = -1

    if hasattr(env, "episode_length_buf"):
        episode_time_s = env.episode_length_buf.to(dtype=quantized_pos.dtype, device=quantized_pos.device)
        episode_time_s = episode_time_s.unsqueeze(-1) * _POLICY_DT_S
    else:
        episode_time_s = torch.zeros_like(quantized_pos)

    sensor_sample_idx = torch.floor((episode_time_s + sensor_phase) / _ENCODER_DT_S).to(torch.long)
    should_update = reset_mask.unsqueeze(-1) | (sensor_sample_idx > last_sample_idx)

    if torch.any(should_update):
        held_pos[should_update] = quantized_pos[should_update]

        # 角速度只在“编码器出了新样本”时才更新，并通过 EMA 体现板端估计滞后。
        ema_updated_vel = held_vel + _MT6701_VEL_EMA_ALPHA * (raw_vel - held_vel)
        held_vel[should_update] = ema_updated_vel[should_update]
        last_sample_idx[should_update] = sensor_sample_idx[should_update]

    return held_pos, held_vel


def rk_wrapped_flex_joint_pos_rel(
    env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """活动杆位置观测：wrap 角 + 量化 + sample-and-hold。"""

    _, pos, vel = _get_joint_state_rel(env, asset_cfg)
    held_pos, _ = _update_wrapped_flex_encoder_model(
        env,
        raw_pos=pos,
        raw_vel=vel,
        state_key=f"_rk_wrapped_flex_obs_{asset_cfg.name}_{tuple(asset_cfg.joint_ids)}",
    )
    return held_pos


def rk_wrapped_flex_joint_vel_rel(
    env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """活动杆速度观测：200 Hz sample-and-hold 后的一阶 EMA 速度。"""

    _, pos, vel = _get_joint_state_rel(env, asset_cfg)
    _, held_vel = _update_wrapped_flex_encoder_model(
        env,
        raw_pos=pos,
        raw_vel=vel,
        state_key=f"_rk_wrapped_flex_obs_{asset_cfg.name}_{tuple(asset_cfg.joint_ids)}",
    )
    return held_vel


def rk_last_executed_action(
    env: ManagerBasedEnv,
    action_name: str = "motor_pos",
) -> torch.Tensor:
    """返回上一控制步真正执行的动作增量。

    旧训练合同里的 `last_action` 使用的是 raw action，这会让训练看到的闭环
    与部署后经过 action-limit 和 slew-limit 的真实闭环不一致。
    新合同直接把“上一拍真正执行的 delta（单位 rad）”送回策略。
    """

    term = env.action_manager.get_term(action_name)
    if hasattr(term, "prev_applied_actions"):
        return term.prev_applied_actions.clone()

    # 兜底分支：如果未来任务误用了旧 action term，至少不要直接报错。
    if hasattr(term, "processed_actions"):
        return term.processed_actions.clone()
    return env.action_manager.prev_action.clone()
