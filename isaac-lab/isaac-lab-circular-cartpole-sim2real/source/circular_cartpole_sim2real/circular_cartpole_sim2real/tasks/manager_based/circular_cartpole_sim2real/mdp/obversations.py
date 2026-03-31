from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

# MT6701 magnetic encoder: 14-bit resolution, 16384 counts per full revolution
_MT6701_LSB = 2.0 * math.pi / 16384  # rad per count (~3.835e-4 rad)
_MT6701_RATE_HZ = 200.0  # encoder read rate used for velocity estimation

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
