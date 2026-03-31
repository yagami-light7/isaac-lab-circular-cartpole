from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers.command_manager import CommandTerm


def rk_time_out(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate the episode when the episode length exceeds a randomized episode length.

    Uses a random timeout between 50% and 100% of max_episode_length for each environment.
    """
    if not hasattr(env, "_rk_timeout_lengths"):
        # Initialize random timeout lengths for each environment (50% to 100% of max length)
        env._rk_timeout_lengths = torch.randint(
            low=env.max_episode_length // 2,
            high=env.max_episode_length + 1,
            size=(env.num_envs,),
            device=env.device,
        )

    # Reset timeout for environments that just reset
    reset_env_ids = (env.episode_length_buf == 0).nonzero(as_tuple=False).squeeze(-1)
    if len(reset_env_ids) > 0:
        env._rk_timeout_lengths[reset_env_ids] = torch.randint(
            low=env.max_episode_length // 2,
            high=env.max_episode_length + 1,
            size=(len(reset_env_ids),),
            device=env.device,
        )
    return env.episode_length_buf >= env._rk_timeout_lengths
