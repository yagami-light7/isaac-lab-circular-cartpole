# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass

from . import mdp
from .s2r_task_l1_v1 import (
    CircularCartpoleSim2realSceneCfg,
    EventCfg,
    ObservationsCfg,
    RewardsCfg,
    TerminationsCfg,
)


pos_clip_range = (-2 * math.pi, 2 * math.pi)
pos_scale = 0.1


@configclass
class ActionsCfg:
    """Action specifications for the real-hardware-aligned L1 task.

    This variant keeps the L1-V1 task, rewards, and randomization intact and only
    changes the action semantics to a relative joint-position command so the
    policy output maps cleanly onto the host->MC02 position-delta contract.
    """

    motor_pos = mdp.RelativeJointPositionActionCfg(
        asset_name="robot",
        joint_names=["base_to_fixed"],
        scale=pos_scale,
        use_zero_offset=True,
        clip={"base_to_fixed": pos_clip_range},
    )


@configclass
class CircularCartpoleSim2realL1RealV0EnvCfg(ManagerBasedRLEnvCfg):
    scene: CircularCartpoleSim2realSceneCfg = CircularCartpoleSim2realSceneCfg(
        num_envs=4096, env_spacing=1.5
    )
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 1
        self.episode_length_s = 5
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.sim.dt = 1 / 250
        self.sim.render_interval = self.decimation
