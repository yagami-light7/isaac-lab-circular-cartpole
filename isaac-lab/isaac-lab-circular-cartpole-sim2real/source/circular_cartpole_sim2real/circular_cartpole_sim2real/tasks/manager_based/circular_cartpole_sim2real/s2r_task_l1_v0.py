# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import ActionTermCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg

from . import mdp

##
# Pre-defined configs
##

from .s2r_config import CIRCULAR_CARTPOLE_S2R_ROBOT_L1_CFG

fixed_pole_target = 0.0
flex_1_pole_target = 0.0
pos_clip_range = (-2 * math.pi, 2 * math.pi)
pos_scale = 0.1

##
# Scene definition
##


@configclass
class CircularCartpoleSim2realSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = CIRCULAR_CARTPOLE_S2R_ROBOT_L1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=700.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    motor_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["base_to_fixed"],
        scale=pos_scale,
        clip={"base_to_fixed": pos_clip_range},
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        history_length = 3

        based_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            clip=(-2 * math.pi, 2 * math.pi),
            noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"])
            },
        )
        base_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"])
            },
        )
        flex_1_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            clip=(-2 * math.pi, 2 * math.pi),
            noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"])
            },
        )
        flex_1_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"])
            },
        )
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_fixed_joint_velocity = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"]),
            "position_range": (-3.0, 3.0),
            "velocity_range": (-10.0, 10.0),
        },
    )

    reset_flex_1_joint_velocity = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"]),
            "position_range": (-6.0, 6.0),
            "velocity_range": (-10.0, 10.0),
        },
    )

    # startup
    randomize_rigid_body_mass_fixed = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["fixed_pole"]),
            "mass_distribution_params": (-0.14, -0.07),
            "operation": "add",
            "recompute_inertia": True,
        },
    )

    randomize_rigid_body_mass_flex_1 = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["flex_pole_1"]),
            "mass_distribution_params": (-0.14, -0.07),
            "operation": "add",
            "recompute_inertia": True,
        },
    )

    randomize_fixed_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["fixed_pole"]),
            "com_range": {"x": (0.0, 0.0), "y": (-0.02, 0.02), "z": (0.0, 0.0)},
        },
    )

    randomize_flex_1_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["flex_pole_1"]),
            "com_range": {"x": (0.0, 0.0), "y": (-0.02, 0.02), "z": (0.0, 0.0)},
        },
    )

    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"]),
            "stiffness_distribution_params": (0.5, 2.0),
            "damping_distribution_params": (0.5, 2.0),
            "operation": "scale",
            "distribution": "uniform",
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    fixed_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"])},
    )
    flex_1_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"])},
    )
    pos_reward = RewTerm(
        func=mdp.rk_alignment_pos_reward,
        weight=1.0,
        params={
            "flex_target": flex_1_pole_target,
            "fixed_target": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "fixed_pole_joint": "base_to_fixed",
            "flex_pole_joint": "fixed_to_flex_1",
            "additive_weight": 4.0,
            "multiplicative_weight": 6.0,
        },
    )
    action_smoothness = RewTerm(
        func=mdp.action_rate_l2,
        weight=-0.002,
    )


##
# Environment configuration
##


@configclass
class CircularCartpoleSim2realEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: CircularCartpoleSim2realSceneCfg = CircularCartpoleSim2realSceneCfg(
        num_envs=4096, env_spacing=1.5
    )
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 1
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 250
        self.sim.render_interval = self.decimation
