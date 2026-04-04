# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
from copy import deepcopy

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg

from . import mdp
from .s2r_config import CIRCULAR_CARTPOLE_S2R_ROBOT_L1_CFG
from .s2r_task_l1_v0 import TerminationsCfg


fixed_pole_target = 0.0
flex_1_pole_target = 0.0

# 这两个量直接对齐当前 Windows 实机部署合同。
ACTION_DELTA_LIMIT_RAD = 0.45
ACTION_SLEW_LIMIT_RAD = 0.02
ACTION_SCALE = 0.05


PURE_RL_REAL_R1_ROBOT_CFG = deepcopy(CIRCULAR_CARTPOLE_S2R_ROBOT_L1_CFG)
PURE_RL_REAL_R1_ROBOT_CFG.actuators["fixed_pole_actuator"].min_delay = 6
PURE_RL_REAL_R1_ROBOT_CFG.actuators["fixed_pole_actuator"].max_delay = 10


@configclass
class CircularCartpoleSim2realL1RealPureRlR1SceneCfg(InteractiveSceneCfg):
    """L1 REAL PURE-RL-R1 场景配置。

    命名说明：
    - REAL：面向实机部署链的任务；
    - PURE-RL：这是纯强化学习基线，不混入 LQR / Hybrid；
    - R1：表示第一轮重训(retrain round 1)，不是原项目里姿态版本的 V1。

    这版只做 pure RL 重训，不碰 LQR / Hybrid。
    目标是把“部署时真实存在的动作约束”和“近直立稳定目标”
    直接写进训练环境。
    """

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    robot: ArticulationCfg = PURE_RL_REAL_R1_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=700.0),
    )


@configclass
class ActionsCfg:
    """纯 RL REAL-PURE-RL-R1 的动作配置。

    核心变化只有一条：
    训练不再把 raw action 直接送进 RelativeJointPositionAction，
    而是显式经过：
        raw -> scale(0.05) -> clip(0.45) -> slew(0.02)
    这样策略在训练阶段就必须学会真实部署链的行为。
    """

    motor_pos = mdp.SlewedRelativeJointPositionActionCfg(
        asset_name="robot",
        joint_names=("base_to_fixed",),
        scale=ACTION_SCALE,
        delta_clip=(-ACTION_DELTA_LIMIT_RAD, ACTION_DELTA_LIMIT_RAD),
        slew_rate_limit=ACTION_SLEW_LIMIT_RAD,
    )


@configclass
class ObservationsCfg:
    """观测配置。

    这版刻意做了两项合同修正：
    1. 活动杆位置改成 wrap 角语义；
    2. last_action 改成“上一拍真实执行的 delta(rad)”。
    """

    @configclass
    class PolicyCfg(ObsGroup):
        history_length = 3

        based_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            clip=(-2 * math.pi, 2 * math.pi),
            noise=UniformNoiseCfg(n_min=-0.01, n_max=0.01),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"])},
        )
        base_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=UniformNoiseCfg(n_min=-0.01, n_max=0.01),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"])},
        )
        flex_1_joint_pos = ObsTerm(
            func=mdp.rk_wrapped_flex_joint_pos_rel,
            clip=(-math.pi, math.pi),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"])},
        )
        flex_1_joint_vel = ObsTerm(
            func=mdp.rk_wrapped_flex_joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"])},
        )
        last_action = ObsTerm(
            func=mdp.rk_last_executed_action,
            params={"action_name": "motor_pos"},
        )

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """环境 reset 和随机化配置。

    这版 curriculum 的主角是 staged reset：
    - 先学近直立稳定
    - 再学中等偏离恢复
    - 最后学全范围 swing-up
    """

    reset_fixed_joint = EventTerm(
        func=mdp.rk_reset_joints_by_stage,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"]),
            "stage1_position_range": (-0.35, 0.35),
            "stage1_velocity_range": (-1.0, 1.0),
            "stage2_position_range": (-1.2, 1.2),
            "stage2_velocity_range": (-3.0, 3.0),
            "stage3_position_range": (-3.0, 3.0),
            "stage3_velocity_range": (-6.0, 6.0),
            "stage1_end_iters": 1500,
            "stage2_end_iters": 3500,
            "steps_per_env_per_iter": 32,
        },
    )

    reset_flex_1_joint = EventTerm(
        func=mdp.rk_reset_joints_by_stage,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"]),
            "stage1_position_range": (-0.35, 0.35),
            "stage1_velocity_range": (-1.0, 1.0),
            "stage2_position_range": (-1.2, 1.2),
            "stage2_velocity_range": (-3.0, 3.0),
            "stage3_position_range": (-3.0, 3.0),
            "stage3_velocity_range": (-6.0, 6.0),
            "stage1_end_iters": 1500,
            "stage2_end_iters": 3500,
            "steps_per_env_per_iter": 32,
        },
    )

    randomize_rigid_body_mass_fixed = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["fixed_pole"]),
            "mass_distribution_params": (-0.12, -0.08),
            "operation": "add",
            "recompute_inertia": True,
        },
    )

    randomize_rigid_body_mass_flex_1 = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["flex_pole_1"]),
            "mass_distribution_params": (-0.12, -0.08),
            "operation": "add",
            "recompute_inertia": True,
        },
    )

    randomize_fixed_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["fixed_pole"]),
            "com_range": {"x": (0.0, 0.0), "y": (-0.01, 0.01), "z": (0.0, 0.0)},
        },
    )

    randomize_flex_1_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["flex_pole_1"]),
            "com_range": {"x": (0.0, 0.0), "y": (-0.01, 0.01), "z": (0.0, 0.0)},
        },
    )

    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"]),
            "stiffness_distribution_params": (0.8, 1.4),
            "damping_distribution_params": (0.8, 1.4),
            "operation": "scale",
            "distribution": "uniform",
        },
    )


@configclass
class RewardsCfg:
    """REAL-PURE-RL-R1 奖励函数。

    设计目标：
    - 仍然保留“角度对齐”主干，避免完全推翻旧任务
    - 明确奖励“稳住”和“小动作”
    - 弱化“只在 episode 末尾给高分”的终点导向
    """

    fixed_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=1.20,
        params={
            "target_angle": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed", "fixed_to_flex_1"]),
            "joint_index": 0,
            "linear_weight": 5.0,
            "exp_weight": 2.0,
        },
    )

    flex_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=1.90,
        params={
            "target_angle": flex_1_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed", "fixed_to_flex_1"]),
            "joint_index": 1,
            "linear_weight": 5.0,
            "exp_weight": 2.0,
        },
    )

    upright_hold = RewTerm(
        func=mdp.rk_upright_hold_reward,
        weight=2.60,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "fixed_pole_joint": "base_to_fixed",
            "flex_pole_joint": "fixed_to_flex_1",
            "base_angle_window": 0.12,
            "tip_angle_window": 0.12,
            "base_vel_window": 1.2,
            "tip_vel_window": 1.5,
        },
    )

    near_upright_velocity = RewTerm(
        func=mdp.rk_near_upright_velocity_l2,
        weight=-0.12,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "fixed_pole_joint": "base_to_fixed",
            "flex_pole_joint": "fixed_to_flex_1",
            "base_angle_window": 0.20,
            "tip_angle_window": 0.20,
        },
    )

    timeout_alignment = RewTerm(
        func=mdp.rk_joint_angle_timeout_reward,
        weight=3.00,
        params={
            "target_angle": flex_1_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed", "fixed_to_flex_1"]),
            "linear_weight": 8.0,
            "exp_weight": 3.0,
        },
    )

    timeout_vel_reward = RewTerm(
        func=mdp.rk_timeout_vel_reward,
        weight=-0.80,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed", "fixed_to_flex_1"]),
        },
    )

    fixed_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.010,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed"])},
    )

    flex_1_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.020,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["fixed_to_flex_1"])},
    )

    action_magnitude = RewTerm(
        func=mdp.rk_action_l2,
        weight=-0.020,
        params={
            "flex_target": flex_1_pole_target,
            "fixed_target": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "fixed_pole_joint": "base_to_fixed",
            "flex_pole_joint": "fixed_to_flex_1",
            "high_weight": 1.0,
            "low_weight": 0.25,
        },
    )

    action_smoothness = RewTerm(
        func=mdp.rk_action_rate_l2,
        weight=-0.060,
        params={
            "flex_target": flex_1_pole_target,
            "fixed_target": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "fixed_pole_joint": "base_to_fixed",
            "flex_pole_joint": "fixed_to_flex_1",
            "high_weight": 1.4,
            "low_weight": 0.3,
        },
    )

    flex_joint_limit_penalty = RewTerm(
        func=mdp.rk_flex1_joint_limit_penalty,
        weight=-0.05,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "flex_pole_joint": "fixed_to_flex_1",
        },
    )


@configclass
class CurriculumCfg:
    """辅助型 reward curriculum。

    主 curriculum 已经放进 reset-state。
    这里仅做少量权重微调，让任务从“先会稳住”逐步过渡到
    “稳住 + 小动作 + 不依赖终点奖励”。
    """

    stronger_action_smoothness = CurrTerm(
        func=mdp.modify_reward_weight_relative,
        params={
            "term_name": "action_smoothness",
            "weight": -0.085,
            "num_iters": 1500,
            "steps_per_env_per_iter": 32,
        },
    )

    stronger_action_magnitude = CurrTerm(
        func=mdp.modify_reward_weight_relative,
        params={
            "term_name": "action_magnitude",
            "weight": -0.030,
            "num_iters": 1500,
            "steps_per_env_per_iter": 32,
        },
    )

    stronger_hold_reward = CurrTerm(
        func=mdp.modify_reward_weight_relative,
        params={
            "term_name": "upright_hold",
            "weight": 3.20,
            "num_iters": 3500,
            "steps_per_env_per_iter": 32,
        },
    )

    stronger_near_upright_velocity_penalty = CurrTerm(
        func=mdp.modify_reward_weight_relative,
        params={
            "term_name": "near_upright_velocity",
            "weight": -0.18,
            "num_iters": 3500,
            "steps_per_env_per_iter": 32,
        },
    )

    weaker_timeout_alignment = CurrTerm(
        func=mdp.modify_reward_weight_relative,
        params={
            "term_name": "timeout_alignment",
            "weight": 1.50,
            "num_iters": 3500,
            "steps_per_env_per_iter": 32,
        },
    )


@configclass
class CircularCartpoleSim2realL1RealPureRlR1EnvCfg(ManagerBasedRLEnvCfg):
    scene: CircularCartpoleSim2realL1RealPureRlR1SceneCfg = CircularCartpoleSim2realL1RealPureRlR1SceneCfg(
        num_envs=4096, env_spacing=1.5
    )
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self) -> None:
        self.decimation = 1
        self.episode_length_s = 5
        self.viewer.eye = (0, -10.0, 3.0)
        self.sim.dt = 1 / 250
        self.sim.render_interval = self.decimation
