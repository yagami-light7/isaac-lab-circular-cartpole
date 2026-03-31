# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from math import pi
import isaaclab.sim as sim_utils
from isaaclab.actuators import (
    ImplicitActuatorCfg,
    DelayedPDActuatorCfg,
    DCMotorCfg,
    IdealPDActuatorCfg,
)
from isaaclab.assets import ArticulationCfg
import os


##
# Configuration
##

l1_urdf_path = os.path.join(
    os.getcwd(),
    "models\\S2R_Sim_L1.SLDASM\\urdf\\S2R_Sim_L1.SLDASM.urdf",
)


CIRCULAR_CARTPOLE_S2R_ROBOT_L1_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=True,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=l1_urdf_path,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0, damping=0
            )
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={"base_to_fixed": 0.0, "fixed_to_flex_1": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators={
        "fixed_pole_actuator": DelayedPDActuatorCfg(
            joint_names_expr=["base_to_fixed"],
            effort_limit=3.0,
            velocity_limit=10.0,
            stiffness=7.0,
            damping=0.7,
            friction=0.0001,
            min_delay=7,
            max_delay=8,
        ),
        "flex_pole_actuator": ImplicitActuatorCfg(
            joint_names_expr=["fixed_to_flex_1"],
            stiffness=0.0,
            damping=0.0001,
            friction=0.0001,
        ),
    },
)
