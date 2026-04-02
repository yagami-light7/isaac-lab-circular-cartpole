import re

with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "r") as f:
    text = f.read()

# Define the bad fixed_alignment
bad_fixed = """    fixed_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=2.0,
        params={
            "target_angle": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "joint_name": "base_to_fixed",
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    ),
            "fixed_pole_joint": "base_to_fixed",
        },
    )"""

# Define the good fixed_alignment
good_fixed = """    fixed_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=2.0,
        params={
            "target_angle": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "joint_name": "base_to_fixed",
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    )"""

text = text.replace(bad_fixed, good_fixed)

# Define the bad flex_alignment
bad_flex = """    flex_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=15.0,
        params={
            "target_angle": flex_1_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "joint_name": "fixed_to_flex_1",
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    ),
            "fixed_pole_joint": "base_to_fixed",
            "flex_pole_joint": "fixed_to_flex_1",
        },
    )"""

# Define the good flex_alignment
good_flex = """    flex_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=15.0,
        params={
            "target_angle": flex_1_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "joint_name": "fixed_to_flex_1",
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    )"""

text = text.replace(bad_flex, good_flex)

with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "w") as f:
    f.write(text)

print("Fixed syntax in config.")
