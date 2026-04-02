import re

# 1. Append the new function to mdp/rewards.py
with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/rewards.py", "r") as f:
    rewards_content = f.read()

new_func = """
def rk_joint_angle_timeout_reward(
    env: ManagerBasedRLEnv,
    linear_weight: float,
    exp_weight: float,
    target_angle: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    \"\"\"
    终止时刻奖励（角度版本）。
    只在 episode 超时时，对所有杆的角度接近程度求平均奖励。
    所有杆使用同一个 target_angle。
    \"\"\"
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
"""

if "def rk_joint_angle_timeout_reward" not in rewards_content:
    with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/rewards.py", "a") as f:
        f.write("\n" + new_func + "\n")

# 2. Update s2r_task_l1_v0.py
with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "r") as f:
    task_content = f.read()

# We need to replace the timeout_alignment RewTerm.
old_timeout_pattern = r"timeout_alignment\s*=\s*RewTerm\([^)]*func=mdp\.rk_timeout_alignment_pos_reward.*?\}\s*,\s*\n\s*\)"

new_timeout_term = """timeout_alignment = RewTerm(
        func=mdp.rk_joint_angle_timeout_reward,
        weight=5.0,  # 给一个初始的权重，让这个生存奖励生效
        params={
            "target_angle": flex_1_pole_target,  # 所有的杆最后都朝向这个绝对角度
            "asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_fixed", "fixed_to_flex_1"]),
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    )"""

task_content_new = re.sub(old_timeout_pattern, new_timeout_term, task_content, flags=re.DOTALL)

with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "w") as f:
    f.write(task_content_new)

print("Patch applied successfully for timeout reward.")
