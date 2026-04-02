import re

with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/rewards.py", "r") as f:
    content = f.read()

new_funcs = """
def rk_get_joint_angle_error(
    env: ManagerBasedRLEnv,
    joint_name: str,
    target_angle: float,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    
    asset_id = id(asset)
    if asset_id not in _joint_index_cache:
        _joint_index_cache[asset_id] = {}
    if joint_name not in _joint_index_cache[asset_id]:
        _joint_index_cache[asset_id][joint_name] = asset.find_joints(joint_name)[0][0]
    
    joint_idx = _joint_index_cache[asset_id][joint_name]
    # Get raw joint position
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, joint_idx])
    
    # Calculate shortest angular distance (absolute error)
    error = torch.abs(wrap_to_pi(joint_pos - target_angle)) # (N,) range [0, pi]
    return error.unsqueeze(-1) # (N, 1)

def rk_joint_angle_alignment_reward(
    env: ManagerBasedRLEnv,
    linear_weight: float,
    exp_weight: float,
    joint_name: str,
    target_angle: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    error = rk_get_joint_angle_error(env, joint_name, target_angle, asset_cfg) # (N, 1) 范围：[0, π]

    linear_reward = (torch.pi - error) / torch.pi # 范围[0,1]
    exp_reward = torch.exp(-1.0 * error) # 范围(0,1]

    combined_reward = linear_weight * linear_reward + exp_weight * exp_reward
    # NaN 保护
    combined_reward = torch.nan_to_num(combined_reward, nan=0.0, posinf=0.0, neginf=0.0)

    return combined_reward.squeeze(-1) # 返回形状 (N,)

"""

# add it to the end if not already there
if "def rk_joint_angle_alignment_reward" not in content:
    with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/rewards.py", "a") as f:
        f.write(new_funcs)

# Now update s2r_task_l1_v0.py
with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "r") as f:
    task_content = f.read()

task_content = re.sub(
    r"fixed_alignment\s*=\s*RewTerm\([^)]+func=mdp\.rk_fixed_joint_alignment[^)]+\)",
    r'''fixed_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=2.0,
        params={
            "target_angle": fixed_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "joint_name": "base_to_fixed",
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    )''',
    task_content,
    flags=re.DOTALL
)

task_content = re.sub(
    r"flex_alignment\s*=\s*RewTerm\([^)]+func=mdp\.rk_flex1_absolute_alignment[^)]+\)",
    r'''flex_alignment = RewTerm(
        func=mdp.rk_joint_angle_alignment_reward,
        weight=15.0,
        params={
            "target_angle": flex_1_pole_target,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "joint_name": "fixed_to_flex_1",
            "linear_weight": 0.3,
            "exp_weight": 0.7,
        },
    )''',
    task_content,
    flags=re.DOTALL
)

with open("source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "w") as f:
    f.write(task_content)

print("Patch applied successfully.")
