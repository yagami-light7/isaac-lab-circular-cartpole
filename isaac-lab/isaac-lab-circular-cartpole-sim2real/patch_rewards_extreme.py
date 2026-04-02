import re

with open("/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "r") as f:
    content = f.read()

# Make fixed_alignment weight 0.1 (almost zero, just for drift correction)
content = re.sub(r'(fixed_alignment = RewTerm\([^)]*weight=)[0-9.]+', r'\g<1>0.2', content, count=1)

# Make flex_alignment weight huge (10.0)
content = re.sub(r'(flex_alignment = RewTerm\([^)]*weight=)[0-9.]+', r'\g<1>15.0', content, count=1)

# Drop pos_reward to 0 to prevent 0-gradient valleys
content = re.sub(r'(pos_reward = RewTerm\([^)]*weight=)[0-9.]+', r'\g<1>0.0', content, count=1)

# Drop timeout to 0
content = re.sub(r'(timeout_alignment = RewTerm\([^)]*weight=)[0-9.]+', r'\g<1>0.0', content, count=1)

with open("/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "w") as f:
    f.write(content)
