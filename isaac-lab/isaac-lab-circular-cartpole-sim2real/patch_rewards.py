import re

with open("/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "r") as f:
    content = f.read()

# Make fixed_alignment weight 1.0
content = re.sub(r'(fixed_alignment = RewTerm\([^)]*weight=)10\.0', r'\g<1>2.0', content, count=1)

# Make flex_alignment weight 1.0
content = re.sub(r'(flex_alignment = RewTerm\([^)]*weight=)10\.0', r'\g<1>2.0', content, count=1)

# Modify pos_reward
content = re.sub(r'(pos_reward = RewTerm\([^)]*weight=)10\.0', r'\g<1>25.0', content, count=1)
content = re.sub(r'("additive_weight": )0\.4', r'\g<1>0.0', content)
content = re.sub(r'("multiplicative_weight": )0\.6', r'\g<1>1.0', content)

with open("/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v0.py", "w") as f:
    f.write(content)
