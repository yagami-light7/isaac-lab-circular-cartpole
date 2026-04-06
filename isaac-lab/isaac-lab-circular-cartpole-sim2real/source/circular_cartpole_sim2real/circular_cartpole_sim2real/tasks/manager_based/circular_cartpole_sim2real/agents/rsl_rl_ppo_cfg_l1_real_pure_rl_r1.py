# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """REAL-PURE-RL-R1 的 PPO 配置。

    这版用于 checkpoint 续训微调：
    - 保持网络结构不变，避免引入额外分布偏移
    - 适度降低学习率和熵系数，优先提升近直立稳态而非继续大幅探索
    """

    num_steps_per_env = 32
    max_iterations = 10000
    save_interval = 100
    experiment_name = "cartpole_s2r_l1_real_pure_rl_r1"
    clip_actions = 5.0
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.25,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.001,
        num_learning_epochs=5,
        num_mini_batches=8,
        learning_rate=7e-5,
        schedule="fixed",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=1.0,
    )
