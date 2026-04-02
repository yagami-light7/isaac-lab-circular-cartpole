from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.managers import CurriculumTermCfg, ManagerTermBase

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class modify_reward_weight_relative(ManagerTermBase):
    """Modify a reward term weight after a relative threshold from resume/start.

    This behaves like IsaacLab's modify_reward_weight, but the threshold is measured
    from the training step when this term is initialized. It supports both:
    - num_iters: threshold in training iterations (preferred)
    - num_steps: threshold in env-steps (backward compatible)

    In this project, env.common_step_counter increases approximately by
    `steps_per_env_per_iter` each RL iteration. Therefore num_iters is converted as:
    threshold_steps = num_iters * steps_per_env_per_iter.
    The rollout length can be passed via steps_per_env_per_iter (default: 32).
    """

    def __init__(self, cfg: CurriculumTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        term_name = cfg.params["term_name"]
        self._term_cfg = env.reward_manager.get_term_cfg(term_name)
        self._start_step = int(env.common_step_counter)

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        env_ids: Sequence[int],
        term_name: str,
        weight: float,
        num_iters: int | None = None,
        num_steps: int | None = None,
        steps_per_env_per_iter: int = 32,
    ) -> float:
        if num_iters is not None:
            threshold_steps = int(num_iters) * int(steps_per_env_per_iter)
        elif num_steps is not None:
            threshold_steps = int(num_steps)
        else:
            raise ValueError("Either 'num_iters' or 'num_steps' must be provided.")

        if (env.common_step_counter - self._start_step) > threshold_steps:
            self._term_cfg.weight = weight
            env.reward_manager.set_term_cfg(term_name, self._term_cfg)
        return self._term_cfg.weight
