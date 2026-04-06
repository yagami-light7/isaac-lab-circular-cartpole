from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ActionTermCfg
from isaaclab.managers.action_manager import ActionTerm
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedEnv


class SlewedRelativeJointPositionAction(ActionTerm):
    """带显式限幅和限速的相对关节位置动作。

    设计目标很直接：把 Windows 实机部署链中真正执行的动作合同
    直接搬进训练，而不是让策略先输出“大跳变”，再在部署末端做补救。

    这条动作链分成三步：
    1. 策略输出 raw action；
    2. 乘以 ``scale`` 得到目标位置增量 ``delta``；
    3. 对 ``delta`` 先做硬限幅，再做单一 ``slew-limit``；
    4. 最终把 ``current_joint_pos + delta`` 作为新的关节目标。

    类里额外保存了两份历史：
    - ``processed_actions``: 当前步真正执行的 delta
    - ``prev_applied_actions``: 上一控制步真正执行的 delta

    这样 observation 和 reward 就可以直接围绕“真实执行动作”建模，
    而不是继续使用未经限幅/限速的 raw action。
    """

    cfg: SlewedRelativeJointPositionActionCfg
    _asset: Articulation

    def __init__(self, cfg: SlewedRelativeJointPositionActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)

        joint_ids, joint_names = self._asset.find_joints(cfg.joint_names, preserve_order=cfg.preserve_order)
        self._joint_ids = joint_ids
        self._joint_names = joint_names
        self._num_joints = len(joint_ids)

        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._prev_applied_actions = torch.zeros_like(self._raw_actions)

        self._scale = float(cfg.scale)
        self._slew_limit = None if cfg.slew_rate_limit is None else abs(float(cfg.slew_rate_limit))
        self._delta_clip_min = float(cfg.delta_clip[0])
        self._delta_clip_max = float(cfg.delta_clip[1])

    @property
    def action_dim(self) -> int:
        return self._num_joints

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def prev_applied_actions(self) -> torch.Tensor:
        """上一控制步真正执行的 delta。

        训练侧的 last_action 观测和动作惩罚应该围绕这个量，而不是 raw action。
        """

        return self._prev_applied_actions

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0
        self._processed_actions[env_ids] = 0.0
        self._prev_applied_actions[env_ids] = 0.0

    def process_actions(self, actions: torch.Tensor) -> None:
        """把策略输出映射成真实会执行的关节增量。"""

        self._raw_actions[:] = actions

        # 先把 raw action 变成物理量：单位是 rad 的位置增量。
        delta = self._raw_actions * self._scale

        # 再做硬限幅，模拟部署侧 action-limit=0.45 rad。
        delta = torch.clamp(delta, min=self._delta_clip_min, max=self._delta_clip_max)

        # 最后做单一限速，模拟部署侧 action-slew-limit=0.02 rad/step。
        if self._slew_limit is not None:
            delta = torch.clamp(
                delta,
                min=self._processed_actions - self._slew_limit,
                max=self._processed_actions + self._slew_limit,
            )

        self._prev_applied_actions[:] = self._processed_actions
        self._processed_actions[:] = delta

    def apply_actions(self) -> None:
        """把真实执行的 delta 叠加到当前关节位置上。"""

        current_joint_pos = self._asset.data.joint_pos[:, self._joint_ids]
        target_joint_pos = current_joint_pos + self._processed_actions
        self._asset.set_joint_position_target(target_joint_pos, joint_ids=self._joint_ids)


@configclass
class SlewedRelativeJointPositionActionCfg(ActionTermCfg):
    """自定义动作项配置。

    这里只保留本项目真正需要的字段，故意不做过度泛化：
    - 单个或少量关节的相对位置增量控制
    - 固定 scale
    - 固定硬限幅
    - 固定单一 slew-limit
    """

    class_type: type = SlewedRelativeJointPositionAction
    asset_name: str = "robot"
    joint_names: tuple[str, ...] = ("base_to_fixed",)
    scale: float = 0.05
    delta_clip: tuple[float, float] = (-0.45, 0.45)
    slew_rate_limit: float | None = 0.02
    preserve_order: bool = False
