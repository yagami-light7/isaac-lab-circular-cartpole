from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .observation_contract import (
    POLICY_FRAME_DIM,
    POLICY_HISTORY_LENGTH,
    POLICY_TERM_FIELD_NAMES,
    build_policy_frame,
    flatten_history_term_major,
)
from .protocol import ActionFrameV1, SensorFrameV1


HISTORY_TERM_NAMES = POLICY_TERM_FIELD_NAMES
DEFAULT_FEATURE_DIM = POLICY_FRAME_DIM


def build_history_frame(
    sensor: SensorFrameV1,
    last_action: ActionFrameV1 | None = None,
) -> list[float]:
    """Build one timestep of host-side policy features.

    The order is fixed to match the runtime contract:
    base_pos, flex1_pos, base_vel, flex1_vel, last_action.
    """

    if last_action is None:
        last_action_delta = 0.0
    else:
        last_action_delta = float(last_action.base_pos_delta_rad)

    return build_policy_frame(
        base_pos=float(sensor.base_pos_rad),
        flex_pos=float(sensor.flex1_pos_rad),
        base_vel=float(sensor.base_vel_rad_s),
        flex_vel=float(sensor.flex1_vel_rad_s),
        last_action=last_action_delta,
    )


default_observation_features = build_history_frame


@dataclass(slots=True)
class ObservationHistory:
    frame_dim: int = DEFAULT_FEATURE_DIM
    history_length: int = POLICY_HISTORY_LENGTH
    pad_value: float = 0.0
    _frames: deque[list[float]] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if self.frame_dim <= 0:
            raise ValueError("frame_dim must be positive")
        if self.history_length <= 0:
            raise ValueError("history_length must be positive")
        self._frames = deque(maxlen=self.history_length)

    def reset(self) -> None:
        self._frames.clear()

    def push(self, frame: Sequence[float] | Iterable[float]) -> None:
        values = [float(value) for value in frame]
        if len(values) != self.frame_dim:
            raise ValueError(
                f"expected frame_dim={self.frame_dim}, received {len(values)} values"
            )
        self._frames.append(values)

    def flatten(self) -> list[float]:
        return flatten_history_term_major(
            list(self._frames),
            history_length=self.history_length,
            frame_dim=self.frame_dim,
            pad_value=self.pad_value,
        )

    @property
    def is_full(self) -> bool:
        return len(self._frames) == self.history_length

    def __len__(self) -> int:
        return len(self._frames)
