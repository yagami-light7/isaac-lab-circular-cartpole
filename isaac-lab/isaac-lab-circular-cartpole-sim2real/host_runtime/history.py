from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .protocol import ActionFrameV1, SensorFrameV1


HISTORY_TERM_NAMES = (
    "base_pos_rad",
    "flex1_pos_rad",
    "base_vel_rad_s",
    "flex1_vel_rad_s",
    "last_action_delta_rad",
)
DEFAULT_FEATURE_DIM = len(HISTORY_TERM_NAMES)


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

    return [
        float(sensor.base_pos_rad),
        float(sensor.flex1_pos_rad),
        float(sensor.base_vel_rad_s),
        float(sensor.flex1_vel_rad_s),
        last_action_delta,
    ]


default_observation_features = build_history_frame


@dataclass(slots=True)
class ObservationHistory:
    frame_dim: int = DEFAULT_FEATURE_DIM
    history_length: int = 3
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
        missing = self.history_length - len(self._frames)
        padded_frames = [
            [self.pad_value] * self.frame_dim for _ in range(missing)
        ] + list(self._frames)
        output: list[float] = []
        # Isaac Lab concatenates observation histories term-major:
        # [base_pos_hist, flex1_pos_hist, base_vel_hist, flex1_vel_hist, last_action_hist]
        for term_index in range(self.frame_dim):
            for frame in padded_frames:
                output.append(frame[term_index])
        return output

    @property
    def is_full(self) -> bool:
        return len(self._frames) == self.history_length

    def __len__(self) -> int:
        return len(self._frames)
