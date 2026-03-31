from __future__ import annotations

from typing import Iterable, Sequence


OBSERVATION_CONTRACT_VERSION = "l1_policy_v1"
CONTROL_FREQUENCY_HZ = 250
CONTROL_DT_S = 1.0 / CONTROL_FREQUENCY_HZ

POLICY_HISTORY_LENGTH = 3
POLICY_TERM_NAMES = (
    "base_pos",
    "flex_pos",
    "base_vel",
    "flex_vel",
    "last_action",
)
POLICY_TERM_FIELD_NAMES = (
    "base_pos_rad",
    "flex1_pos_rad",
    "base_vel_rad_s",
    "flex1_vel_rad_s",
    "last_action_delta_rad",
)
POLICY_HISTORY_LAYOUT = "term_major_oldest_to_newest"
POLICY_FRAME_DIM = len(POLICY_TERM_NAMES)
POLICY_INPUT_DIM = POLICY_HISTORY_LENGTH * POLICY_FRAME_DIM

BASE_POSITION_SEMANTICS = (
    "relative angle of joint base_to_fixed against its default joint position"
)
FLEX_POSITION_SEMANTICS = (
    "relative angle of joint fixed_to_flex_1 against its default joint position; "
    "the current contract keeps the raw joint-relative value and does not apply "
    "host-side unwrap inside the policy path"
)
LAST_ACTION_SEMANTICS = (
    "position-delta command sent in the previous control period, maintained by the host"
)


def build_policy_frame(
    *,
    base_pos: float,
    flex_pos: float,
    base_vel: float,
    flex_vel: float,
    last_action: float,
) -> list[float]:
    """Build one policy frame in the agreed per-timestep order."""

    return [
        float(base_pos),
        float(flex_pos),
        float(base_vel),
        float(flex_vel),
        float(last_action),
    ]


def flatten_history_term_major(
    frames: Sequence[Sequence[float]] | Iterable[Sequence[float]],
    *,
    history_length: int = POLICY_HISTORY_LENGTH,
    frame_dim: int = POLICY_FRAME_DIM,
    pad_value: float = 0.0,
) -> list[float]:
    """Flatten history with Isaac Lab's term-major, oldest-to-newest layout."""

    normalized_frames = [[float(value) for value in frame] for frame in frames]
    for frame in normalized_frames:
        if len(frame) != frame_dim:
            raise ValueError(
                f"expected frame_dim={frame_dim}, received {len(frame)} values"
            )

    if len(normalized_frames) > history_length:
        normalized_frames = normalized_frames[-history_length:]

    missing = history_length - len(normalized_frames)
    padded_frames = [[pad_value] * frame_dim for _ in range(missing)] + normalized_frames

    flattened: list[float] = []
    for term_index in range(frame_dim):
        for frame in padded_frames:
            flattened.append(frame[term_index])
    return flattened


def split_history_buckets(
    flattened: Sequence[float] | Iterable[float],
    *,
    history_length: int = POLICY_HISTORY_LENGTH,
    term_names: Sequence[str] = POLICY_TERM_NAMES,
) -> dict[str, list[float]]:
    """Split a flattened policy observation into named term buckets."""

    values = [float(value) for value in flattened]
    expected_length = history_length * len(term_names)
    if len(values) != expected_length:
        raise ValueError(
            f"expected flattened observation length {expected_length}, got {len(values)}"
        )

    buckets: dict[str, list[float]] = {}
    for index, term_name in enumerate(term_names):
        start = index * history_length
        end = start + history_length
        buckets[str(term_name)] = values[start:end]
    return buckets
