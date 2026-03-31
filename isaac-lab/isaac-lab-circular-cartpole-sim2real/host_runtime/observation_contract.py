from __future__ import annotations

import importlib.util
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[1]
_CONTRACT_PATH = (
    _ROOT
    / "source"
    / "circular_cartpole_sim2real"
    / "circular_cartpole_sim2real"
    / "observation_contract.py"
)

if not _CONTRACT_PATH.exists():
    raise FileNotFoundError(f"observation contract module not found: {_CONTRACT_PATH}")

_SPEC = importlib.util.spec_from_file_location(
    "_circular_cartpole_observation_contract",
    _CONTRACT_PATH,
)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"unable to load observation contract from {_CONTRACT_PATH}")

_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)

BASE_POSITION_SEMANTICS = _MODULE.BASE_POSITION_SEMANTICS
CONTROL_DT_S = _MODULE.CONTROL_DT_S
CONTROL_FREQUENCY_HZ = _MODULE.CONTROL_FREQUENCY_HZ
FLEX_POSITION_SEMANTICS = _MODULE.FLEX_POSITION_SEMANTICS
LAST_ACTION_SEMANTICS = _MODULE.LAST_ACTION_SEMANTICS
OBSERVATION_CONTRACT_VERSION = _MODULE.OBSERVATION_CONTRACT_VERSION
POLICY_FRAME_DIM = _MODULE.POLICY_FRAME_DIM
POLICY_HISTORY_LAYOUT = _MODULE.POLICY_HISTORY_LAYOUT
POLICY_HISTORY_LENGTH = _MODULE.POLICY_HISTORY_LENGTH
POLICY_INPUT_DIM = _MODULE.POLICY_INPUT_DIM
POLICY_TERM_FIELD_NAMES = _MODULE.POLICY_TERM_FIELD_NAMES
POLICY_TERM_NAMES = _MODULE.POLICY_TERM_NAMES
build_policy_frame = _MODULE.build_policy_frame
flatten_history_term_major = _MODULE.flatten_history_term_major
split_history_buckets = _MODULE.split_history_buckets


__all__ = [
    "BASE_POSITION_SEMANTICS",
    "CONTROL_DT_S",
    "CONTROL_FREQUENCY_HZ",
    "FLEX_POSITION_SEMANTICS",
    "LAST_ACTION_SEMANTICS",
    "OBSERVATION_CONTRACT_VERSION",
    "POLICY_FRAME_DIM",
    "POLICY_HISTORY_LAYOUT",
    "POLICY_HISTORY_LENGTH",
    "POLICY_INPUT_DIM",
    "POLICY_TERM_FIELD_NAMES",
    "POLICY_TERM_NAMES",
    "build_policy_frame",
    "flatten_history_term_major",
    "split_history_buckets",
]
