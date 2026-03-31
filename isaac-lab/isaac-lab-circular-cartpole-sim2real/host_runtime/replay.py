from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from .history import ObservationHistory, build_history_frame
from .policy import PolicyBackend, load_policy
from .protocol import ActionFrameV1, SensorFrameV1
from .session_log import iter_jsonl


@dataclass(slots=True)
class ReplayReport:
    records: int = 0
    seq_errors: int = 0
    observation_errors: int = 0
    action_errors: int = 0
    frame_decode_errors: int = 0
    details: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return (
            self.seq_errors == 0
            and self.observation_errors == 0
            and self.action_errors == 0
            and self.frame_decode_errors == 0
        )


def _max_abs_diff(lhs: Sequence[float], rhs: Sequence[float]) -> float:
    lhs_values = [float(value) for value in lhs]
    rhs_values = [float(value) for value in rhs]
    if len(lhs_values) != len(rhs_values):
        return float("inf")
    return max((abs(a - b) for a, b in zip(lhs_values, rhs_values)), default=0.0)


def replay_log(
    path: str | Path,
    *,
    policy: PolicyBackend | None = None,
    history_length: int = 3,
    feature_dim: int = 5,
    atol: float = 1e-5,
    validate_policy: bool = True,
    detail_limit: int = 20,
) -> ReplayReport:
    history = ObservationHistory(frame_dim=feature_dim, history_length=history_length)
    report = ReplayReport()
    previous_sensor_seq: int | None = None

    for record in iter_jsonl(path):
        report.records += 1

        try:
            sensor = _load_sensor_frame(record)
            action = _load_action_frame(record)
        except (KeyError, ValueError, TypeError) as exc:
            report.frame_decode_errors += 1
            if len(report.details) < detail_limit:
                report.details.append(f"decode error at record {report.records}: {exc}")
            continue

        if previous_sensor_seq is not None and sensor.seq != previous_sensor_seq + 1:
            report.seq_errors += 1
            if len(report.details) < detail_limit:
                report.details.append(
                    f"seq gap at record {report.records}: {previous_sensor_seq} -> {sensor.seq}"
                )
        previous_sensor_seq = sensor.seq

        previous_action = _load_previous_action(record)
        features = build_history_frame(sensor, previous_action)
        history.push(features)
        observation = history.flatten()

        logged_observation = record.get("observation_history") or record.get("observation")
        if logged_observation is not None:
            diff = _max_abs_diff(observation, logged_observation)
            if diff > atol:
                report.observation_errors += 1
                if len(report.details) < detail_limit:
                    report.details.append(
                        f"observation mismatch at seq {sensor.seq}: diff={diff:.6g}"
                    )

        if validate_policy and policy is not None:
            predicted = list(policy.predict(observation))
            logged_action = action.base_pos_delta_rad
            if not predicted:
                report.action_errors += 1
                if len(report.details) < detail_limit:
                    report.details.append(f"policy returned no output at seq {sensor.seq}")
            elif abs(float(predicted[0]) - float(logged_action)) > atol:
                report.action_errors += 1
                if len(report.details) < detail_limit:
                    report.details.append(
                        f"action mismatch at seq {sensor.seq}: "
                        f"pred={predicted[0]:.6g}, logged={logged_action:.6g}"
                    )

    return report


def _load_sensor_frame(record: dict) -> SensorFrameV1:
    if "sensor_frame_hex" in record:
        return SensorFrameV1.from_bytes(bytes.fromhex(record["sensor_frame_hex"]))
    if "sensor_frame" in record:
        return SensorFrameV1.from_dict(record["sensor_frame"])
    raise KeyError("sensor frame missing")


def _load_action_frame(record: dict) -> ActionFrameV1:
    if "action_frame_hex" in record:
        return ActionFrameV1.from_bytes(bytes.fromhex(record["action_frame_hex"]))
    if "action_frame" in record:
        return ActionFrameV1.from_dict(record["action_frame"])
    raise KeyError("action frame missing")


def _load_previous_action(record: dict) -> ActionFrameV1 | None:
    value = record.get("previous_action_frame")
    if value is None:
        return None
    return ActionFrameV1.from_dict(value)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay and validate host runtime logs.")
    parser.add_argument("--log", required=True, help="Path to a JSONL log file.")
    parser.add_argument("--policy", help="Optional .onnx or TorchScript policy path.")
    parser.add_argument("--history-length", type=int, default=3)
    parser.add_argument("--feature-dim", type=int, default=5)
    parser.add_argument("--atol", type=float, default=1e-5)
    parser.add_argument(
        "--no-policy-check", action="store_true", help="Skip policy output validation."
    )
    args = parser.parse_args(argv)

    policy = load_policy(args.policy) if args.policy else None
    report = replay_log(
        args.log,
        policy=policy,
        history_length=args.history_length,
        feature_dim=args.feature_dim,
        atol=args.atol,
        validate_policy=not args.no_policy_check,
    )
    print(report)
    if report.details:
        for item in report.details:
            print(item)
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
