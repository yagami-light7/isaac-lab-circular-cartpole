from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from .history import ObservationHistory, build_history_frame
from .observation_contract import POLICY_FRAME_DIM, POLICY_HISTORY_LENGTH
from .policy import PolicyBackend, load_policy
from .protocol import ActionFrameV1, SensorFrameV1


@dataclass(slots=True)
class HostRuntimeConfig:
    policy_path: str | Path | None = None
    history_length: int = POLICY_HISTORY_LENGTH
    feature_dim: int = POLICY_FRAME_DIM
    kp: float = 1.0
    kd: float = 0.0
    watchdog_ms: int = 100
    mode_flags: int = 1
    action_limit_rad: float | None = None


@dataclass(slots=True)
class HostRuntime:
    config: HostRuntimeConfig = field(default_factory=HostRuntimeConfig)
    policy: PolicyBackend | None = None
    history: ObservationHistory = field(init=False)
    previous_action: ActionFrameV1 | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self.history = ObservationHistory(
            frame_dim=self.config.feature_dim,
            history_length=self.config.history_length,
        )
        if self.policy is None:
            if self.config.policy_path is None:
                raise ValueError("policy or policy_path must be provided")
            self.policy = load_policy(self.config.policy_path)

    def reset(self) -> None:
        self.history.reset()
        self.previous_action = None

    def build_observation(self, sensor: SensorFrameV1) -> list[float]:
        features = build_history_frame(sensor, self.previous_action)
        self.history.push(features)
        return self.history.flatten()

    def step(self, sensor: SensorFrameV1) -> tuple[list[float], ActionFrameV1, list[float]]:
        if self.policy is None:
            raise RuntimeError("policy backend not available")

        observation = self.build_observation(sensor)
        model_output = list(self.policy.predict(observation))
        if not model_output:
            raise RuntimeError("policy returned no action values")

        delta = float(model_output[0])
        if self.config.action_limit_rad is not None:
            limit = float(self.config.action_limit_rad)
            delta = max(-limit, min(limit, delta))

        action = ActionFrameV1(
            seq=sensor.seq,
            t_us=sensor.t_us,
            base_pos_delta_rad=delta,
            kp=float(self.config.kp),
            kd=float(self.config.kd),
            watchdog_ms=int(self.config.watchdog_ms),
            mode_flags=int(self.config.mode_flags),
        )
        self.previous_action = action
        return observation, action, model_output

    def step_from_values(
        self,
        *,
        seq: int,
        t_us: int,
        sensor_values: Sequence[float],
        status_flags: int = 0,
    ) -> tuple[list[float], ActionFrameV1, list[float]]:
        """Build one sensor frame from policy-order values.

        ``sensor_values`` follows the runtime contract order:
        base_pos, flex1_pos, base_vel, flex1_vel, motor_temp.
        """

        sensor = SensorFrameV1(
            seq=seq,
            t_us=t_us,
            base_pos_rad=float(sensor_values[0]),
            base_vel_rad_s=float(sensor_values[2]),
            flex1_pos_rad=float(sensor_values[1]),
            flex1_vel_rad_s=float(sensor_values[3]),
            motor_temp_c=float(sensor_values[4]),
            status_flags=int(status_flags),
        )
        return self.step(sensor)
