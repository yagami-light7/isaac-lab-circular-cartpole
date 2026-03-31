from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, TextIO

from .protocol import ActionFrameV1, SensorFrameV1


@dataclass(slots=True)
class JsonlLogger:
    path: Path
    encoding: str = "utf-8"
    _fh: TextIO | None = None

    def __enter__(self) -> "JsonlLogger":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self._fh is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = self.path.open("a", encoding=self.encoding, newline="\n")

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def write(self, record: Mapping[str, Any]) -> None:
        if self._fh is None:
            self.open()
        assert self._fh is not None
        self._fh.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
        self._fh.write("\n")
        self._fh.flush()

    def log_cycle(
        self,
        *,
        sensor: SensorFrameV1,
        observation: Iterable[float],
        action: ActionFrameV1,
        previous_action: ActionFrameV1 | None,
        observation_history: Iterable[float] | None = None,
        model_output: Iterable[float] | None = None,
        extra: Mapping[str, Any] | None = None,
    ) -> None:
        record: dict[str, Any] = {
            "seq": int(sensor.seq),
            "t_us": int(sensor.t_us),
            "sensor_frame": sensor.to_dict(),
            "sensor_frame_hex": sensor.to_bytes().hex(),
            "action_frame": action.to_dict(),
            "action_frame_hex": action.to_bytes().hex(),
            "previous_action_frame": previous_action.to_dict() if previous_action else None,
            "observation": [float(value) for value in observation],
        }
        if observation_history is not None:
            record["observation_history"] = [float(value) for value in observation_history]
        if model_output is not None:
            record["model_output"] = [float(value) for value in model_output]
        if extra:
            record["extra"] = dict(extra)
        self.write(record)


def iter_jsonl(path: str | Path) -> Iterable[dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                yield json.loads(line)

