from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol, Sequence


class PolicyBackend(Protocol):
    def predict(self, observation: Sequence[float]) -> Sequence[float]:
        raise NotImplementedError


@dataclass(slots=True)
class CallablePolicyBackend:
    predict_fn: Callable[[Sequence[float]], Sequence[float]]

    def predict(self, observation: Sequence[float]) -> Sequence[float]:
        return self.predict_fn(observation)


@dataclass(slots=True)
class OnnxPolicyBackend:
    model_path: Path

    def __post_init__(self) -> None:
        import onnxruntime as ort  # type: ignore[import-not-found]

        self._session = ort.InferenceSession(
            str(self.model_path), providers=["CPUExecutionProvider"]
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name

    def predict(self, observation: Sequence[float]) -> Sequence[float]:
        import numpy as np

        obs = np.asarray([list(observation)], dtype=np.float32)
        output = self._session.run([self._output_name], {self._input_name: obs})[0]
        return [float(value) for value in output.reshape(-1)]


@dataclass(slots=True)
class TorchScriptPolicyBackend:
    model_path: Path

    def __post_init__(self) -> None:
        import torch  # type: ignore[import-not-found]

        self._torch = torch
        self._module = torch.jit.load(str(self.model_path), map_location="cpu")
        self._module.eval()

    def predict(self, observation: Sequence[float]) -> Sequence[float]:
        with self._torch.inference_mode():
            tensor = self._torch.tensor([list(observation)], dtype=self._torch.float32)
            output = self._module(tensor)
        if hasattr(output, "detach"):
            output = output.detach()
        if hasattr(output, "cpu"):
            output = output.cpu()
        if hasattr(output, "numpy"):
            output = output.numpy()
        if hasattr(output, "reshape"):
            output = output.reshape(-1)
        return [float(value) for value in output]


def load_policy(path: str | Path) -> PolicyBackend:
    model_path = Path(path)
    suffix = model_path.suffix.lower()
    if suffix == ".onnx":
        return OnnxPolicyBackend(model_path)
    if suffix in {".pt", ".pth", ".jit"}:
        return TorchScriptPolicyBackend(model_path)
    raise ValueError(f"unsupported policy format: {model_path.suffix}")

