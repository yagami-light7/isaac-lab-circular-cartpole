from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from struct import Struct


MAGIC_SENSOR_V1 = b"SEN1"
MAGIC_ACTION_V1 = b"ACT1"
FRAME_VERSION_V1 = 1
FRAME_LENGTH_SENSOR_V1 = 46
FRAME_LENGTH_ACTION_V1 = 42

_HEADER = Struct("<4sBBHIQ")
_SENSOR_PAYLOAD = Struct("<fffffI")
_ACTION_PAYLOAD = Struct("<fffII")
_CRC = Struct("<H")


class FrameKind(IntEnum):
    SENSOR = 1
    ACTION = 2


class FrameCodecError(ValueError):
    """Raised when a binary frame cannot be decoded."""


def crc16_ccitt_false(data: bytes, *, initial: int = 0xFFFF) -> int:
    """Compute CRC-16/CCITT-FALSE over ``data``."""

    crc = initial & 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(slots=True)
class SensorFrameV1:
    seq: int
    t_us: int
    base_pos_rad: float
    base_vel_rad_s: float
    flex1_pos_rad: float
    flex1_vel_rad_s: float
    motor_temp_c: float
    status_flags: int
    frame_kind: FrameKind = FrameKind.SENSOR

    def to_bytes(self) -> bytes:
        payload = _SENSOR_PAYLOAD.pack(
            float(self.base_pos_rad),
            float(self.base_vel_rad_s),
            float(self.flex1_pos_rad),
            float(self.flex1_vel_rad_s),
            float(self.motor_temp_c),
            int(self.status_flags) & 0xFFFFFFFF,
        )
        header = _HEADER.pack(
            MAGIC_SENSOR_V1,
            FRAME_VERSION_V1,
            int(self.frame_kind),
            FRAME_LENGTH_SENSOR_V1,
            int(self.seq) & 0xFFFFFFFF,
            int(self.t_us) & 0xFFFFFFFFFFFFFFFF,
        )
        body = header + payload
        return body + _CRC.pack(crc16_ccitt_false(body))

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorFrameV1":
        frame = _decode_frame(data)
        if not isinstance(frame, SensorFrameV1):
            raise FrameCodecError("frame kind does not match SensorFrameV1")
        return frame

    def to_dict(self) -> dict[str, int | float | str]:
        return {
            "kind": "sensor",
            "seq": self.seq,
            "t_us": self.t_us,
            "base_pos_rad": self.base_pos_rad,
            "base_vel_rad_s": self.base_vel_rad_s,
            "flex1_pos_rad": self.flex1_pos_rad,
            "flex1_vel_rad_s": self.flex1_vel_rad_s,
            "motor_temp_c": self.motor_temp_c,
            "status_flags": self.status_flags,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "SensorFrameV1":
        return cls(
            seq=int(data["seq"]),
            t_us=int(data["t_us"]),
            base_pos_rad=float(data["base_pos_rad"]),
            base_vel_rad_s=float(data["base_vel_rad_s"]),
            flex1_pos_rad=float(data["flex1_pos_rad"]),
            flex1_vel_rad_s=float(data["flex1_vel_rad_s"]),
            motor_temp_c=float(data["motor_temp_c"]),
            status_flags=int(data["status_flags"]),
        )


@dataclass(slots=True)
class ActionFrameV1:
    seq: int
    t_us: int
    base_pos_delta_rad: float
    kp: float
    kd: float
    watchdog_ms: int
    mode_flags: int
    frame_kind: FrameKind = FrameKind.ACTION

    def to_bytes(self) -> bytes:
        payload = _ACTION_PAYLOAD.pack(
            float(self.base_pos_delta_rad),
            float(self.kp),
            float(self.kd),
            int(self.watchdog_ms) & 0xFFFFFFFF,
            int(self.mode_flags) & 0xFFFFFFFF,
        )
        header = _HEADER.pack(
            MAGIC_ACTION_V1,
            FRAME_VERSION_V1,
            int(self.frame_kind),
            FRAME_LENGTH_ACTION_V1,
            int(self.seq) & 0xFFFFFFFF,
            int(self.t_us) & 0xFFFFFFFFFFFFFFFF,
        )
        body = header + payload
        return body + _CRC.pack(crc16_ccitt_false(body))

    @classmethod
    def from_bytes(cls, data: bytes) -> "ActionFrameV1":
        frame = _decode_frame(data)
        if not isinstance(frame, ActionFrameV1):
            raise FrameCodecError("frame kind does not match ActionFrameV1")
        return frame

    def to_dict(self) -> dict[str, int | float | str]:
        return {
            "kind": "action",
            "seq": self.seq,
            "t_us": self.t_us,
            "base_pos_delta_rad": self.base_pos_delta_rad,
            "kp": self.kp,
            "kd": self.kd,
            "watchdog_ms": self.watchdog_ms,
            "mode_flags": self.mode_flags,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ActionFrameV1":
        return cls(
            seq=int(data["seq"]),
            t_us=int(data["t_us"]),
            base_pos_delta_rad=float(data["base_pos_delta_rad"]),
            kp=float(data["kp"]),
            kd=float(data["kd"]),
            watchdog_ms=int(data["watchdog_ms"]),
            mode_flags=int(data["mode_flags"]),
        )


def _decode_frame(data: bytes) -> SensorFrameV1 | ActionFrameV1:
    if len(data) < _HEADER.size + _CRC.size:
        raise FrameCodecError("frame too short")

    magic, version, kind, frame_length, seq, t_us = _HEADER.unpack_from(data, 0)
    if version != FRAME_VERSION_V1:
        raise FrameCodecError(f"unsupported frame version: {version}")
    if kind == FrameKind.SENSOR:
        if magic != MAGIC_SENSOR_V1:
            raise FrameCodecError("bad sensor magic")
        expected_len = FRAME_LENGTH_SENSOR_V1
        payload_unpack = _SENSOR_PAYLOAD.unpack
    elif kind == FrameKind.ACTION:
        if magic != MAGIC_ACTION_V1:
            raise FrameCodecError("bad action magic")
        expected_len = FRAME_LENGTH_ACTION_V1
        payload_unpack = _ACTION_PAYLOAD.unpack
    else:
        raise FrameCodecError(f"unsupported frame kind: {kind}")

    if frame_length != expected_len:
        raise FrameCodecError(
            f"unexpected frame length field: {frame_length} (expected {expected_len})"
        )
    if len(data) != expected_len:
        raise FrameCodecError(
            f"unexpected frame size: {len(data)} (expected {expected_len})"
        )

    body = data[:-_CRC.size]
    expected_crc = _CRC.unpack_from(data, len(data) - _CRC.size)[0]
    actual_crc = crc16_ccitt_false(body)
    if actual_crc != expected_crc:
        raise FrameCodecError("crc mismatch")

    payload = data[_HEADER.size:-_CRC.size]
    if kind == FrameKind.SENSOR:
        base_pos_rad, base_vel_rad_s, flex1_pos_rad, flex1_vel_rad_s, motor_temp_c, status_flags = payload_unpack(
            payload
        )
        return SensorFrameV1(
            seq=seq,
            t_us=t_us,
            base_pos_rad=base_pos_rad,
            base_vel_rad_s=base_vel_rad_s,
            flex1_pos_rad=flex1_pos_rad,
            flex1_vel_rad_s=flex1_vel_rad_s,
            motor_temp_c=motor_temp_c,
            status_flags=status_flags,
        )

    base_pos_delta_rad, kp, kd, watchdog_ms, mode_flags = payload_unpack(payload)
    return ActionFrameV1(
        seq=seq,
        t_us=t_us,
        base_pos_delta_rad=base_pos_delta_rad,
        kp=kp,
        kd=kd,
        watchdog_ms=watchdog_ms,
        mode_flags=mode_flags,
    )


class FrameStreamParser:
    """Incremental parser for mixed sensor/action binary frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray | memoryview) -> list[SensorFrameV1 | ActionFrameV1]:
        self._buffer.extend(data)
        frames: list[SensorFrameV1 | ActionFrameV1] = []

        while True:
            magic_index = self._find_magic()
            if magic_index < 0:
                break
            if magic_index > 0:
                del self._buffer[:magic_index]
            if len(self._buffer) < _HEADER.size + _CRC.size:
                break

            _, version, kind, frame_length, _seq, _t_us = _HEADER.unpack_from(
                self._buffer, 0
            )
            if version != FRAME_VERSION_V1 or kind not in (FrameKind.SENSOR, FrameKind.ACTION):
                del self._buffer[0]
                continue

            expected_len = (
                FRAME_LENGTH_SENSOR_V1 if kind == FrameKind.SENSOR else FRAME_LENGTH_ACTION_V1
            )
            frame_size = expected_len
            if len(self._buffer) < frame_size:
                break
            if frame_length != expected_len:
                del self._buffer[0]
                continue

            frame_bytes = bytes(self._buffer[:frame_size])
            try:
                frames.append(_decode_frame(frame_bytes))
            except FrameCodecError:
                del self._buffer[0]
                continue
            del self._buffer[:frame_size]

        return frames

    def reset(self) -> None:
        self._buffer.clear()

    def _find_magic(self) -> int:
        sensor_index = self._buffer.find(MAGIC_SENSOR_V1)
        action_index = self._buffer.find(MAGIC_ACTION_V1)
        candidates = [index for index in (sensor_index, action_index) if index >= 0]
        if not candidates:
            if len(self._buffer) > 3:
                del self._buffer[: -3]
            return -1
        return min(candidates)
