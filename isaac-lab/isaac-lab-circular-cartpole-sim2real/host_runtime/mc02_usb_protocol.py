from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from struct import Struct
from typing import Any


# 板端当前采用的是基于 A5 帧头的 USB CDC 二进制协议。
# 这里的常量、CRC 算法和 payload 打包顺序都尽量与 MC02 侧保持一致。
USB_PC_SOF = 0xA5
USB_PC_FRAME_HEADER_LENGTH = 5
USB_PC_CMD_ID_LENGTH = 2
USB_PC_FRAME_TAIL_LENGTH = 2
USB_PC_MIN_FRAME_LENGTH = (
    USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + USB_PC_FRAME_TAIL_LENGTH
)
USB_PC_MAX_DATA_LENGTH = 64
USB_PC_FAST_MIT_CONTROL_CMD_ID = 0x0101

USB_PC_PROTOCOL_VERSION = 0x0001

USB_PC_STATUS_MOTOR_ONLINE = 0x0001
USB_PC_STATUS_ENCODER_ONLINE = 0x0002
USB_PC_STATUS_USB_LINK_OK = 0x0004
USB_PC_STATUS_MOTOR_ENABLED = 0x0008
USB_PC_STATUS_MIT_MODE_ACTIVE = 0x0010

USB_PC_FAULT_MOTOR_OVERTEMP = 0x0001
USB_PC_FAULT_MOTOR_OVERSPEED = 0x0002
USB_PC_FAULT_ENCODER_INVALID = 0x0004
USB_PC_FAULT_USB_TIMEOUT = 0x0008
USB_PC_FAULT_FDCAN_OFFLINE = 0x0010

_A5_HEADER = Struct("<BHBB")
_CMD_ID = Struct("<H")
_CRC16 = Struct("<H")

_HEARTBEAT_STRUCT = Struct("<IHH")
_BOARD_STATUS_STRUCT = Struct("<IHHff")
_DM4310_STATE_STRUCT = Struct("<IBBHffff")
_MT6701_STATE_STRUCT = Struct("<IBBHfff")
_L1_SENSOR_STATE_STRUCT = Struct("<IHHfffff")
_SIMPLE_POLE_STATE_STRUCT = Struct("<fffff")
_MOTOR_ENABLE_COMMAND_STRUCT = Struct("<BBBB")
_MIT_COMMAND_STRUCT = Struct("<BBHfffff")
_POSITION_DELTA_COMMAND_STRUCT = Struct("<Bf")
_EMERGENCY_STOP_STRUCT = Struct("<I")


class UsbPcCommandId(IntEnum):
    """当前 USB CDC 协议中已经定义好的命令字。"""

    SIMPLE_POLE_STATE = 0x0100
    HEARTBEAT_REQ = 0x0101
    HEARTBEAT_ACK = 0x0102
    BOARD_STATUS = 0x0110

    DM4310_STATE = 0x0201
    MT6701_STATE = 0x0202
    L1_SENSOR_STATE = 0x0203

    MOTOR_ENABLE = 0x0301
    CUSTOM_CONTROL = 0x0302
    REMOTE_CONTROL = 0x0304
    MIT_CONTROL = 0x0310
    POSITION_DELTA_CONTROL = 0x0311
    EMERGENCY_STOP = 0x03FF


class Mc02UsbFrameError(ValueError):
    """当 A5 数据帧无法通过长度或 CRC 校验时抛出。"""


# 下面两张表直接来自板端 Board_CRC8_CRC16.c。
# 这样 Python 侧和 MCU 侧的 CRC 结果可以完全一致。
_CRC8_INIT = 0xFF
_CRC8_TABLE = (
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
)

_CRC16_INIT = 0xFFFF
_CRC16_TABLE = (
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
)


def get_crc8_check_sum(data: bytes | bytearray | memoryview, initial: int = _CRC8_INIT) -> int:
    """计算板端同款 CRC8。"""

    crc = int(initial) & 0xFF
    for byte in data:
        crc = _CRC8_TABLE[(crc ^ int(byte)) & 0xFF]
    return crc


def verify_crc8_check_sum(data: bytes | bytearray | memoryview) -> bool:
    """验证帧头最后一个字节是否等于 CRC8 结果。"""

    if len(data) <= 2:
        return False
    expected = get_crc8_check_sum(data[:-1], _CRC8_INIT)
    return expected == int(data[-1])


def append_crc8_check_sum(buffer: bytearray) -> None:
    """将 CRC8 写入帧头最后一个字节。"""

    if len(buffer) <= 2:
        return
    buffer[-1] = get_crc8_check_sum(buffer[:-1], _CRC8_INIT)


def get_crc16_check_sum(data: bytes | bytearray | memoryview, initial: int = _CRC16_INIT) -> int:
    """计算板端同款 CRC16。"""

    crc = int(initial) & 0xFFFF
    for byte in data:
        crc = ((crc >> 8) ^ _CRC16_TABLE[(crc ^ int(byte)) & 0x00FF]) & 0xFFFF
    return crc


def verify_crc16_check_sum(data: bytes | bytearray | memoryview) -> bool:
    """验证完整 A5 帧尾部的 CRC16。"""

    if len(data) <= 2:
        return False
    expected = get_crc16_check_sum(data[:-2], _CRC16_INIT)
    received = int(data[-2]) | (int(data[-1]) << 8)
    return expected == received


def append_crc16_check_sum(buffer: bytearray) -> None:
    """将 CRC16 以小端序写入帧尾。"""

    if len(buffer) <= 2:
        return
    crc = get_crc16_check_sum(buffer[:-2], _CRC16_INIT)
    buffer[-2] = crc & 0xFF
    buffer[-1] = (crc >> 8) & 0xFF


@dataclass(slots=True)
class A5UsbFrame:
    """一帧完整的 MC02 USB CDC A5 数据帧。"""

    seq: int
    cmd_id: int
    payload: bytes

    def to_bytes(self) -> bytes:
        payload = bytes(self.payload)
        if len(payload) > USB_PC_MAX_DATA_LENGTH:
            raise Mc02UsbFrameError(
                f"payload too long: {len(payload)} > {USB_PC_MAX_DATA_LENGTH}"
            )

        header = bytearray(_A5_HEADER.pack(USB_PC_SOF, len(payload), self.seq & 0xFF, 0))
        append_crc8_check_sum(header)
        body = bytearray(header)
        body.extend(_CMD_ID.pack(int(self.cmd_id) & 0xFFFF))
        body.extend(payload)
        body.extend(b"\x00\x00")
        append_crc16_check_sum(body)
        return bytes(body)

    @classmethod
    def from_bytes(cls, data: bytes | bytearray | memoryview) -> "A5UsbFrame":
        raw = bytes(data)
        if len(raw) < USB_PC_MIN_FRAME_LENGTH:
            raise Mc02UsbFrameError("frame too short")

        sof, data_length, seq, header_crc8 = _A5_HEADER.unpack_from(raw, 0)
        if sof != USB_PC_SOF:
            raise Mc02UsbFrameError(f"bad sof: 0x{sof:02X}")
        if data_length > USB_PC_MAX_DATA_LENGTH:
            raise Mc02UsbFrameError(
                f"payload too long in header: {data_length} > {USB_PC_MAX_DATA_LENGTH}"
            )
        if not verify_crc8_check_sum(raw[:USB_PC_FRAME_HEADER_LENGTH]):
            raise Mc02UsbFrameError("header crc8 mismatch")

        expected_length = (
            USB_PC_FRAME_HEADER_LENGTH
            + USB_PC_CMD_ID_LENGTH
            + data_length
            + USB_PC_FRAME_TAIL_LENGTH
        )
        if len(raw) != expected_length:
            raise Mc02UsbFrameError(
                f"unexpected frame length: {len(raw)} != {expected_length}"
            )
        if not verify_crc16_check_sum(raw):
            raise Mc02UsbFrameError("frame crc16 mismatch")

        cmd_id = _CMD_ID.unpack_from(raw, USB_PC_FRAME_HEADER_LENGTH)[0]
        payload_offset = USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH
        payload = raw[payload_offset : payload_offset + data_length]
        _ = header_crc8
        return cls(seq=seq, cmd_id=cmd_id, payload=payload)

    def to_dict(self) -> dict[str, Any]:
        return {
            "seq": int(self.seq),
            "cmd_id": int(self.cmd_id),
            "payload_hex": self.payload.hex(),
        }


class A5UsbFrameStreamParser:
    """将串口收到的字节流切分为完整的 A5 数据帧。"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray | memoryview) -> list[A5UsbFrame]:
        self._buffer.extend(data)
        frames: list[A5UsbFrame] = []

        while True:
            sof_index = self._buffer.find(USB_PC_SOF)
            if sof_index < 0:
                if len(self._buffer) > USB_PC_FRAME_HEADER_LENGTH:
                    del self._buffer[:-USB_PC_FRAME_HEADER_LENGTH]
                break

            if sof_index > 0:
                del self._buffer[:sof_index]

            if len(self._buffer) < USB_PC_MIN_FRAME_LENGTH:
                break

            if not verify_crc8_check_sum(self._buffer[:USB_PC_FRAME_HEADER_LENGTH]):
                del self._buffer[0]
                continue

            _sof, data_length, _seq, _crc8 = _A5_HEADER.unpack_from(self._buffer, 0)
            if data_length > USB_PC_MAX_DATA_LENGTH:
                del self._buffer[0]
                continue

            full_frame_length = (
                USB_PC_FRAME_HEADER_LENGTH
                + USB_PC_CMD_ID_LENGTH
                + data_length
                + USB_PC_FRAME_TAIL_LENGTH
            )
            if len(self._buffer) < full_frame_length:
                break

            frame_bytes = bytes(self._buffer[:full_frame_length])
            if not verify_crc16_check_sum(frame_bytes):
                del self._buffer[0]
                continue

            frames.append(A5UsbFrame.from_bytes(frame_bytes))
            del self._buffer[:full_frame_length]

        return frames

    def reset(self) -> None:
        self._buffer.clear()


@dataclass(slots=True)
class HeartbeatPayload:
    tick_ms: int
    protocol_version: int = USB_PC_PROTOCOL_VERSION
    status_flags: int = 0

    def to_bytes(self) -> bytes:
        return _HEARTBEAT_STRUCT.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.protocol_version) & 0xFFFF,
            int(self.status_flags) & 0xFFFF,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "HeartbeatPayload":
        tick_ms, protocol_version, status_flags = _HEARTBEAT_STRUCT.unpack(data)
        return cls(
            tick_ms=tick_ms,
            protocol_version=protocol_version,
            status_flags=status_flags,
        )

    def to_dict(self) -> dict[str, int]:
        return {
            "tick_ms": int(self.tick_ms),
            "protocol_version": int(self.protocol_version),
            "status_flags": int(self.status_flags),
        }


@dataclass(slots=True)
class BoardStatusPayload:
    tick_ms: int
    status_flags: int
    fault_flags: int
    usb_rx_rate_hz: float
    usb_tx_rate_hz: float

    def to_bytes(self) -> bytes:
        return _BOARD_STATUS_STRUCT.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.status_flags) & 0xFFFF,
            int(self.fault_flags) & 0xFFFF,
            float(self.usb_rx_rate_hz),
            float(self.usb_tx_rate_hz),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "BoardStatusPayload":
        tick_ms, status_flags, fault_flags, usb_rx_rate_hz, usb_tx_rate_hz = _BOARD_STATUS_STRUCT.unpack(
            data
        )
        return cls(
            tick_ms=tick_ms,
            status_flags=status_flags,
            fault_flags=fault_flags,
            usb_rx_rate_hz=usb_rx_rate_hz,
            usb_tx_rate_hz=usb_tx_rate_hz,
        )

    def to_dict(self) -> dict[str, int | float]:
        return {
            "tick_ms": int(self.tick_ms),
            "status_flags": int(self.status_flags),
            "fault_flags": int(self.fault_flags),
            "usb_rx_rate_hz": float(self.usb_rx_rate_hz),
            "usb_tx_rate_hz": float(self.usb_tx_rate_hz),
        }


@dataclass(slots=True)
class Dm4310StatePayload:
    tick_ms: int
    motor_id: int
    mode: int
    fault_flags: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float

    def to_bytes(self) -> bytes:
        return _DM4310_STATE_STRUCT.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.motor_id) & 0xFF,
            int(self.mode) & 0xFF,
            int(self.fault_flags) & 0xFFFF,
            float(self.position_rad),
            float(self.velocity_rad_s),
            float(self.torque_nm),
            float(self.temperature_c),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "Dm4310StatePayload":
        (
            tick_ms,
            motor_id,
            mode,
            fault_flags,
            position_rad,
            velocity_rad_s,
            torque_nm,
            temperature_c,
        ) = _DM4310_STATE_STRUCT.unpack(data)
        return cls(
            tick_ms=tick_ms,
            motor_id=motor_id,
            mode=mode,
            fault_flags=fault_flags,
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            torque_nm=torque_nm,
            temperature_c=temperature_c,
        )

    def to_dict(self) -> dict[str, int | float]:
        return {
            "tick_ms": int(self.tick_ms),
            "motor_id": int(self.motor_id),
            "mode": int(self.mode),
            "fault_flags": int(self.fault_flags),
            "position_rad": float(self.position_rad),
            "velocity_rad_s": float(self.velocity_rad_s),
            "torque_nm": float(self.torque_nm),
            "temperature_c": float(self.temperature_c),
        }


@dataclass(slots=True)
class Mt6701StatePayload:
    tick_ms: int
    encoder_id: int
    valid_flag: int
    fault_flags: int
    single_turn_rad: float
    multi_turn_rad: float
    velocity_rad_s: float

    def to_bytes(self) -> bytes:
        return _MT6701_STATE_STRUCT.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.encoder_id) & 0xFF,
            int(self.valid_flag) & 0xFF,
            int(self.fault_flags) & 0xFFFF,
            float(self.single_turn_rad),
            float(self.multi_turn_rad),
            float(self.velocity_rad_s),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "Mt6701StatePayload":
        (
            tick_ms,
            encoder_id,
            valid_flag,
            fault_flags,
            single_turn_rad,
            multi_turn_rad,
            velocity_rad_s,
        ) = _MT6701_STATE_STRUCT.unpack(data)
        return cls(
            tick_ms=tick_ms,
            encoder_id=encoder_id,
            valid_flag=valid_flag,
            fault_flags=fault_flags,
            single_turn_rad=single_turn_rad,
            multi_turn_rad=multi_turn_rad,
            velocity_rad_s=velocity_rad_s,
        )

    def to_dict(self) -> dict[str, int | float]:
        return {
            "tick_ms": int(self.tick_ms),
            "encoder_id": int(self.encoder_id),
            "valid_flag": int(self.valid_flag),
            "fault_flags": int(self.fault_flags),
            "single_turn_rad": float(self.single_turn_rad),
            "multi_turn_rad": float(self.multi_turn_rad),
            "velocity_rad_s": float(self.velocity_rad_s),
        }


@dataclass(slots=True)
class L1SensorStatePayload:
    tick_ms: int
    status_flags: int
    fault_flags: int
    base_pos_rad: float
    base_vel_rad_s: float
    flex_pos_rad: float
    flex_vel_rad_s: float
    motor_temp_c: float

    def to_bytes(self) -> bytes:
        return _L1_SENSOR_STATE_STRUCT.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.status_flags) & 0xFFFF,
            int(self.fault_flags) & 0xFFFF,
            float(self.base_pos_rad),
            float(self.base_vel_rad_s),
            float(self.flex_pos_rad),
            float(self.flex_vel_rad_s),
            float(self.motor_temp_c),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "L1SensorStatePayload":
        (
            tick_ms,
            status_flags,
            fault_flags,
            base_pos_rad,
            base_vel_rad_s,
            flex_pos_rad,
            flex_vel_rad_s,
            motor_temp_c,
        ) = _L1_SENSOR_STATE_STRUCT.unpack(data)
        return cls(
            tick_ms=tick_ms,
            status_flags=status_flags,
            fault_flags=fault_flags,
            base_pos_rad=base_pos_rad,
            base_vel_rad_s=base_vel_rad_s,
            flex_pos_rad=flex_pos_rad,
            flex_vel_rad_s=flex_vel_rad_s,
            motor_temp_c=motor_temp_c,
        )

    def to_dict(self) -> dict[str, int | float]:
        return {
            "tick_ms": int(self.tick_ms),
            "status_flags": int(self.status_flags),
            "fault_flags": int(self.fault_flags),
            "base_pos_rad": float(self.base_pos_rad),
            "base_vel_rad_s": float(self.base_vel_rad_s),
            "flex_pos_rad": float(self.flex_pos_rad),
            "flex_vel_rad_s": float(self.flex_vel_rad_s),
            "motor_temp_c": float(self.motor_temp_c),
        }


@dataclass(slots=True)
class SimplePoleStatePayload:
    base_pos_rad: float
    base_vel_rad_s: float
    motor_torque_nm: float
    flex_pos_rad: float
    flex_vel_rad_s: float

    def to_bytes(self) -> bytes:
        return _SIMPLE_POLE_STATE_STRUCT.pack(
            float(self.base_pos_rad),
            float(self.base_vel_rad_s),
            float(self.motor_torque_nm),
            float(self.flex_pos_rad),
            float(self.flex_vel_rad_s),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "SimplePoleStatePayload":
        (
            base_pos_rad,
            base_vel_rad_s,
            motor_torque_nm,
            flex_pos_rad,
            flex_vel_rad_s,
        ) = _SIMPLE_POLE_STATE_STRUCT.unpack(data)
        return cls(
            base_pos_rad=base_pos_rad,
            base_vel_rad_s=base_vel_rad_s,
            motor_torque_nm=motor_torque_nm,
            flex_pos_rad=flex_pos_rad,
            flex_vel_rad_s=flex_vel_rad_s,
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "base_pos_rad": float(self.base_pos_rad),
            "base_vel_rad_s": float(self.base_vel_rad_s),
            "motor_torque_nm": float(self.motor_torque_nm),
            "flex_pos_rad": float(self.flex_pos_rad),
            "flex_vel_rad_s": float(self.flex_vel_rad_s),
        }


@dataclass(slots=True)
class MotorEnableCommandPayload:
    motor_id: int
    enable_flag: int
    clear_fault_flag: int = 0
    reserved: int = 0

    def to_bytes(self) -> bytes:
        return _MOTOR_ENABLE_COMMAND_STRUCT.pack(
            int(self.motor_id) & 0xFF,
            int(self.enable_flag) & 0xFF,
            int(self.clear_fault_flag) & 0xFF,
            int(self.reserved) & 0xFF,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "MotorEnableCommandPayload":
        motor_id, enable_flag, clear_fault_flag, reserved = _MOTOR_ENABLE_COMMAND_STRUCT.unpack(
            data
        )
        return cls(
            motor_id=motor_id,
            enable_flag=enable_flag,
            clear_fault_flag=clear_fault_flag,
            reserved=reserved,
        )

    def to_dict(self) -> dict[str, int]:
        return {
            "motor_id": int(self.motor_id),
            "enable_flag": int(self.enable_flag),
            "clear_fault_flag": int(self.clear_fault_flag),
            "reserved": int(self.reserved),
        }


@dataclass(slots=True)
class MitControlCommandPayload:
    motor_id: int
    kp: float
    kd: float
    pos_rad: float
    vel_rad_s: float
    torque_nm: float
    reserved0: int = 0
    reserved1: int = 0

    def to_bytes(self) -> bytes:
        return _MIT_COMMAND_STRUCT.pack(
            int(self.motor_id) & 0xFF,
            int(self.reserved0) & 0xFF,
            int(self.reserved1) & 0xFFFF,
            float(self.kp),
            float(self.kd),
            float(self.pos_rad),
            float(self.vel_rad_s),
            float(self.torque_nm),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "MitControlCommandPayload":
        (
            motor_id,
            reserved0,
            reserved1,
            kp,
            kd,
            pos_rad,
            vel_rad_s,
            torque_nm,
        ) = _MIT_COMMAND_STRUCT.unpack(data)
        return cls(
            motor_id=motor_id,
            reserved0=reserved0,
            reserved1=reserved1,
            kp=kp,
            kd=kd,
            pos_rad=pos_rad,
            vel_rad_s=vel_rad_s,
            torque_nm=torque_nm,
        )

    def to_dict(self) -> dict[str, int | float]:
        return {
            "motor_id": int(self.motor_id),
            "reserved0": int(self.reserved0),
            "reserved1": int(self.reserved1),
            "kp": float(self.kp),
            "kd": float(self.kd),
            "pos_rad": float(self.pos_rad),
            "vel_rad_s": float(self.vel_rad_s),
            "torque_nm": float(self.torque_nm),
        }


@dataclass(slots=True)
class PositionDeltaControlCommandPayload:
    motor_id: int
    pos_delta_rad: float

    def to_bytes(self) -> bytes:
        return _POSITION_DELTA_COMMAND_STRUCT.pack(
            int(self.motor_id) & 0xFF,
            float(self.pos_delta_rad),
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "PositionDeltaControlCommandPayload":
        motor_id, pos_delta_rad = _POSITION_DELTA_COMMAND_STRUCT.unpack(data)
        return cls(
            motor_id=motor_id,
            pos_delta_rad=pos_delta_rad,
        )

    def to_dict(self) -> dict[str, int | float]:
        return {
            "motor_id": int(self.motor_id),
            "pos_delta_rad": float(self.pos_delta_rad),
        }


@dataclass(slots=True)
class EmergencyStopCommandPayload:
    stop_code: int

    def to_bytes(self) -> bytes:
        return _EMERGENCY_STOP_STRUCT.pack(int(self.stop_code) & 0xFFFFFFFF)

    @classmethod
    def from_bytes(cls, data: bytes) -> "EmergencyStopCommandPayload":
        (stop_code,) = _EMERGENCY_STOP_STRUCT.unpack(data)
        return cls(stop_code=stop_code)

    def to_dict(self) -> dict[str, int]:
        return {"stop_code": int(self.stop_code)}


_PAYLOAD_DECODER_MAP = {
    int(UsbPcCommandId.SIMPLE_POLE_STATE): SimplePoleStatePayload,
    int(UsbPcCommandId.HEARTBEAT_REQ): HeartbeatPayload,
    int(UsbPcCommandId.HEARTBEAT_ACK): HeartbeatPayload,
    int(UsbPcCommandId.BOARD_STATUS): BoardStatusPayload,
    int(UsbPcCommandId.DM4310_STATE): Dm4310StatePayload,
    int(UsbPcCommandId.MT6701_STATE): Mt6701StatePayload,
    int(UsbPcCommandId.L1_SENSOR_STATE): L1SensorStatePayload,
    int(UsbPcCommandId.MOTOR_ENABLE): MotorEnableCommandPayload,
    int(UsbPcCommandId.MIT_CONTROL): MitControlCommandPayload,
    int(UsbPcCommandId.POSITION_DELTA_CONTROL): PositionDeltaControlCommandPayload,
    int(UsbPcCommandId.EMERGENCY_STOP): EmergencyStopCommandPayload,
}


def decode_known_payload(frame: A5UsbFrame) -> Any:
    """按命令字自动解码已知 payload。未知命令直接返回原始 bytes。"""

    decoder = _PAYLOAD_DECODER_MAP.get(int(frame.cmd_id))
    if decoder is None:
        return bytes(frame.payload)
    return decoder.from_bytes(frame.payload)


def decode_frame_to_dict(frame: A5UsbFrame) -> dict[str, Any]:
    """把一帧数据转成便于打印和记录日志的字典。"""
    try:
        if int(frame.cmd_id) == int(USB_PC_FAST_MIT_CONTROL_CMD_ID) and len(frame.payload) == _MIT_COMMAND_STRUCT.size:
            cmd_name = "FAST_MIT_CONTROL"
            payload_obj: Any = MitControlCommandPayload.from_bytes(frame.payload)
        else:
            payload_obj = decode_known_payload(frame)
            try:
                cmd_name = UsbPcCommandId(frame.cmd_id).name
            except ValueError:
                cmd_name = "UNKNOWN"

        if hasattr(payload_obj, "to_dict"):
            payload_repr: Any = payload_obj.to_dict()
        else:
            payload_repr = {"raw_hex": bytes(payload_obj).hex()}
    except Exception as exc:
        try:
            cmd_name = UsbPcCommandId(frame.cmd_id).name
        except ValueError:
            cmd_name = "UNKNOWN"
        payload_repr = {
            "raw_hex": bytes(frame.payload).hex(),
            "decode_error": str(exc),
        }

    return {
        "seq": int(frame.seq),
        "cmd_id": int(frame.cmd_id),
        "cmd_name": cmd_name,
        "payload": payload_repr,
    }
