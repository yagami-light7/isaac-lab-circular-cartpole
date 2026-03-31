from __future__ import annotations

from dataclasses import dataclass
from struct import Struct
from typing import ClassVar


USB_PC_SOF = 0xA5
USB_PC_FRAME_HEADER_LENGTH = 5
USB_PC_CMD_ID_LENGTH = 2
USB_PC_FRAME_TAIL_LENGTH = 2
USB_PC_MAX_DATA_LENGTH = 64
USB_PC_MAX_FRAME_LENGTH = (
    USB_PC_FRAME_HEADER_LENGTH
    + USB_PC_CMD_ID_LENGTH
    + USB_PC_MAX_DATA_LENGTH
    + USB_PC_FRAME_TAIL_LENGTH
)

USB_PC_CMD_ID_HEARTBEAT_REQ = 0x0101
USB_PC_CMD_ID_HEARTBEAT_ACK = 0x0102
USB_PC_CMD_ID_BOARD_STATUS = 0x0110
USB_PC_CMD_ID_DM4310_STATE = 0x0201
USB_PC_CMD_ID_MT6701_STATE = 0x0202
USB_PC_CMD_ID_L1_SENSOR_STATE = 0x0203
USB_PC_CMD_ID_MOTOR_ENABLE = 0x0301
USB_PC_CMD_ID_CUSTOM_CONTROL = 0x0302
USB_PC_CMD_ID_REMOTE_CONTROL = 0x0304
USB_PC_CMD_ID_MIT_CONTROL = 0x0310
USB_PC_CMD_ID_POSITION_DELTA_CONTROL = 0x0311
USB_PC_CMD_ID_EMERGENCY_STOP = 0x03FF

_HEADER = Struct("<BHBB")
_CMD_ID = Struct("<H")
_CRC16 = Struct("<H")

_HEARTBEAT = Struct("<IHH")
_BOARD_STATUS = Struct("<IHHff")
_DM4310_STATE = Struct("<IBBHffff")
_MT6701_STATE = Struct("<IBBHfff")
_L1_SENSOR_STATE = Struct("<IHHfffff")
_MOTOR_ENABLE = Struct("<BBBB")
_MIT_CONTROL = Struct("<BBHfffff")
_POSITION_DELTA = Struct("<Bf")
_EMERGENCY_STOP = Struct("<I")

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


class A5ProtocolError(ValueError):
    """Raised when a MC02 A5 frame or payload cannot be decoded."""


def crc8_dji(data: bytes, *, initial: int = 0xFF) -> int:
    crc = initial & 0xFF
    for byte in data:
        crc = _CRC8_TABLE[(crc ^ byte) & 0xFF]
    return crc


def crc16_dji(data: bytes, *, initial: int = 0xFFFF) -> int:
    crc = initial & 0xFFFF
    for byte in data:
        crc = ((crc >> 8) ^ _CRC16_TABLE[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc


@dataclass(slots=True)
class A5Frame:
    seq: int
    cmd_id: int
    payload: bytes

    def to_bytes(self) -> bytes:
        return encode_a5_frame(self.cmd_id, self.payload, seq=self.seq)


def encode_a5_frame(cmd_id: int, payload: bytes, *, seq: int) -> bytes:
    if len(payload) > USB_PC_MAX_DATA_LENGTH:
        raise A5ProtocolError(
            f"payload too large: {len(payload)} > {USB_PC_MAX_DATA_LENGTH}"
        )
    header = bytearray(_HEADER.pack(USB_PC_SOF, len(payload), seq & 0xFF, 0))
    header[-1] = crc8_dji(header[:-1])
    body = bytes(header) + _CMD_ID.pack(cmd_id & 0xFFFF) + payload
    return body + _CRC16.pack(crc16_dji(body))


def decode_a5_frame(data: bytes) -> A5Frame:
    if len(data) < USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + USB_PC_FRAME_TAIL_LENGTH:
        raise A5ProtocolError("frame too short")

    sof, data_length, seq, header_crc = _HEADER.unpack_from(data, 0)
    if sof != USB_PC_SOF:
        raise A5ProtocolError(f"bad sof: {sof:#x}")
    if data_length > USB_PC_MAX_DATA_LENGTH:
        raise A5ProtocolError(f"payload too large: {data_length}")
    expected_length = (
        USB_PC_FRAME_HEADER_LENGTH
        + USB_PC_CMD_ID_LENGTH
        + data_length
        + USB_PC_FRAME_TAIL_LENGTH
    )
    if len(data) != expected_length:
        raise A5ProtocolError(
            f"unexpected frame size: {len(data)} (expected {expected_length})"
        )
    if crc8_dji(data[: USB_PC_FRAME_HEADER_LENGTH - 1]) != header_crc:
        raise A5ProtocolError("header crc8 mismatch")

    body = data[:-USB_PC_FRAME_TAIL_LENGTH]
    actual_crc16 = crc16_dji(body)
    expected_crc16 = _CRC16.unpack_from(data, len(data) - USB_PC_FRAME_TAIL_LENGTH)[0]
    if actual_crc16 != expected_crc16:
        raise A5ProtocolError("frame crc16 mismatch")

    cmd_id = _CMD_ID.unpack_from(data, USB_PC_FRAME_HEADER_LENGTH)[0]
    payload_start = USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH
    payload = data[payload_start : payload_start + data_length]
    return A5Frame(seq=seq, cmd_id=cmd_id, payload=payload)


class A5FrameStreamParser:
    """Incremental parser for MC02 A5 USB CDC frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes | bytearray | memoryview) -> list[A5Frame]:
        self._buffer.extend(data)
        frames: list[A5Frame] = []

        while True:
            sof_index = self._buffer.find(bytes((USB_PC_SOF,)))
            if sof_index < 0:
                if len(self._buffer) > USB_PC_FRAME_HEADER_LENGTH - 1:
                    del self._buffer[: -(USB_PC_FRAME_HEADER_LENGTH - 1)]
                break
            if sof_index > 0:
                del self._buffer[:sof_index]
            if len(self._buffer) < USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + USB_PC_FRAME_TAIL_LENGTH:
                break

            sof, data_length, _seq, header_crc = _HEADER.unpack_from(self._buffer, 0)
            if sof != USB_PC_SOF or data_length > USB_PC_MAX_DATA_LENGTH:
                del self._buffer[0]
                continue
            if crc8_dji(self._buffer[: USB_PC_FRAME_HEADER_LENGTH - 1]) != header_crc:
                del self._buffer[0]
                continue

            frame_length = (
                USB_PC_FRAME_HEADER_LENGTH
                + USB_PC_CMD_ID_LENGTH
                + data_length
                + USB_PC_FRAME_TAIL_LENGTH
            )
            if len(self._buffer) < frame_length:
                break

            frame_bytes = bytes(self._buffer[:frame_length])
            try:
                frames.append(decode_a5_frame(frame_bytes))
            except A5ProtocolError:
                del self._buffer[0]
                continue
            del self._buffer[:frame_length]

        return frames


@dataclass(slots=True)
class Heartbeat:
    tick_ms: int
    protocol_version: int
    status_flags: int
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_HEARTBEAT_REQ

    def to_payload(self) -> bytes:
        return _HEARTBEAT.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.protocol_version) & 0xFFFF,
            int(self.status_flags) & 0xFFFF,
        )

    @classmethod
    def from_payload(cls, payload: bytes) -> "Heartbeat":
        tick_ms, protocol_version, status_flags = _unpack_exact(
            _HEARTBEAT, payload, "heartbeat"
        )
        return cls(
            tick_ms=tick_ms,
            protocol_version=protocol_version,
            status_flags=status_flags,
        )


@dataclass(slots=True)
class BoardStatus:
    tick_ms: int
    status_flags: int
    fault_flags: int
    usb_rx_rate_hz: float
    usb_tx_rate_hz: float
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_BOARD_STATUS

    def to_payload(self) -> bytes:
        return _BOARD_STATUS.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.status_flags) & 0xFFFF,
            int(self.fault_flags) & 0xFFFF,
            float(self.usb_rx_rate_hz),
            float(self.usb_tx_rate_hz),
        )

    @classmethod
    def from_payload(cls, payload: bytes) -> "BoardStatus":
        tick_ms, status_flags, fault_flags, usb_rx_rate_hz, usb_tx_rate_hz = _unpack_exact(
            _BOARD_STATUS, payload, "board status"
        )
        return cls(
            tick_ms=tick_ms,
            status_flags=status_flags,
            fault_flags=fault_flags,
            usb_rx_rate_hz=usb_rx_rate_hz,
            usb_tx_rate_hz=usb_tx_rate_hz,
        )


@dataclass(slots=True)
class DM4310State:
    tick_ms: int
    motor_id: int
    mode: int
    fault_flags: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_DM4310_STATE

    def to_payload(self) -> bytes:
        return _DM4310_STATE.pack(
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
    def from_payload(cls, payload: bytes) -> "DM4310State":
        values = _unpack_exact(_DM4310_STATE, payload, "dm4310 state")
        return cls(
            tick_ms=values[0],
            motor_id=values[1],
            mode=values[2],
            fault_flags=values[3],
            position_rad=values[4],
            velocity_rad_s=values[5],
            torque_nm=values[6],
            temperature_c=values[7],
        )


@dataclass(slots=True)
class MT6701State:
    tick_ms: int
    encoder_id: int
    valid_flag: int
    fault_flags: int
    single_turn_rad: float
    multi_turn_rad: float
    velocity_rad_s: float
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_MT6701_STATE

    def to_payload(self) -> bytes:
        return _MT6701_STATE.pack(
            int(self.tick_ms) & 0xFFFFFFFF,
            int(self.encoder_id) & 0xFF,
            int(self.valid_flag) & 0xFF,
            int(self.fault_flags) & 0xFFFF,
            float(self.single_turn_rad),
            float(self.multi_turn_rad),
            float(self.velocity_rad_s),
        )

    @classmethod
    def from_payload(cls, payload: bytes) -> "MT6701State":
        values = _unpack_exact(_MT6701_STATE, payload, "mt6701 state")
        return cls(
            tick_ms=values[0],
            encoder_id=values[1],
            valid_flag=values[2],
            fault_flags=values[3],
            single_turn_rad=values[4],
            multi_turn_rad=values[5],
            velocity_rad_s=values[6],
        )


@dataclass(slots=True)
class L1SensorState:
    tick_ms: int
    status_flags: int
    fault_flags: int
    base_pos_rad: float
    base_vel_rad_s: float
    flex_pos_rad: float
    flex_vel_rad_s: float
    motor_temp_c: float
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_L1_SENSOR_STATE

    def to_payload(self) -> bytes:
        return _L1_SENSOR_STATE.pack(
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
    def from_payload(cls, payload: bytes) -> "L1SensorState":
        values = _unpack_exact(_L1_SENSOR_STATE, payload, "l1 sensor state")
        return cls(
            tick_ms=values[0],
            status_flags=values[1],
            fault_flags=values[2],
            base_pos_rad=values[3],
            base_vel_rad_s=values[4],
            flex_pos_rad=values[5],
            flex_vel_rad_s=values[6],
            motor_temp_c=values[7],
        )


@dataclass(slots=True)
class MotorEnableCommand:
    motor_id: int
    enable_flag: int
    clear_fault_flag: int = 0
    reserved: int = 0
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_MOTOR_ENABLE

    def to_payload(self) -> bytes:
        return _MOTOR_ENABLE.pack(
            int(self.motor_id) & 0xFF,
            int(self.enable_flag) & 0xFF,
            int(self.clear_fault_flag) & 0xFF,
            int(self.reserved) & 0xFF,
        )

    @classmethod
    def from_payload(cls, payload: bytes) -> "MotorEnableCommand":
        motor_id, enable_flag, clear_fault_flag, reserved = _unpack_exact(
            _MOTOR_ENABLE, payload, "motor enable command"
        )
        return cls(
            motor_id=motor_id,
            enable_flag=enable_flag,
            clear_fault_flag=clear_fault_flag,
            reserved=reserved,
        )


@dataclass(slots=True)
class MITControlCommand:
    motor_id: int
    kp: float
    kd: float
    pos_rad: float
    vel_rad_s: float
    torque_nm: float
    reserved0: int = 0
    reserved1: int = 0
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_MIT_CONTROL

    def to_payload(self) -> bytes:
        return _MIT_CONTROL.pack(
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
    def from_payload(cls, payload: bytes) -> "MITControlCommand":
        values = _unpack_exact(_MIT_CONTROL, payload, "mit control command")
        return cls(
            motor_id=values[0],
            reserved0=values[1],
            reserved1=values[2],
            kp=values[3],
            kd=values[4],
            pos_rad=values[5],
            vel_rad_s=values[6],
            torque_nm=values[7],
        )


@dataclass(slots=True)
class PositionDeltaCommand:
    motor_id: int
    pos_delta_rad: float
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_POSITION_DELTA_CONTROL

    def to_payload(self) -> bytes:
        return _POSITION_DELTA.pack(
            int(self.motor_id) & 0xFF,
            float(self.pos_delta_rad),
        )

    @classmethod
    def from_payload(cls, payload: bytes) -> "PositionDeltaCommand":
        values = _unpack_exact(_POSITION_DELTA, payload, "position delta command")
        return cls(
            motor_id=values[0],
            pos_delta_rad=values[1],
        )


@dataclass(slots=True)
class EmergencyStopCommand:
    stop_code: int
    cmd_id: ClassVar[int] = USB_PC_CMD_ID_EMERGENCY_STOP

    def to_payload(self) -> bytes:
        return _EMERGENCY_STOP.pack(int(self.stop_code) & 0xFFFFFFFF)

    @classmethod
    def from_payload(cls, payload: bytes) -> "EmergencyStopCommand":
        (stop_code,) = _unpack_exact(_EMERGENCY_STOP, payload, "emergency stop command")
        return cls(stop_code=stop_code)


def decode_message(frame: A5Frame) -> object:
    if frame.cmd_id in (USB_PC_CMD_ID_HEARTBEAT_REQ, USB_PC_CMD_ID_HEARTBEAT_ACK):
        return Heartbeat.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_BOARD_STATUS:
        return BoardStatus.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_DM4310_STATE:
        return DM4310State.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_MT6701_STATE:
        return MT6701State.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_L1_SENSOR_STATE:
        return L1SensorState.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_MOTOR_ENABLE:
        return MotorEnableCommand.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_MIT_CONTROL:
        return MITControlCommand.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_POSITION_DELTA_CONTROL:
        return PositionDeltaCommand.from_payload(frame.payload)
    if frame.cmd_id == USB_PC_CMD_ID_EMERGENCY_STOP:
        return EmergencyStopCommand.from_payload(frame.payload)
    return frame


def encode_message(message: object, *, seq: int) -> bytes:
    if not hasattr(message, "cmd_id") or not hasattr(message, "to_payload"):
        raise TypeError(f"unsupported message type: {type(message)!r}")
    return encode_a5_frame(message.cmd_id, message.to_payload(), seq=seq)


def _unpack_exact(layout: Struct, payload: bytes, label: str) -> tuple:
    if len(payload) != layout.size:
        raise A5ProtocolError(
            f"unexpected {label} payload size: {len(payload)} (expected {layout.size})"
        )
    return layout.unpack(payload)
