from __future__ import annotations

from dataclasses import dataclass, field

from .a5_protocol import (
    A5Frame,
    A5FrameStreamParser,
    BoardStatus,
    DM4310State,
    EmergencyStopCommand,
    L1SensorState,
    MITControlCommand,
    MT6701State,
    MotorEnableCommand,
    PositionDeltaCommand,
    decode_message,
    encode_message,
)
from .protocol import ActionFrameV1, FrameStreamParser, SensorFrameV1


@dataclass(slots=True)
class SerialCdcLink:
    port: str
    baudrate: int = 921600
    timeout: float = 0.01
    _serial: object | None = field(default=None, init=False, repr=False)
    _parser: FrameStreamParser = field(default_factory=FrameStreamParser, init=False)

    def open(self) -> None:
        if self._serial is not None:
            return
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise ImportError("pyserial is required for SerialCdcLink") from exc

        self._serial = serial.Serial(
            self.port, baudrate=self.baudrate, timeout=self.timeout
        )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def write_frame(self, frame: SensorFrameV1 | ActionFrameV1) -> None:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        self._serial.write(frame.to_bytes())

    def read_frames(self) -> list[SensorFrameV1 | ActionFrameV1]:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        in_waiting = int(getattr(self._serial, "in_waiting", 0))
        data = self._serial.read(in_waiting or 1)
        return self._parser.feed(data)

    def feed_bytes(self, data: bytes | bytearray | memoryview) -> list[SensorFrameV1 | ActionFrameV1]:
        return self._parser.feed(data)

    def __enter__(self) -> "SerialCdcLink":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


@dataclass(slots=True)
class _Mc02SensorAssembler:
    use_multi_turn_encoder: bool = True
    _next_seq: int = field(default=1, init=False)
    _latest_motor: DM4310State | None = field(default=None, init=False, repr=False)
    _latest_encoder: MT6701State | None = field(default=None, init=False, repr=False)
    _motor_dirty: bool = field(default=False, init=False, repr=False)
    _encoder_dirty: bool = field(default=False, init=False, repr=False)

    def ingest(self, message: object) -> list[SensorFrameV1]:
        if isinstance(message, L1SensorState):
            return [self._from_l1_state(message)]
        if isinstance(message, DM4310State):
            self._latest_motor = message
            self._motor_dirty = True
        elif isinstance(message, MT6701State):
            self._latest_encoder = message
            self._encoder_dirty = True
        else:
            return []

        if (
            self._latest_motor is None
            or self._latest_encoder is None
            or not self._motor_dirty
            or not self._encoder_dirty
            or not int(self._latest_encoder.valid_flag)
        ):
            return []

        self._motor_dirty = False
        self._encoder_dirty = False
        return [self._from_motor_and_encoder(self._latest_motor, self._latest_encoder)]

    def _from_l1_state(self, message: L1SensorState) -> SensorFrameV1:
        status_flags = (int(message.fault_flags) & 0xFFFF) << 16
        status_flags |= int(message.status_flags) & 0xFFFF
        return SensorFrameV1(
            seq=self._allocate_seq(),
            t_us=int(message.tick_ms) * 1000,
            base_pos_rad=float(message.base_pos_rad),
            base_vel_rad_s=float(message.base_vel_rad_s),
            flex1_pos_rad=float(message.flex_pos_rad),
            flex1_vel_rad_s=float(message.flex_vel_rad_s),
            motor_temp_c=float(message.motor_temp_c),
            status_flags=status_flags,
        )

    def _from_motor_and_encoder(
        self,
        motor: DM4310State,
        encoder: MT6701State,
    ) -> SensorFrameV1:
        flex_pos = (
            float(encoder.multi_turn_rad)
            if self.use_multi_turn_encoder
            else float(encoder.single_turn_rad)
        )
        status_flags = 0
        status_flags |= 0x0001
        if int(encoder.valid_flag):
            status_flags |= 0x0002
        status_flags |= (int(motor.fault_flags) & 0xFF) << 16
        status_flags |= (int(encoder.fault_flags) & 0xFF) << 24
        return SensorFrameV1(
            seq=self._allocate_seq(),
            t_us=max(int(motor.tick_ms), int(encoder.tick_ms)) * 1000,
            base_pos_rad=float(motor.position_rad),
            base_vel_rad_s=float(motor.velocity_rad_s),
            flex1_pos_rad=flex_pos,
            flex1_vel_rad_s=float(encoder.velocity_rad_s),
            motor_temp_c=float(motor.temperature_c),
            status_flags=status_flags,
        )

    def _allocate_seq(self) -> int:
        seq = self._next_seq
        self._next_seq += 1
        return seq


@dataclass(slots=True)
class Mc02A5SerialCdcLink:
    port: str
    baudrate: int = 921600
    timeout: float = 0.01
    motor_id: int = 1
    use_multi_turn_encoder: bool = True
    _serial: object | None = field(default=None, init=False, repr=False)
    _parser: A5FrameStreamParser = field(default_factory=A5FrameStreamParser, init=False, repr=False)
    _assembler: _Mc02SensorAssembler = field(init=False, repr=False)
    _tx_seq: int = field(default=0, init=False, repr=False)
    last_messages: list[object] = field(default_factory=list, init=False)
    last_board_status: BoardStatus | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self._assembler = _Mc02SensorAssembler(
            use_multi_turn_encoder=self.use_multi_turn_encoder
        )

    def open(self) -> None:
        if self._serial is not None:
            return
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise ImportError("pyserial is required for Mc02A5SerialCdcLink") from exc

        self._serial = serial.Serial(
            self.port, baudrate=self.baudrate, timeout=self.timeout
        )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def read_messages(self) -> list[object]:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        in_waiting = int(getattr(self._serial, "in_waiting", 0))
        data = self._serial.read(in_waiting or 1)
        return self.feed_bytes(data)

    def read_frames(self) -> list[SensorFrameV1]:
        frames: list[SensorFrameV1] = []
        for message in self.read_messages():
            frames.extend(self._assembler.ingest(message))
        return frames

    def feed_bytes(self, data: bytes | bytearray | memoryview) -> list[object]:
        frames = self._parser.feed(data)
        messages = [decode_message(frame) for frame in frames]
        self.last_messages = messages
        for message in messages:
            if isinstance(message, BoardStatus):
                self.last_board_status = message
        return messages

    def feed_sensor_bytes(self, data: bytes | bytearray | memoryview) -> list[SensorFrameV1]:
        frames: list[SensorFrameV1] = []
        for message in self.feed_bytes(data):
            frames.extend(self._assembler.ingest(message))
        return frames

    def write_frame(self, frame: SensorFrameV1 | ActionFrameV1) -> None:
        if not isinstance(frame, ActionFrameV1):
            raise TypeError("Mc02A5SerialCdcLink only supports ActionFrameV1 writes")
        command = PositionDeltaCommand(
            motor_id=int(self.motor_id),
            pos_delta_rad=float(frame.base_pos_delta_rad),
        )
        self.write_message(command)

    def write_message(self, message: object) -> None:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        self._serial.write(encode_message(message, seq=self._tx_seq))
        self._tx_seq = (self._tx_seq + 1) & 0xFF

    def write_raw_frame(self, frame: A5Frame) -> None:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        self._serial.write(frame.to_bytes())

    def enable_motor(self, *, enable: bool = True, clear_fault: bool = False) -> None:
        self.write_message(
            MotorEnableCommand(
                motor_id=int(self.motor_id),
                enable_flag=1 if enable else 0,
                clear_fault_flag=1 if clear_fault else 0,
            )
        )

    def send_mit_command(
        self,
        *,
        kp: float,
        kd: float,
        pos_rad: float,
        vel_rad_s: float = 0.0,
        torque_nm: float = 0.0,
    ) -> None:
        self.write_message(
            MITControlCommand(
                motor_id=int(self.motor_id),
                kp=float(kp),
                kd=float(kd),
                pos_rad=float(pos_rad),
                vel_rad_s=float(vel_rad_s),
                torque_nm=float(torque_nm),
            )
        )

    def emergency_stop(self, *, stop_code: int = 1) -> None:
        self.write_message(EmergencyStopCommand(stop_code=int(stop_code)))

    def __enter__(self) -> "Mc02A5SerialCdcLink":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
