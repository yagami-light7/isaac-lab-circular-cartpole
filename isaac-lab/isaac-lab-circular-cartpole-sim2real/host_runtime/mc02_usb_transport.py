from __future__ import annotations

from dataclasses import dataclass, field

from .mc02_usb_protocol import (
    A5UsbFrame,
    A5UsbFrameStreamParser,
    MitControlCommandPayload,
    MotorEnableCommandPayload,
    USB_PC_FAST_MIT_CONTROL_CMD_ID,
    UsbPcCommandId,
)


def find_stm_virtual_com_port() -> str:
    """自动查找最像 MC02 的 STM 虚拟串口。"""

    try:
        from serial.tools import list_ports  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - 依赖缺失时由运行环境决定
        raise ImportError("pyserial is required for COM port auto-detection") from exc

    candidates: list[tuple[int, str]] = []

    for port in list_ports.comports():
        description = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        hwid = (port.hwid or "").lower()
        score = 0

        # 这是 ST 官方 VCP 驱动最可靠的识别特征。
        if port.vid == 0x0483 and port.pid == 0x5740:
            score += 100
        if "stmicroelectronics" in description:
            score += 60
        if "stmicroelectronics" in manufacturer:
            score += 40
        if "virtual com port" in description:
            score += 30
        if "stm" in description or "stm" in manufacturer or "0483:5740" in hwid:
            score += 20

        # 蓝牙串口经常很多，主动降权避免误选。
        if "bluetooth" in description:
            score -= 200

        if score > 0:
            candidates.append((score, port.device))

    if not candidates:
        raise RuntimeError("未找到可用的 STM 虚拟串口，请手动指定 --port COMx")

    candidates.sort(key=lambda item: (-item[0], item[1]))
    return candidates[0][1]


@dataclass(slots=True)
class Mc02SerialCdcLink:
    """当前 MC02 USB CDC 链路的串口封装。"""

    port: str | None = None
    baudrate: int = 921600
    timeout: float = 0.01
    _serial: object | None = field(default=None, init=False, repr=False)
    _parser: A5UsbFrameStreamParser = field(
        default_factory=A5UsbFrameStreamParser, init=False, repr=False
    )
    _resolved_port: str | None = field(default=None, init=False, repr=False)

    def open(self) -> None:
        if self._serial is not None:
            return

        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - 依赖缺失时由运行环境决定
            raise ImportError("pyserial is required for Mc02SerialCdcLink") from exc

        if self.port in (None, "", "auto"):
            self._resolved_port = find_stm_virtual_com_port()
        else:
            self._resolved_port = str(self.port)

        self._serial = serial.Serial(
            self._resolved_port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def write_frame(self, frame: A5UsbFrame) -> None:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        self._serial.write(frame.to_bytes())

    def read_frames(self) -> list[A5UsbFrame]:
        if self._serial is None:
            self.open()
        assert self._serial is not None
        in_waiting = int(getattr(self._serial, "in_waiting", 0))
        data = self._serial.read(in_waiting or 1)
        return self._parser.feed(data)

    def feed_bytes(
        self, data: bytes | bytearray | memoryview
    ) -> list[A5UsbFrame]:
        return self._parser.feed(data)

    @property
    def resolved_port(self) -> str | None:
        return self._resolved_port

    def enable_motor(
        self,
        *,
        motor_id: int = 1,
        enable: bool = True,
        clear_fault: bool = False,
        seq: int = 0,
    ) -> None:
        """发送一帧电机使能/失能命令。"""

        payload = MotorEnableCommandPayload(
            motor_id=motor_id,
            enable_flag=1 if enable else 0,
            clear_fault_flag=1 if clear_fault else 0,
        )
        self.write_frame(
            A5UsbFrame(
                seq=seq,
                cmd_id=int(UsbPcCommandId.MOTOR_ENABLE),
                payload=payload.to_bytes(),
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
        motor_id: int = 1,
        seq: int = 0,
    ) -> None:
        """发送一帧 MIT 控制命令。"""

        payload = MitControlCommandPayload(
            motor_id=motor_id,
            kp=kp,
            kd=kd,
            pos_rad=pos_rad,
            vel_rad_s=vel_rad_s,
            torque_nm=torque_nm,
        )
        self.write_frame(
            A5UsbFrame(
                seq=seq,
                cmd_id=int(USB_PC_FAST_MIT_CONTROL_CMD_ID),
                payload=payload.to_bytes(),
            )
        )

    def __enter__(self) -> "Mc02SerialCdcLink":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
