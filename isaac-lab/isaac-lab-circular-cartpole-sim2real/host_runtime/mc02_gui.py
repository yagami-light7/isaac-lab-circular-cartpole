from __future__ import annotations

import json
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass

from .mc02_usb_protocol import (
    A5UsbFrame,
    HeartbeatPayload,
    MitControlCommandPayload,
    MotorEnableCommandPayload,
    SimplePoleStatePayload,
    USB_PC_FAST_MIT_CONTROL_CMD_ID,
    USB_PC_STATUS_USB_LINK_OK,
    UsbPcCommandId,
    decode_frame_to_dict,
)
from .mc02_usb_transport import Mc02SerialCdcLink


THEME = {
    "bg": "#0B1220",
    "panel": "#111827",
    "panel_alt": "#0F172A",
    "border": "#243042",
    "text": "#E5E7EB",
    "text_dim": "#94A3B8",
    "accent": "#38BDF8",
    "success": "#22C55E",
    "warning": "#F59E0B",
    "danger": "#EF4444",
    "violet": "#A78BFA",
}


@dataclass(slots=True)
class TelemetryState:
    """保存最近一帧 5 个 float 的回传数据。"""

    timestamp: float = 0.0
    seq: int = 0
    base_pos_rad: float = 0.0
    base_vel_rad_s: float = 0.0
    feedback_torque_nm: float = 0.0
    flex_pos_rad: float = 0.0
    flex_vel_rad_s: float = 0.0


@dataclass(slots=True)
class CommandState:
    """保存最近一次发出的 MIT 指令。"""

    timestamp: float = 0.0
    pos_rad: float = 0.0
    vel_rad_s: float = 0.0
    torque_nm: float = 0.0
    kp: float = 8.0
    kd: float = 0.2
    mode: str = "待机"


@dataclass(slots=True)
class SineConfig:
    """正弦测试配置。"""

    motor_id: int
    amplitude_rad: float
    frequency_hz: float
    offset_rad: float
    torque_bias_nm: float
    kp: float
    kd: float


@dataclass(slots=True)
class ManualMitConfig:
    """手动 MIT 持续发送配置。"""

    motor_id: int
    pos_rad: float
    vel_rad_s: float
    torque_nm: float
    kp: float
    kd: float


class MetricCard(tk.Frame):
    """简单状态卡片。"""

    def __init__(self, master: tk.Misc, title: str, accent: str) -> None:
        super().__init__(master, bg=THEME["panel"], highlightthickness=1, highlightbackground=THEME["border"])
        tk.Frame(self, bg=accent, height=3).pack(fill="x", side="top")
        tk.Label(self, text=title, bg=THEME["panel"], fg=THEME["text_dim"], font=("Microsoft YaHei UI", 10)).pack(
            anchor="w", padx=12, pady=(10, 2)
        )
        self.value = tk.Label(
            self,
            text="--",
            bg=THEME["panel"],
            fg=THEME["text"],
            font=("Consolas", 18, "bold"),
        )
        self.value.pack(anchor="w", padx=12, pady=(0, 10))

    def set(self, text: str) -> None:
        self.value.configure(text=text)


class StripChart(tk.Frame):
    """复用折线对象的轻量图表。"""

    def __init__(self, master: tk.Misc, title: str, series: list[tuple[str, str]], history_size: int) -> None:
        super().__init__(master, bg=THEME["panel"], highlightthickness=1, highlightbackground=THEME["border"])
        self._series_def = series
        self._history_size = history_size
        self._values: dict[str, list[float]] = {name: [] for name, _ in series}
        self._lines: dict[str, int] = {}

        header = tk.Frame(self, bg=THEME["panel"])
        header.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(header, text=title, bg=THEME["panel"], fg=THEME["text"], font=("Microsoft YaHei UI", 11, "bold")).pack(
            side="left"
        )
        legend = tk.Frame(header, bg=THEME["panel"])
        legend.pack(side="right")
        for name, color in series:
            tk.Label(legend, text=f"■ {name}", bg=THEME["panel"], fg=color, font=("Microsoft YaHei UI", 9)).pack(
                side="left", padx=(8, 0)
            )

        self.canvas = tk.Canvas(self, bg=THEME["panel_alt"], highlightthickness=0, height=180)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.canvas.bind("<Configure>", self._on_resize)
        self.range_label = tk.Label(self.canvas, bg=THEME["panel_alt"], fg=THEME["text_dim"], font=("Consolas", 9))
        self.range_window = self.canvas.create_window(0, 0, anchor="ne", window=self.range_label)

    def _on_resize(self, _event: tk.Event[tk.Canvas]) -> None:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        self.canvas.delete("grid")
        for i in range(6):
            y = i * height / 5
            x = i * width / 5
            self.canvas.create_line(0, y, width, y, fill="#162235", tags="grid")
            self.canvas.create_line(x, 0, x, height, fill="#152033", tags="grid")
        for name, color in self._series_def:
            if name not in self._lines:
                self._lines[name] = self.canvas.create_line(0, 0, 0, 0, fill=color, width=2)
        self.canvas.coords(self.range_window, width - 8, 8)
        self.redraw()

    def update(self, values: dict[str, list[float]]) -> None:
        self._values = values
        self.redraw()

    def redraw(self) -> None:
        if not self._lines:
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        all_values: list[float] = []
        for name, _ in self._series_def:
            all_values.extend(self._values.get(name, []))
        if not all_values:
            for line in self._lines.values():
                self.canvas.coords(line, 0, 0, 0, 0)
            self.range_label.configure(text="无数据")
            return
        y_min = min(all_values)
        y_max = max(all_values)
        if math.isclose(y_min, y_max, abs_tol=1e-6):
            y_min -= 1.0
            y_max += 1.0
        pad = (y_max - y_min) * 0.12 + 1e-4
        y_min -= pad
        y_max += pad
        self.range_label.configure(text=f"{y_max:+.3f}\n{y_min:+.3f}")
        for name, _ in self._series_def:
            raw = self._values.get(name, [])
            if len(raw) < 2:
                self.canvas.coords(self._lines[name], 0, 0, 0, 0)
                continue
            pts = raw[-self._history_size :]
            step = width / max(1, len(pts) - 1)
            coords: list[float] = []
            for idx, value in enumerate(pts):
                x = idx * step
                ratio = (float(value) - y_min) / max(y_max - y_min, 1e-6)
                y = height - ratio * height
                coords.extend((x, y))
            self.canvas.coords(self._lines[name], *coords)


class Mc02GuiWorker(threading.Thread):
    """后台串口线程。

    串口线程只处理收发和控制节拍，不直接操作任何 Tk 控件。
    """

    def __init__(self, *, port: str, baudrate: int, event_queue: "queue.Queue[dict[str, object]]") -> None:
        super().__init__(daemon=True)
        self._port = port
        self._baudrate = baudrate
        self._event_queue = event_queue
        self._cmd_queue: "queue.Queue[dict[str, object]]" = queue.Queue()
        self._stop_event = threading.Event()
        self._tx_seq = 0
        self._manual_cfg: ManualMitConfig | None = None
        self._manual_active = False
        self._next_manual_send = 0.0
        self._sine_cfg: SineConfig | None = None
        self._sine_active = False
        self._sine_t0 = 0.0
        self._next_sine_send = 0.0

    def stop(self) -> None:
        self._stop_event.set()

    def send_enable(self, *, motor_id: int, enable: bool, clear_fault: bool) -> None:
        self._cmd_queue.put({"type": "enable", "motor_id": motor_id, "enable": enable, "clear_fault": clear_fault})

    def send_mit(self, *, motor_id: int, pos_rad: float, vel_rad_s: float, torque_nm: float, kp: float, kd: float) -> None:
        self._cmd_queue.put(
            {
                "type": "mit",
                "motor_id": motor_id,
                "pos_rad": pos_rad,
                "vel_rad_s": vel_rad_s,
                "torque_nm": torque_nm,
                "kp": kp,
                "kd": kd,
            }
        )

    def stop_output(self) -> None:
        self._cmd_queue.put({"type": "stop_output"})

    def start_sine(self, cfg: SineConfig) -> None:
        self._cmd_queue.put({"type": "sine_start", "cfg": cfg})

    def stop_sine(self) -> None:
        self._cmd_queue.put({"type": "sine_stop"})

    def run(self) -> None:
        link: Mc02SerialCdcLink | None = None
        try:
            link = Mc02SerialCdcLink(port=self._port, baudrate=self._baudrate, timeout=0.003)
            link.open()
            self._emit("status", connected=True, port=str(link.resolved_port or self._port), message="串口已连接")
            self._emit("log", text=f"已连接串口 {link.resolved_port or self._port}")
            while not self._stop_event.is_set():
                self._drain_commands(link)
                self._maybe_send_manual(link)
                self._maybe_send_sine(link)
                frames = link.read_frames()
                if not frames:
                    time.sleep(0.001)
                    continue
                for frame in frames:
                    self._handle_frame(link, frame)
        except Exception as exc:
            self._emit("status", connected=False, port="--", message=f"串口异常: {exc}")
            self._emit("log", text=f"串口异常: {exc}")
        finally:
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
            self._emit("status", connected=False, port="--", message="串口已断开")

    def _drain_commands(self, link: Mc02SerialCdcLink) -> None:
        while True:
            try:
                item = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            typ = str(item["type"])
            if typ == "enable":
                self._manual_active = False
                self._sine_active = False
                self._send_enable(link, int(item["motor_id"]), bool(item["enable"]), bool(item["clear_fault"]))
            elif typ == "mit":
                self._sine_active = False
                self._manual_cfg = ManualMitConfig(
                    motor_id=int(item["motor_id"]),
                    pos_rad=float(item["pos_rad"]),
                    vel_rad_s=float(item["vel_rad_s"]),
                    torque_nm=float(item["torque_nm"]),
                    kp=float(item["kp"]),
                    kd=float(item["kd"]),
                )
                self._manual_active = True
                self._next_manual_send = 0.0
                self._emit("log", text="手动 MIT 持续发送已启动")
            elif typ == "stop_output":
                self._manual_active = False
                self._sine_active = False
                self._emit("log", text="持续发送已停止")
            elif typ == "sine_start":
                self._manual_active = False
                self._sine_cfg = item["cfg"]  # type: ignore[assignment]
                self._sine_active = True
                self._sine_t0 = time.monotonic()
                self._next_sine_send = 0.0
                self._emit("log", text="正弦测试已启动")
            elif typ == "sine_stop":
                self._sine_active = False
                self._emit("log", text="正弦测试已停止")

    def _maybe_send_manual(self, link: Mc02SerialCdcLink) -> None:
        if not self._manual_active or self._manual_cfg is None:
            return
        now = time.monotonic()
        if now < self._next_manual_send:
            return
        self._send_mit(
            link,
            motor_id=self._manual_cfg.motor_id,
            pos_rad=self._manual_cfg.pos_rad,
            vel_rad_s=self._manual_cfg.vel_rad_s,
            torque_nm=self._manual_cfg.torque_nm,
            kp=self._manual_cfg.kp,
            kd=self._manual_cfg.kd,
            mode="手动 MIT",
            emit_event=False,
        )
        self._next_manual_send = now + 0.02

    def _maybe_send_sine(self, link: Mc02SerialCdcLink) -> None:
        if not self._sine_active or self._sine_cfg is None:
            return
        now = time.monotonic()
        if now < self._next_sine_send:
            return
        elapsed = now - self._sine_t0
        omega = 2.0 * math.pi * self._sine_cfg.frequency_hz
        pos = self._sine_cfg.offset_rad + self._sine_cfg.amplitude_rad * math.sin(omega * elapsed)
        vel = self._sine_cfg.amplitude_rad * omega * math.cos(omega * elapsed)
        self._send_mit(
            link,
            motor_id=self._sine_cfg.motor_id,
            pos_rad=pos,
            vel_rad_s=vel,
            torque_nm=self._sine_cfg.torque_bias_nm,
            kp=self._sine_cfg.kp,
            kd=self._sine_cfg.kd,
            mode="正弦 MIT",
            emit_event=False,
        )
        self._next_sine_send = now + 0.02

    def _send_enable(self, link: Mc02SerialCdcLink, motor_id: int, enable: bool, clear_fault: bool) -> None:
        frame = A5UsbFrame(
            seq=self._next_seq(),
            cmd_id=int(UsbPcCommandId.MOTOR_ENABLE),
            payload=MotorEnableCommandPayload(
                motor_id=motor_id,
                enable_flag=1 if enable else 0,
                clear_fault_flag=1 if clear_fault else 0,
            ).to_bytes(),
        )
        link.write_frame(frame)
        self._emit(
            "tx",
            mode="使能" if enable else "失能",
            pos_rad=0.0,
            vel_rad_s=0.0,
            torque_nm=0.0,
            kp=0.0,
            kd=0.0,
            frame=decode_frame_to_dict(frame),
        )

    def _send_mit(
        self,
        link: Mc02SerialCdcLink,
        *,
        motor_id: int,
        pos_rad: float,
        vel_rad_s: float,
        torque_nm: float,
        kp: float,
        kd: float,
        mode: str,
        emit_event: bool = True,
    ) -> None:
        frame = A5UsbFrame(
            seq=self._next_seq(),
            cmd_id=int(USB_PC_FAST_MIT_CONTROL_CMD_ID),
            payload=MitControlCommandPayload(
                motor_id=motor_id,
                kp=kp,
                kd=kd,
                pos_rad=pos_rad,
                vel_rad_s=vel_rad_s,
                torque_nm=torque_nm,
            ).to_bytes(),
        )
        link.write_frame(frame)
        if emit_event:
            self._emit(
                "tx",
                mode=mode,
                pos_rad=pos_rad,
                vel_rad_s=vel_rad_s,
                torque_nm=torque_nm,
                kp=kp,
                kd=kd,
                frame=decode_frame_to_dict(frame),
            )

    def _handle_frame(self, link: Mc02SerialCdcLink, frame: A5UsbFrame) -> None:
        if int(frame.cmd_id) == int(UsbPcCommandId.SIMPLE_POLE_STATE):
            payload = SimplePoleStatePayload.from_bytes(frame.payload)
            self._emit(
                "telemetry",
                seq=int(frame.seq),
                timestamp=time.monotonic(),
                base_pos_rad=float(payload.base_pos_rad),
                base_vel_rad_s=float(payload.base_vel_rad_s),
                feedback_torque_nm=float(payload.motor_torque_nm),
                flex_pos_rad=float(payload.flex_pos_rad),
                flex_vel_rad_s=float(payload.flex_vel_rad_s),
                frame=decode_frame_to_dict(frame),
            )
            return
        if int(frame.cmd_id) == int(UsbPcCommandId.HEARTBEAT_REQ) and len(frame.payload) == 8:
            request = HeartbeatPayload.from_bytes(frame.payload)
            ack = A5UsbFrame(
                seq=self._next_seq(),
                cmd_id=int(UsbPcCommandId.HEARTBEAT_ACK),
                payload=HeartbeatPayload(
                    tick_ms=request.tick_ms,
                    protocol_version=request.protocol_version,
                    status_flags=request.status_flags | USB_PC_STATUS_USB_LINK_OK,
                ).to_bytes(),
            )
            link.write_frame(ack)
            self._emit("log", text="已回复板端心跳 ACK")
            return
        self._emit("log", text="收到其他命令帧:\n" + json.dumps(decode_frame_to_dict(frame), ensure_ascii=False, indent=2))

    def _next_seq(self) -> int:
        value = self._tx_seq
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        return value

    def _emit(self, typ: str, **payload: object) -> None:
        self._event_queue.put({"type": typ, **payload})


class Mc02GuiApp:
    """GUI 主体。

    设计原则：
    1. 串口线程全速收包。
    2. 主线程按固定节拍刷新。
    3. 点击按钮后立即更新界面，再异步发串口。
    """

    def __init__(self, root: tk.Tk, *, port: str, baudrate: int, history_size: int) -> None:
        self.root = root
        self.port = port
        self.baudrate = baudrate
        self.history_size = max(120, history_size)
        self.worker: Mc02GuiWorker | None = None
        self.events: "queue.Queue[dict[str, object]]" = queue.Queue()

        self.connected = False
        self.connected_port = "--"
        self.status_text = "等待连接"
        self.telemetry = TelemetryState()
        self.command = CommandState()
        self.frame_times: deque[float] = deque(maxlen=240)
        self.logs: deque[str] = deque(maxlen=120)
        self.last_rx_frame: dict[str, object] | None = None
        self.last_tx_frame: dict[str, object] | None = None

        self.h_base_pos: deque[float] = deque(maxlen=self.history_size)
        self.h_flex_pos: deque[float] = deque(maxlen=self.history_size)
        self.h_base_vel: deque[float] = deque(maxlen=self.history_size)
        self.h_flex_vel: deque[float] = deque(maxlen=self.history_size)
        self.h_feedback_torque: deque[float] = deque(maxlen=self.history_size)
        self.h_command_torque: deque[float] = deque(maxlen=self.history_size)

        self.var_motor_id = tk.StringVar(value="1")
        self.var_manual_pos = tk.StringVar(value="0.00")
        self.var_manual_vel = tk.StringVar(value="0.00")
        self.var_manual_torque = tk.StringVar(value="0.00")
        self.var_manual_kp = tk.StringVar(value="8.0")
        self.var_manual_kd = tk.StringVar(value="0.2")
        self.var_sine_amp = tk.StringVar(value="0.08")
        self.var_sine_freq = tk.StringVar(value="0.50")
        self.var_sine_offset = tk.StringVar(value="0.00")
        self.var_sine_torque = tk.StringVar(value="0.00")
        self.var_sine_kp = tk.StringVar(value="8.0")
        self.var_sine_kd = tk.StringVar(value="0.2")

        self.ui_dirty = True
        self.log_dirty = True
        self.last_log_flush = 0.0

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(10, self._poll_events)
        self.root.after(33, self._refresh_ui)
        self.root.after(120, self._connect)

    def _build_ui(self) -> None:
        self.root.title("MC02 L1 Debug Console")
        self.root.geometry("1420x860")
        self.root.minsize(1180, 720)
        self.root.configure(bg=THEME["bg"])

        header = tk.Frame(self.root, bg=THEME["bg"])
        header.pack(fill="x", padx=18, pady=(14, 10))
        tk.Label(header, text="MC02 L1 实机联调控制台", bg=THEME["bg"], fg=THEME["text"], font=("Microsoft YaHei UI", 18, "bold")).pack(side="left")
        self.status_label = tk.Label(header, text="● 等待连接", bg=THEME["bg"], fg=THEME["warning"], font=("Microsoft YaHei UI", 11, "bold"))
        self.status_label.pack(side="right", padx=(12, 0))
        self.substatus = tk.Label(header, text="端口 -- | RX 0.0 Hz | 模式 待机", bg=THEME["bg"], fg=THEME["text_dim"], font=("Microsoft YaHei UI", 10))
        self.substatus.pack(side="right")

        main = tk.Frame(self.root, bg=THEME["bg"])
        main.pack(fill="both", expand=True, padx=18, pady=(0, 18))
        main.grid_columnconfigure(0, weight=0, minsize=330)
        main.grid_columnconfigure(1, weight=1)
        main.grid_rowconfigure(0, weight=1)

        left_shell = tk.Frame(main, bg=THEME["bg"])
        right = tk.Frame(main, bg=THEME["bg"])
        left_shell.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        right.grid(row=0, column=1, sticky="nsew")
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)

        left_shell.grid_rowconfigure(0, weight=1)
        left_shell.grid_columnconfigure(0, weight=1)
        self.left_canvas = tk.Canvas(left_shell, bg=THEME["bg"], highlightthickness=0, width=332)
        self.left_scrollbar = tk.Scrollbar(left_shell, orient="vertical", command=self.left_canvas.yview)
        self.left_canvas.configure(yscrollcommand=self.left_scrollbar.set)
        self.left_canvas.grid(row=0, column=0, sticky="nsew")
        self.left_scrollbar.grid(row=0, column=1, sticky="ns")
        self.left_content = tk.Frame(self.left_canvas, bg=THEME["bg"])
        self.left_window = self.left_canvas.create_window((0, 0), window=self.left_content, anchor="nw")
        self.left_content.bind("<Configure>", self._on_left_content_configure)
        self.left_canvas.bind("<Configure>", self._on_left_canvas_configure)
        self.left_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        self._build_left(self.left_content)
        self._build_right(right)

    def _on_left_content_configure(self, _event: tk.Event[tk.Frame]) -> None:
        self.left_canvas.configure(scrollregion=self.left_canvas.bbox("all"))

    def _on_left_canvas_configure(self, event: tk.Event[tk.Canvas]) -> None:
        self.left_canvas.itemconfigure(self.left_window, width=event.width)

    def _on_mousewheel(self, event: tk.Event[tk.Widget]) -> None:
        widget = self.root.winfo_containing(self.root.winfo_pointerx(), self.root.winfo_pointery())
        if widget is None:
            return
        current = widget
        while current is not None:
            if current == self.left_canvas or current == self.left_content:
                delta = -1 if event.delta > 0 else 1
                self.left_canvas.yview_scroll(delta, "units")
                return
            current = current.master

    def _section(self, master: tk.Misc, title: str) -> tk.Frame:
        frame = tk.Frame(master, bg=THEME["panel"], highlightthickness=1, highlightbackground=THEME["border"])
        frame.pack(fill="x", pady=(0, 10))
        tk.Label(frame, text=title, bg=THEME["panel"], fg=THEME["text"], font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w", padx=12, pady=(10, 6))
        return frame

    def _entry_row(self, master: tk.Misc, label: str, *, variable: tk.StringVar | None = None, value: str | None = None, readonly: bool = False) -> None:
        row = tk.Frame(master, bg=THEME["panel"])
        row.pack(fill="x", padx=12, pady=3)
        tk.Label(row, text=label, width=12, anchor="w", bg=THEME["panel"], fg=THEME["text_dim"], font=("Microsoft YaHei UI", 9)).pack(side="left")
        entry = tk.Entry(row, textvariable=variable, bg=THEME["panel_alt"], fg=THEME["text"], bd=0, relief="flat", insertbackground=THEME["text"], font=("Consolas", 10))
        if value is not None and variable is None:
            entry.insert(0, value)
        if readonly:
            entry.configure(state="disabled", disabledbackground=THEME["panel_alt"], disabledforeground=THEME["text_dim"])
        entry.pack(side="right", fill="x", expand=True, padx=(8, 0), ipady=4)

    def _button(self, master: tk.Misc, text: str, command, color: str) -> tk.Button:
        return tk.Button(master, text=text, command=command, bg=color, fg="#08111F", activebackground=color, activeforeground="#08111F", bd=0, relief="flat", font=("Microsoft YaHei UI", 9, "bold"), cursor="hand2", padx=8, pady=6)

    def _build_left(self, master: tk.Misc) -> None:
        sec = self._section(master, "连接控制")
        self._entry_row(sec, "端口", value=self.port, readonly=True)
        self._entry_row(sec, "波特率", value=str(self.baudrate), readonly=True)
        self._entry_row(sec, "电机 ID", variable=self.var_motor_id)
        row = tk.Frame(sec, bg=THEME["panel"])
        row.pack(fill="x", padx=12, pady=(8, 6))
        self._button(row, "连接", self._connect, THEME["accent"]).pack(side="left", expand=True, fill="x")
        self._button(row, "断开", self._disconnect, THEME["danger"]).pack(side="left", expand=True, fill="x", padx=(8, 0))
        row = tk.Frame(sec, bg=THEME["panel"])
        row.pack(fill="x", padx=12, pady=(0, 12))
        self._button(row, "使能", self._enable, THEME["success"]).pack(side="left", expand=True, fill="x")
        self._button(row, "失能", self._disable, THEME["warning"]).pack(side="left", expand=True, fill="x", padx=(8, 0))
        self._button(row, "清故障使能", self._clear_and_enable, THEME["violet"]).pack(side="left", expand=True, fill="x", padx=(8, 0))

        sec = self._section(master, "手动 MIT 控制")
        self._entry_row(sec, "目标位置 rad", variable=self.var_manual_pos)
        self._entry_row(sec, "目标速度 rad/s", variable=self.var_manual_vel)
        self._entry_row(sec, "目标力矩 Nm", variable=self.var_manual_torque)
        self._entry_row(sec, "Kp", variable=self.var_manual_kp)
        self._entry_row(sec, "Kd", variable=self.var_manual_kd)
        row = tk.Frame(sec, bg=THEME["panel"])
        row.pack(fill="x", padx=12, pady=(8, 12))
        self._button(row, "启动持续 MIT", self._manual_mit, THEME["accent"]).pack(side="left", expand=True, fill="x")
        self._button(row, "零位保持", self._zero_mit, THEME["panel_alt"]).pack(side="left", expand=True, fill="x", padx=(8, 0))
        self._button(row, "停止发送", self._stop_output, THEME["danger"]).pack(side="left", expand=True, fill="x", padx=(8, 0))

        sec = self._section(master, "正弦测试")
        self._entry_row(sec, "振幅 rad", variable=self.var_sine_amp)
        self._entry_row(sec, "频率 Hz", variable=self.var_sine_freq)
        self._entry_row(sec, "偏置 rad", variable=self.var_sine_offset)
        self._entry_row(sec, "力矩偏置 Nm", variable=self.var_sine_torque)
        self._entry_row(sec, "Kp", variable=self.var_sine_kp)
        self._entry_row(sec, "Kd", variable=self.var_sine_kd)
        row = tk.Frame(sec, bg=THEME["panel"])
        row.pack(fill="x", padx=12, pady=(8, 12))
        self._button(row, "启动正弦", self._start_sine, THEME["success"]).pack(side="left", expand=True, fill="x")
        self._button(row, "停止正弦", self._stop_sine, THEME["danger"]).pack(side="left", expand=True, fill="x", padx=(8, 0))

    def _build_right(self, master: tk.Misc) -> None:
        cards = tk.Frame(master, bg=THEME["bg"])
        cards.grid(row=0, column=0, sticky="ew")
        for i in range(3):
            cards.grid_columnconfigure(i, weight=1)
        self.card_base_pos = MetricCard(cards, "固定杆角度 (rad)", THEME["accent"])
        self.card_base_vel = MetricCard(cards, "固定杆角速度 (rad/s)", THEME["accent"])
        self.card_flex_pos = MetricCard(cards, "活动杆角度 (rad)", THEME["violet"])
        self.card_flex_vel = MetricCard(cards, "活动杆角速度 (rad/s)", THEME["violet"])
        self.card_feedback_torque = MetricCard(cards, "回传力矩 (Nm)", THEME["warning"])
        self.card_command_torque = MetricCard(cards, "指令力矩 (Nm)", THEME["success"])
        items = [self.card_base_pos, self.card_base_vel, self.card_flex_pos, self.card_flex_vel, self.card_feedback_torque, self.card_command_torque]
        for idx, item in enumerate(items):
            r, c = divmod(idx, 3)
            item.grid(row=r, column=c, sticky="nsew", padx=(0 if c == 0 else 8, 0), pady=(0 if r == 0 else 8, 0))

        body = tk.Frame(master, bg=THEME["bg"])
        body.grid(row=1, column=0, sticky="nsew", pady=(12, 0))
        body.grid_rowconfigure(0, weight=3)
        body.grid_rowconfigure(1, weight=2)
        body.grid_columnconfigure(0, weight=1)

        charts = tk.Frame(body, bg=THEME["bg"])
        charts.grid(row=0, column=0, sticky="nsew")
        charts.grid_columnconfigure(0, weight=1)
        charts.grid_rowconfigure(0, weight=1)
        charts.grid_rowconfigure(1, weight=1)
        charts.grid_rowconfigure(2, weight=1)
        self.chart_angle = StripChart(charts, "角度曲线", [("固定杆", THEME["accent"]), ("活动杆", THEME["violet"])], self.history_size)
        self.chart_vel = StripChart(charts, "角速度曲线", [("固定杆", THEME["accent"]), ("活动杆", THEME["violet"])], self.history_size)
        self.chart_torque = StripChart(charts, "力矩曲线", [("回传力矩", THEME["warning"]), ("指令力矩", THEME["success"])], self.history_size)
        self.chart_angle.grid(row=0, column=0, sticky="nsew")
        self.chart_vel.grid(row=1, column=0, sticky="nsew", pady=(10, 10))
        self.chart_torque.grid(row=2, column=0, sticky="nsew")

        log_panel = tk.Frame(body, bg=THEME["panel"], highlightthickness=1, highlightbackground=THEME["border"])
        log_panel.grid(row=1, column=0, sticky="nsew", pady=(12, 0))
        log_panel.grid_rowconfigure(1, weight=1)
        log_panel.grid_columnconfigure(0, weight=1)
        tk.Label(log_panel, text="链路消息", bg=THEME["panel"], fg=THEME["text"], font=("Microsoft YaHei UI", 11, "bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(10, 8))
        self.log_text = tk.Text(log_panel, bg=THEME["panel_alt"], fg=THEME["text"], insertbackground=THEME["text"], relief="flat", wrap="word", font=("Consolas", 10))
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 12))
        self.log_text.configure(state="disabled")

    def _connect(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        self.worker = Mc02GuiWorker(port=self.port, baudrate=self.baudrate, event_queue=self.events)
        self.worker.start()
        self._append_log("正在连接串口...")

    def _disconnect(self) -> None:
        if self.worker is not None:
            self.worker.stop()
        self.connected = False
        self.status_text = "正在断开串口"
        self.ui_dirty = True
        self._append_log("已请求断开串口")

    def _ensure_worker(self) -> Mc02GuiWorker:
        if self.worker is None or not self.worker.is_alive():
            self._connect()
        assert self.worker is not None
        return self.worker

    def _enable(self) -> None:
        self._ensure_worker().send_enable(motor_id=self._int(self.var_motor_id, 1), enable=True, clear_fault=False)

    def _disable(self) -> None:
        self._ensure_worker().send_enable(motor_id=self._int(self.var_motor_id, 1), enable=False, clear_fault=False)

    def _clear_and_enable(self) -> None:
        self._ensure_worker().send_enable(motor_id=self._int(self.var_motor_id, 1), enable=True, clear_fault=True)

    def _manual_mit(self) -> None:
        pos = self._float(self.var_manual_pos, 0.0)
        vel = self._float(self.var_manual_vel, 0.0)
        tor = self._float(self.var_manual_torque, 0.0)
        kp = self._float(self.var_manual_kp, 8.0)
        kd = self._float(self.var_manual_kd, 0.2)
        self.command = CommandState(timestamp=time.monotonic(), pos_rad=pos, vel_rad_s=vel, torque_nm=tor, kp=kp, kd=kd, mode="手动 MIT")
        self.ui_dirty = True
        self._ensure_worker().send_mit(motor_id=self._int(self.var_motor_id, 1), pos_rad=pos, vel_rad_s=vel, torque_nm=tor, kp=kp, kd=kd)

    def _zero_mit(self) -> None:
        self.var_manual_pos.set("0.00")
        self.var_manual_vel.set("0.00")
        self.var_manual_torque.set("0.00")
        self._manual_mit()

    def _start_sine(self) -> None:
        cfg = SineConfig(
            motor_id=self._int(self.var_motor_id, 1),
            amplitude_rad=self._float(self.var_sine_amp, 0.08),
            frequency_hz=self._float(self.var_sine_freq, 0.5),
            offset_rad=self._float(self.var_sine_offset, 0.0),
            torque_bias_nm=self._float(self.var_sine_torque, 0.0),
            kp=self._float(self.var_sine_kp, 8.0),
            kd=self._float(self.var_sine_kd, 0.2),
        )
        self.command = CommandState(timestamp=time.monotonic(), pos_rad=cfg.offset_rad, vel_rad_s=0.0, torque_nm=cfg.torque_bias_nm, kp=cfg.kp, kd=cfg.kd, mode="正弦 MIT")
        self.ui_dirty = True
        self._ensure_worker().start_sine(cfg)

    def _stop_sine(self) -> None:
        self.command.mode = "待机"
        self.ui_dirty = True
        self._ensure_worker().stop_sine()

    def _stop_output(self) -> None:
        self.command.mode = "待机"
        self.ui_dirty = True
        self._ensure_worker().stop_output()

    def _poll_events(self) -> None:
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            typ = str(event["type"])
            if typ == "status":
                self.connected = bool(event["connected"])
                self.connected_port = str(event["port"])
                self.status_text = str(event["message"])
                if not self.connected and self.worker is not None and not self.worker.is_alive():
                    self.worker = None
                self.ui_dirty = True
            elif typ == "telemetry":
                self.telemetry = TelemetryState(
                    timestamp=float(event["timestamp"]),
                    seq=int(event["seq"]),
                    base_pos_rad=float(event["base_pos_rad"]),
                    base_vel_rad_s=float(event["base_vel_rad_s"]),
                    feedback_torque_nm=float(event["feedback_torque_nm"]),
                    flex_pos_rad=float(event["flex_pos_rad"]),
                    flex_vel_rad_s=float(event["flex_vel_rad_s"]),
                )
                self.frame_times.append(self.telemetry.timestamp)
                self.last_rx_frame = event["frame"]  # type: ignore[assignment]
                self.h_base_pos.append(self.telemetry.base_pos_rad)
                self.h_flex_pos.append(self.telemetry.flex_pos_rad)
                self.h_base_vel.append(self.telemetry.base_vel_rad_s)
                self.h_flex_vel.append(self.telemetry.flex_vel_rad_s)
                self.h_feedback_torque.append(self.telemetry.feedback_torque_nm)
                self.h_command_torque.append(self.command.torque_nm)
                self.ui_dirty = True
            elif typ == "tx":
                self.command = CommandState(
                    timestamp=time.monotonic(),
                    pos_rad=float(event["pos_rad"]),
                    vel_rad_s=float(event["vel_rad_s"]),
                    torque_nm=float(event["torque_nm"]),
                    kp=float(event["kp"]),
                    kd=float(event["kd"]),
                    mode=str(event["mode"]),
                )
                self.last_tx_frame = event["frame"]  # type: ignore[assignment]
                self._append_log(
                    f"已发送 {self.command.mode}: pos={self.command.pos_rad:+.3f}, vel={self.command.vel_rad_s:+.3f}, tor={self.command.torque_nm:+.3f}"
                )
                self.ui_dirty = True
            elif typ == "log":
                self._append_log(str(event["text"]))
        self.root.after(10, self._poll_events)

    def _refresh_ui(self) -> None:
        now = time.monotonic()
        while self.frame_times and now - self.frame_times[0] > 1.0:
            self.frame_times.popleft()
        fps = float(len(self.frame_times))
        alive = self.connected and (now - self.telemetry.timestamp) < 0.25
        if alive:
            self.status_label.configure(text="● 链路活跃", fg=THEME["success"])
        elif self.connected:
            self.status_label.configure(text="● 已连接，等待数据", fg=THEME["warning"])
        else:
            self.status_label.configure(text="● 未连接", fg=THEME["danger"])
        self.substatus.configure(text=f"端口 {self.connected_port} | RX {fps:.1f} Hz | 模式 {self.command.mode} | 状态 {self.status_text}")

        if self.ui_dirty:
            self.card_base_pos.set(f"{self.telemetry.base_pos_rad:+.4f}")
            self.card_base_vel.set(f"{self.telemetry.base_vel_rad_s:+.4f}")
            self.card_flex_pos.set(f"{self.telemetry.flex_pos_rad:+.4f}")
            self.card_flex_vel.set(f"{self.telemetry.flex_vel_rad_s:+.4f}")
            self.card_feedback_torque.set(f"{self.telemetry.feedback_torque_nm:+.4f}")
            self.card_command_torque.set(f"{self.command.torque_nm:+.4f}")
            self.chart_angle.update({"固定杆": list(self.h_base_pos), "活动杆": list(self.h_flex_pos)})
            self.chart_vel.update({"固定杆": list(self.h_base_vel), "活动杆": list(self.h_flex_vel)})
            self.chart_torque.update({"回传力矩": list(self.h_feedback_torque), "指令力矩": list(self.h_command_torque)})
            self.ui_dirty = False

        if self.log_dirty and (now - self.last_log_flush) > 0.12:
            parts = list(self.logs)
            if self.last_rx_frame is not None:
                parts.extend(["", "最近一帧下位机回传:", json.dumps(self.last_rx_frame, ensure_ascii=False, indent=2)])
            if self.last_tx_frame is not None:
                parts.extend(["", "最近一帧上位机发送:", json.dumps(self.last_tx_frame, ensure_ascii=False, indent=2)])
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.insert("1.0", "\n".join(parts))
            self.log_text.configure(state="disabled")
            self.log_text.see("end")
            self.log_dirty = False
            self.last_log_flush = now

        self.root.after(33, self._refresh_ui)

    def _append_log(self, text: str) -> None:
        self.logs.append(f"[{time.strftime('%H:%M:%S')}] {text}")
        self.log_dirty = True

    def _on_close(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.left_canvas.unbind_all("<MouseWheel>")
        self.root.destroy()

    @staticmethod
    def _float(var: tk.StringVar, default: float) -> float:
        try:
            return float(var.get().strip())
        except Exception:
            return float(default)

    @staticmethod
    def _int(var: tk.StringVar, default: int) -> int:
        try:
            return int(var.get().strip(), 0)
        except Exception:
            return int(default)


def launch_mc02_gui(*, port: str, baudrate: int, history_size: int) -> int:
    """启动单进程 GUI。"""

    root = tk.Tk()
    Mc02GuiApp(root, port=port, baudrate=baudrate, history_size=history_size)
    root.mainloop()
    return 0
