from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Sequence

from .history import ObservationHistory, build_history_frame
from .mc02_bridge import (
    position_delta_command_from_action,
    sensor_frame_from_l1_sensor,
    sensor_frame_from_simple_pole_state,
)
from .mc02_usb_protocol import (
    A5UsbFrame,
    HeartbeatPayload,
    L1SensorStatePayload,
    MitControlCommandPayload,
    MotorEnableCommandPayload,
    SimplePoleStatePayload,
    USB_PC_FAST_MIT_CONTROL_CMD_ID,
    USB_PC_STATUS_USB_LINK_OK,
    UsbPcCommandId,
    decode_frame_to_dict,
)
from .mc02_usb_transport import Mc02SerialCdcLink
from .policy import load_policy
from .protocol import ActionFrameV1, SensorFrameV1
from .replay import replay_log
from .runtime import HostRuntime, HostRuntimeConfig
from .session_log import JsonlLogger
from .transport import SerialCdcLink

Mc02A5SerialCdcLink = Mc02SerialCdcLink


def _parse_int_auto(value: str) -> int:
    """允许命令行同时接受十进制和 0x 前缀十六进制。"""

    return int(value, 0)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Host-side runtime for sim2real.")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    policy_parser = subparsers.add_parser("policy", help="Run policy-driven control.")
    policy_parser.add_argument("--policy", required=True, help="Path to .onnx or TorchScript model.")
    policy_parser.add_argument("--port", help="USB CDC serial port.")
    policy_parser.add_argument("--baudrate", type=int, default=921600)
    policy_parser.add_argument("--steps", type=int, default=0, help="Offline synthetic steps when --port is omitted.")
    policy_parser.add_argument("--log", help="Optional JSONL log output path.")
    policy_parser.add_argument("--kp", type=float, default=1.0)
    policy_parser.add_argument("--kd", type=float, default=0.0)
    policy_parser.add_argument("--watchdog-ms", type=int, default=100)
    policy_parser.add_argument("--mode-flags", type=int, default=1)
    policy_parser.add_argument("--action-limit-rad", type=float, default=None)
    policy_parser.add_argument(
        "--transport-protocol",
        choices=("legacy-fixed", "mc02-a5"),
        default="legacy-fixed",
        help="legacy-fixed 使用旧固定帧协议，mc02-a5 使用当前板端 A5 USB 协议。",
    )
    policy_parser.add_argument(
        "--motor-id",
        type=int,
        default=1,
        help="当 --transport-protocol=mc02-a5 时使用的电机 id。",
    )

    sine_parser = subparsers.add_parser("sine", help="Generate a sine-wave reference.")
    sine_parser.add_argument("--port", help="USB CDC serial port.")
    sine_parser.add_argument("--baudrate", type=int, default=921600)
    sine_parser.add_argument("--steps", type=int, default=0, help="Offline synthetic steps when --port is omitted.")
    sine_parser.add_argument("--log", help="Optional JSONL log output path.")
    sine_parser.add_argument("--amplitude", type=float, default=0.1)
    sine_parser.add_argument("--frequency-hz", type=float, default=0.5)
    sine_parser.add_argument("--kp", type=float, default=1.0)
    sine_parser.add_argument("--kd", type=float, default=0.0)
    sine_parser.add_argument("--watchdog-ms", type=int, default=100)
    sine_parser.add_argument("--mode-flags", type=int, default=1)

    enable_parser = subparsers.add_parser("enable", help="Send one motor-enable command.")
    enable_parser.add_argument("--port", default="auto")
    enable_parser.add_argument("--baudrate", type=int, default=921600)
    enable_parser.add_argument("--disable", action="store_true", help="Send disable instead of enable.")
    enable_parser.add_argument("--clear-fault", action="store_true")
    enable_parser.add_argument("--motor-id", type=int, default=1)

    mit_parser = subparsers.add_parser("mit", help="Send one MIT command.")
    mit_parser.add_argument("--port", default="auto")
    mit_parser.add_argument("--baudrate", type=int, default=921600)
    mit_parser.add_argument("--motor-id", type=int, default=1)
    mit_parser.add_argument("--kp", type=float, required=True)
    mit_parser.add_argument("--kd", type=float, required=True)
    mit_parser.add_argument("--pos-rad", type=float, required=True)
    mit_parser.add_argument("--vel-rad-s", type=float, default=0.0)
    mit_parser.add_argument("--torque-nm", type=float, default=0.0)

    # 下面三个命令是为当前 MC02 USB CDC A5 协议准备的。
    # 它们直接匹配你现在板端已经跑通的消息格式，不要求你回改下位机。
    mc02_policy_parser = subparsers.add_parser(
        "mc02-policy",
        help="Run ONNX/TorchScript policy over current MC02 A5 USB CDC protocol.",
    )
    mc02_policy_parser.add_argument("--policy", required=True, help="Path to .onnx or TorchScript model.")
    mc02_policy_parser.add_argument(
        "--port",
        default="auto",
        help="MC02 virtual COM port. 留空或填 auto 时会自动识别 STM 虚拟串口。",
    )
    mc02_policy_parser.add_argument("--baudrate", type=int, default=921600)
    mc02_policy_parser.add_argument("--motor-id", type=int, default=1, help="DM4310 motor id.")
    mc02_policy_parser.add_argument("--log", help="Optional JSONL log output path.")
    mc02_policy_parser.add_argument("--kp", type=float, default=1.0)
    mc02_policy_parser.add_argument("--kd", type=float, default=0.0)
    mc02_policy_parser.add_argument("--action-limit-rad", type=float, default=None)
    mc02_policy_parser.add_argument(
        "--auto-enable",
        action="store_true",
        help="Send one motor-enable command before entering control loop.",
    )
    mc02_policy_parser.add_argument(
        "--clear-fault-on-enable",
        action="store_true",
        help="Set clear_fault_flag=1 when --auto-enable is used.",
    )
    mc02_policy_parser.add_argument(
        "--disable-on-exit",
        action="store_true",
        help="Send one motor-disable command when program exits.",
    )
    mc02_policy_parser.add_argument(
        "--print-rx",
        action="store_true",
        help="Print non-L1 incoming frames for debugging.",
    )

    mc02_sine_parser = subparsers.add_parser(
        "mc02-sine",
        help="Send a sine-wave position-delta command over current MC02 A5 protocol.",
    )
    mc02_sine_parser.add_argument(
        "--port",
        default="auto",
        help="MC02 virtual COM port. 留空或填 auto 时会自动识别 STM 虚拟串口。",
    )
    mc02_sine_parser.add_argument("--baudrate", type=int, default=921600)
    mc02_sine_parser.add_argument("--motor-id", type=int, default=1)
    mc02_sine_parser.add_argument("--log", help="Optional JSONL log output path.")
    mc02_sine_parser.add_argument("--amplitude", type=float, default=0.1)
    mc02_sine_parser.add_argument("--frequency-hz", type=float, default=0.5)
    mc02_sine_parser.add_argument("--kp", type=float, default=1.0)
    mc02_sine_parser.add_argument("--kd", type=float, default=0.0)
    mc02_sine_parser.add_argument("--auto-enable", action="store_true")
    mc02_sine_parser.add_argument("--clear-fault-on-enable", action="store_true")
    mc02_sine_parser.add_argument("--disable-on-exit", action="store_true")
    mc02_sine_parser.add_argument("--print-rx", action="store_true")

    mc02_monitor_parser = subparsers.add_parser(
        "mc02-monitor",
        help="Print decoded MC02 USB CDC A5 frames for debugging.",
    )
    mc02_monitor_parser.add_argument(
        "--port",
        default="auto",
        help="MC02 virtual COM port. 留空或填 auto 时会自动识别 STM 虚拟串口。",
    )
    mc02_monitor_parser.add_argument("--baudrate", type=int, default=921600)
    mc02_monitor_parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after receiving this many valid frames. 0 means infinite.",
    )
    mc02_monitor_parser.add_argument(
        "--filter-cmd-id",
        type=_parse_int_auto,
        default=None,
        help="Only print one cmd_id, for example 0x0203.",
    )

    mc02_gui_parser = subparsers.add_parser(
        "mc02-gui",
        help="Open a single-process GUI for MC02 telemetry and MIT testing.",
    )
    mc02_gui_parser.add_argument(
        "--port",
        default="auto",
        help="MC02 virtual COM port. 留空或填 auto 时会自动识别 STM 虚拟串口。",
    )
    mc02_gui_parser.add_argument("--baudrate", type=int, default=921600)
    mc02_gui_parser.add_argument(
        "--history-size",
        type=int,
        default=300,
        help="图形界面中每条曲线保留的历史点数。",
    )

    replay_parser = subparsers.add_parser("replay", help="Replay and validate a JSONL log.")
    replay_parser.add_argument("--log", required=True)
    replay_parser.add_argument("--policy", help="Optional policy path for action validation.")
    replay_parser.add_argument("--history-length", type=int, default=3)
    replay_parser.add_argument("--feature-dim", type=int, default=5)
    replay_parser.add_argument("--atol", type=float, default=1e-5)
    replay_parser.add_argument("--no-policy-check", action="store_true")

    args = parser.parse_args(argv)

    if args.mode == "replay":
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

    if args.mode == "policy":
        runtime = HostRuntime(
            HostRuntimeConfig(
                policy_path=args.policy,
                kp=args.kp,
                kd=args.kd,
                watchdog_ms=args.watchdog_ms,
                mode_flags=args.mode_flags,
                action_limit_rad=args.action_limit_rad,
            )
        )
        if args.transport_protocol == "mc02-a5":
            if args.port:
                return _run_mc02_policy_mode(
                    runtime=runtime,
                    port=args.port,
                    baudrate=args.baudrate,
                    motor_id=args.motor_id,
                    log_path=args.log,
                    auto_enable=False,
                    clear_fault_on_enable=False,
                    disable_on_exit=False,
                    print_rx=False,
                )
            if args.steps <= 0:
                raise ValueError("steps must be positive when --port is omitted")
            logger = JsonlLogger(Path(args.log)) if args.log else None
            if logger is not None:
                logger.open()
            try:
                return _run_policy_synthetic(runtime, steps=args.steps, logger=logger)
            finally:
                if logger is not None:
                    logger.close()
        return _run_policy_mode(runtime, port=args.port, baudrate=args.baudrate, steps=args.steps, log_path=args.log)

    if args.mode == "sine":
        return _run_sine_mode(
            port=args.port,
            baudrate=args.baudrate,
            steps=args.steps,
            log_path=args.log,
            amplitude=args.amplitude,
            frequency_hz=args.frequency_hz,
            kp=args.kp,
            kd=args.kd,
            watchdog_ms=args.watchdog_ms,
            mode_flags=args.mode_flags,
        )

    if args.mode == "enable":
        return _run_enable_mode(
            port=args.port,
            baudrate=args.baudrate,
            motor_id=args.motor_id,
            enable=not args.disable,
            clear_fault=args.clear_fault,
        )

    if args.mode == "mit":
        return _run_mit_mode(
            port=args.port,
            baudrate=args.baudrate,
            motor_id=args.motor_id,
            kp=args.kp,
            kd=args.kd,
            pos_rad=args.pos_rad,
            vel_rad_s=args.vel_rad_s,
            torque_nm=args.torque_nm,
        )

    if args.mode == "mc02-policy":
        runtime = HostRuntime(
            HostRuntimeConfig(
                policy_path=args.policy,
                kp=args.kp,
                kd=args.kd,
                action_limit_rad=args.action_limit_rad,
            )
        )
        return _run_mc02_policy_mode(
            runtime=runtime,
            port=args.port,
            baudrate=args.baudrate,
            motor_id=args.motor_id,
            log_path=args.log,
            auto_enable=args.auto_enable,
            clear_fault_on_enable=args.clear_fault_on_enable,
            disable_on_exit=args.disable_on_exit,
            print_rx=args.print_rx,
        )

    if args.mode == "mc02-sine":
        return _run_mc02_sine_mode(
            port=args.port,
            baudrate=args.baudrate,
            motor_id=args.motor_id,
            log_path=args.log,
            amplitude=args.amplitude,
            frequency_hz=args.frequency_hz,
            kp=args.kp,
            kd=args.kd,
            auto_enable=args.auto_enable,
            clear_fault_on_enable=args.clear_fault_on_enable,
            disable_on_exit=args.disable_on_exit,
            print_rx=args.print_rx,
        )

    if args.mode == "mc02-monitor":
        return _run_mc02_monitor_mode(
            port=args.port,
            baudrate=args.baudrate,
            max_frames=args.max_frames,
            filter_cmd_id=args.filter_cmd_id,
        )

    if args.mode == "mc02-gui":
        from .mc02_gui import launch_mc02_gui

        return launch_mc02_gui(
            port=args.port,
            baudrate=args.baudrate,
            history_size=args.history_size,
        )

    raise ValueError(f"unsupported mode: {args.mode}")


def _create_serial_link(port: str, baudrate: int) -> Mc02A5SerialCdcLink:
    """统一创建当前 MC02 A5 串口链路，便于测试时 monkeypatch。"""

    return Mc02A5SerialCdcLink(port=port, baudrate=baudrate)


def _run_policy_mode(
    runtime: HostRuntime,
    *,
    port: str | None,
    baudrate: int,
    steps: int,
    log_path: str | None,
) -> int:
    logger = JsonlLogger(Path(log_path)) if log_path else None
    if logger is not None:
        logger.open()

    try:
        if port:
            with SerialCdcLink(port=port, baudrate=baudrate) as link:
                return _run_policy_serial(runtime, link, logger)
        if steps <= 0:
            raise ValueError("steps must be positive when --port is omitted")
        return _run_policy_synthetic(runtime, steps=steps, logger=logger)
    finally:
        if logger is not None:
            logger.close()


def _run_policy_serial(runtime: HostRuntime, link: SerialCdcLink, logger: JsonlLogger | None) -> int:
    while True:
        frames = link.read_frames()
        for frame in frames:
            if not isinstance(frame, SensorFrameV1):
                continue
            previous_action = runtime.previous_action
            observation, action, model_output = runtime.step(frame)
            link.write_frame(action)
            if logger is not None:
                logger.log_cycle(
                    sensor=frame,
                    observation=observation,
                    action=action,
                    previous_action=previous_action,
                    observation_history=runtime.history.flatten(),
                    model_output=model_output,
                )
    return 0


def _run_policy_synthetic(
    runtime: HostRuntime,
    *,
    steps: int,
    logger: JsonlLogger | None,
) -> int:
    for index in range(steps):
        sensor = SensorFrameV1(
            seq=index + 1,
            t_us=(index + 1) * 4000,
            base_pos_rad=0.02 * math.sin(0.1 * index),
            base_vel_rad_s=0.02 * math.cos(0.1 * index),
            flex1_pos_rad=0.01 * math.sin(0.07 * index),
            flex1_vel_rad_s=0.01 * math.cos(0.07 * index),
            motor_temp_c=28.0,
            status_flags=0,
        )
        previous_action = runtime.previous_action
        observation, action, model_output = runtime.step(sensor)
        if logger is not None:
            logger.log_cycle(
                sensor=sensor,
                observation=observation,
                action=action,
                previous_action=previous_action,
                observation_history=runtime.history.flatten(),
                model_output=model_output,
            )
    return 0


def _run_sine_mode(
    *,
    port: str | None,
    baudrate: int,
    steps: int,
    log_path: str | None,
    amplitude: float,
    frequency_hz: float,
    kp: float,
    kd: float,
    watchdog_ms: int,
    mode_flags: int,
) -> int:
    logger = JsonlLogger(Path(log_path)) if log_path else None
    if logger is not None:
        logger.open()
    history = ObservationHistory()
    previous_action: ActionFrameV1 | None = None

    try:
        if port:
            with SerialCdcLink(port=port, baudrate=baudrate) as link:
                index = 0
                while True:
                    frames = link.read_frames()
                    for frame in frames:
                        if not isinstance(frame, SensorFrameV1):
                            continue
                        delta = amplitude * math.sin(2.0 * math.pi * frequency_hz * index * 0.004)
                        action = ActionFrameV1(
                            seq=frame.seq,
                            t_us=frame.t_us,
                            base_pos_delta_rad=delta,
                            kp=kp,
                            kd=kd,
                            watchdog_ms=watchdog_ms,
                            mode_flags=mode_flags,
                        )
                        link.write_frame(action)
                        features = build_history_frame(frame, previous_action)
                        history.push(features)
                        if logger is not None:
                            logger.log_cycle(
                                sensor=frame,
                                observation=history.flatten(),
                                action=action,
                                previous_action=previous_action,
                                observation_history=history.flatten(),
                                model_output=[delta],
                            )
                        previous_action = action
                        index += 1
            return 0

        if steps <= 0:
            raise ValueError("steps must be positive when --port is omitted")
        for index in range(steps):
            sensor = SensorFrameV1(
                seq=index + 1,
                t_us=(index + 1) * 4000,
                base_pos_rad=0.0,
                base_vel_rad_s=0.0,
                flex1_pos_rad=0.0,
                flex1_vel_rad_s=0.0,
                motor_temp_c=28.0,
                status_flags=0,
            )
            delta = amplitude * math.sin(2.0 * math.pi * frequency_hz * index * 0.004)
            action = ActionFrameV1(
                seq=sensor.seq,
                t_us=sensor.t_us,
                base_pos_delta_rad=delta,
                kp=kp,
                kd=kd,
                watchdog_ms=watchdog_ms,
                mode_flags=mode_flags,
            )
            features = build_history_frame(sensor, previous_action)
            history.push(features)
            if logger is not None:
                logger.log_cycle(
                    sensor=sensor,
                    observation=history.flatten(),
                    action=action,
                    previous_action=previous_action,
                    observation_history=history.flatten(),
                    model_output=[delta],
                )
            previous_action = action
        return 0
    finally:
        if logger is not None:
            logger.close()


def _run_mc02_policy_mode(
    *,
    runtime: HostRuntime,
    port: str,
    baudrate: int,
    motor_id: int,
    log_path: str | None,
    auto_enable: bool,
    clear_fault_on_enable: bool,
    disable_on_exit: bool,
    print_rx: bool,
) -> int:
    logger = JsonlLogger(Path(log_path)) if log_path else None
    if logger is not None:
        logger.open()

    with Mc02SerialCdcLink(port=port, baudrate=baudrate) as link:
        sensor_seq = 0
        tx_seq = 0

        def next_tx_seq() -> int:
            nonlocal tx_seq
            value = tx_seq
            tx_seq = (tx_seq + 1) & 0xFF
            return value

        try:
            if auto_enable:
                _send_motor_enable(
                    link=link,
                    tx_seq=next_tx_seq(),
                    motor_id=motor_id,
                    enable=True,
                    clear_fault=clear_fault_on_enable,
                )

            while True:
                frames = link.read_frames()
                for frame in frames:
                    if frame.cmd_id == int(UsbPcCommandId.HEARTBEAT_REQ):
                        _reply_heartbeat_ack(link, frame, tx_seq=next_tx_seq())
                        continue

                    sensor_seq += 1
                    if frame.cmd_id == int(UsbPcCommandId.L1_SENSOR_STATE):
                        payload = L1SensorStatePayload.from_bytes(frame.payload)
                        bundle = sensor_frame_from_l1_sensor(payload, seq=sensor_seq)
                    elif frame.cmd_id == int(UsbPcCommandId.SIMPLE_POLE_STATE):
                        payload = SimplePoleStatePayload.from_bytes(frame.payload)
                        bundle = sensor_frame_from_simple_pole_state(
                            payload,
                            seq=sensor_seq,
                            t_us=time.monotonic_ns() // 1000,
                        )
                    else:
                        sensor_seq -= 1
                        if print_rx:
                            print(json.dumps(decode_frame_to_dict(frame), ensure_ascii=False))
                        continue

                    previous_action = runtime.previous_action
                    observation, action, model_output = runtime.step(bundle.sensor_frame)
                    command = position_delta_command_from_action(
                        action,
                        motor_id=motor_id,
                    )
                    tx_frame = A5UsbFrame(
                        seq=next_tx_seq(),
                        cmd_id=int(UsbPcCommandId.POSITION_DELTA_CONTROL),
                        payload=command.to_bytes(),
                    )
                    link.write_frame(tx_frame)

                    if logger is not None:
                        logger.log_cycle(
                            sensor=bundle.sensor_frame,
                            observation=observation,
                            action=action,
                            previous_action=previous_action,
                            observation_history=runtime.history.flatten(),
                            model_output=model_output,
                            extra={
                                "mc02_rx": decode_frame_to_dict(frame),
                                "mc02_tx": decode_frame_to_dict(tx_frame),
                                "mc02_status_flags": bundle.raw_status_flags,
                                "mc02_fault_flags": bundle.raw_fault_flags,
                            },
                        )
        finally:
            if disable_on_exit:
                _send_motor_enable(
                    link=link,
                    tx_seq=next_tx_seq(),
                    motor_id=motor_id,
                    enable=False,
                    clear_fault=False,
                )
            if logger is not None:
                logger.close()

    return 0


def _run_mc02_sine_mode(
    *,
    port: str,
    baudrate: int,
    motor_id: int,
    log_path: str | None,
    amplitude: float,
    frequency_hz: float,
    kp: float,
    kd: float,
    auto_enable: bool,
    clear_fault_on_enable: bool,
    disable_on_exit: bool,
    print_rx: bool,
) -> int:
    logger = JsonlLogger(Path(log_path)) if log_path else None
    if logger is not None:
        logger.open()

    history = ObservationHistory()
    previous_action: ActionFrameV1 | None = None

    with Mc02SerialCdcLink(port=port, baudrate=baudrate) as link:
        sensor_seq = 0
        tx_seq = 0

        def next_tx_seq() -> int:
            nonlocal tx_seq
            value = tx_seq
            tx_seq = (tx_seq + 1) & 0xFF
            return value

        try:
            if auto_enable:
                _send_motor_enable(
                    link=link,
                    tx_seq=next_tx_seq(),
                    motor_id=motor_id,
                    enable=True,
                    clear_fault=clear_fault_on_enable,
                )

            while True:
                frames = link.read_frames()
                for frame in frames:
                    if frame.cmd_id == int(UsbPcCommandId.HEARTBEAT_REQ):
                        _reply_heartbeat_ack(link, frame, tx_seq=next_tx_seq())
                        continue

                    sensor_seq += 1
                    if frame.cmd_id == int(UsbPcCommandId.L1_SENSOR_STATE):
                        payload = L1SensorStatePayload.from_bytes(frame.payload)
                        bundle = sensor_frame_from_l1_sensor(payload, seq=sensor_seq)
                        time_s = float(payload.tick_ms) * 0.001
                    elif frame.cmd_id == int(UsbPcCommandId.SIMPLE_POLE_STATE):
                        payload = SimplePoleStatePayload.from_bytes(frame.payload)
                        bundle = sensor_frame_from_simple_pole_state(
                            payload,
                            seq=sensor_seq,
                            t_us=time.monotonic_ns() // 1000,
                        )
                        time_s = time.monotonic()
                    else:
                        sensor_seq -= 1
                        if print_rx:
                            print(json.dumps(decode_frame_to_dict(frame), ensure_ascii=False))
                        continue

                    delta = float(amplitude) * math.sin(2.0 * math.pi * float(frequency_hz) * time_s)
                    action = ActionFrameV1(
                        seq=bundle.sensor_frame.seq,
                        t_us=bundle.sensor_frame.t_us,
                        base_pos_delta_rad=delta,
                        kp=float(kp),
                        kd=float(kd),
                        watchdog_ms=0,
                        mode_flags=0,
                    )
                    command = position_delta_command_from_action(
                        action,
                        motor_id=motor_id,
                    )
                    tx_frame = A5UsbFrame(
                        seq=next_tx_seq(),
                        cmd_id=int(UsbPcCommandId.POSITION_DELTA_CONTROL),
                        payload=command.to_bytes(),
                    )
                    link.write_frame(tx_frame)

                    features = build_history_frame(bundle.sensor_frame, previous_action)
                    history.push(features)
                    if logger is not None:
                        logger.log_cycle(
                            sensor=bundle.sensor_frame,
                            observation=history.flatten(),
                            action=action,
                            previous_action=previous_action,
                            observation_history=history.flatten(),
                            model_output=[delta],
                            extra={
                                "mc02_rx": decode_frame_to_dict(frame),
                                "mc02_tx": decode_frame_to_dict(tx_frame),
                                "mc02_status_flags": bundle.raw_status_flags,
                                "mc02_fault_flags": bundle.raw_fault_flags,
                            },
                        )
                    previous_action = action
        finally:
            if disable_on_exit:
                _send_motor_enable(
                    link=link,
                    tx_seq=next_tx_seq(),
                    motor_id=motor_id,
                    enable=False,
                    clear_fault=False,
                )
            if logger is not None:
                logger.close()

    return 0


def _run_mc02_monitor_mode(
    *,
    port: str,
    baudrate: int,
    max_frames: int,
    filter_cmd_id: int | None,
) -> int:
    frame_count = 0
    with Mc02SerialCdcLink(port=port, baudrate=baudrate) as link:
        while True:
            frames = link.read_frames()
            for frame in frames:
                if filter_cmd_id is not None and int(frame.cmd_id) != int(filter_cmd_id):
                    continue

                print(json.dumps(decode_frame_to_dict(frame), ensure_ascii=False))
                frame_count += 1
                if max_frames > 0 and frame_count >= max_frames:
                    return 0


def _run_enable_mode(
    *,
    port: str,
    baudrate: int,
    motor_id: int,
    enable: bool,
    clear_fault: bool,
) -> int:
    with _create_serial_link(port, baudrate) as link:
        if hasattr(link, "enable_motor"):
            link.enable_motor(enable=enable, clear_fault=clear_fault)
            return 0

        _send_motor_enable(
            link=link,
            tx_seq=0,
            motor_id=motor_id,
            enable=enable,
            clear_fault=clear_fault,
        )
    return 0


def _run_mit_mode(
    *,
    port: str,
    baudrate: int,
    motor_id: int,
    kp: float,
    kd: float,
    pos_rad: float,
    vel_rad_s: float,
    torque_nm: float,
) -> int:
    with _create_serial_link(port, baudrate) as link:
        if hasattr(link, "send_mit_command"):
            link.send_mit_command(
                kp=kp,
                kd=kd,
                pos_rad=pos_rad,
                vel_rad_s=vel_rad_s,
                torque_nm=torque_nm,
            )
            return 0

        frame = A5UsbFrame(
            seq=0,
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
    return 0


def _send_motor_enable(
    *,
    link: Mc02SerialCdcLink,
    tx_seq: int,
    motor_id: int,
    enable: bool,
    clear_fault: bool,
) -> None:
    payload = MotorEnableCommandPayload(
        motor_id=int(motor_id),
        enable_flag=1 if enable else 0,
        clear_fault_flag=1 if clear_fault else 0,
    )
    frame = A5UsbFrame(
        seq=tx_seq,
        cmd_id=int(UsbPcCommandId.MOTOR_ENABLE),
        payload=payload.to_bytes(),
    )
    link.write_frame(frame)


def _reply_heartbeat_ack(
    link: Mc02SerialCdcLink,
    request_frame: A5UsbFrame,
    *,
    tx_seq: int,
) -> None:
    """如果板端主动发心跳请求，则 PC 侧回一个最小 ACK。"""

    request = HeartbeatPayload.from_bytes(request_frame.payload)
    payload = HeartbeatPayload(
        tick_ms=request.tick_ms,
        protocol_version=request.protocol_version,
        status_flags=request.status_flags | USB_PC_STATUS_USB_LINK_OK,
    )
    ack_frame = A5UsbFrame(
        seq=tx_seq,
        cmd_id=int(UsbPcCommandId.HEARTBEAT_ACK),
        payload=payload.to_bytes(),
    )
    link.write_frame(ack_frame)
