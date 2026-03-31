from __future__ import annotations

from dataclasses import dataclass

from .mc02_usb_protocol import (
    L1SensorStatePayload,
    PositionDeltaControlCommandPayload,
    SimplePoleStatePayload,
)
from .protocol import ActionFrameV1, SensorFrameV1


@dataclass(slots=True)
class Mc02ObservationBundle:
    """把板端原始 L1 观测转换成 host_runtime 可直接消费的对象。"""

    sensor_frame: SensorFrameV1
    raw_status_flags: int
    raw_fault_flags: int


def sensor_frame_from_l1_sensor(
    payload: L1SensorStatePayload,
    *,
    seq: int,
) -> Mc02ObservationBundle:
    """把 MC02 的 L1 传感器上报转成现有 HostRuntime 使用的固定观测结构。"""

    # HostRuntime 只真正消费位置、速度和温度。
    # 状态字不参与推理，但为了方便日志回放，这里把 status/fault 合并存进去。
    combined_status = (int(payload.fault_flags) << 16) | int(payload.status_flags)

    sensor_frame = SensorFrameV1(
        seq=int(seq),
        t_us=int(payload.tick_ms) * 1000,
        base_pos_rad=float(payload.base_pos_rad),
        base_vel_rad_s=float(payload.base_vel_rad_s),
        flex1_pos_rad=float(payload.flex_pos_rad),
        flex1_vel_rad_s=float(payload.flex_vel_rad_s),
        motor_temp_c=float(payload.motor_temp_c),
        status_flags=combined_status,
    )
    return Mc02ObservationBundle(
        sensor_frame=sensor_frame,
        raw_status_flags=int(payload.status_flags),
        raw_fault_flags=int(payload.fault_flags),
    )


def sensor_frame_from_simple_pole_state(
    payload: SimplePoleStatePayload,
    *,
    seq: int,
    t_us: int,
) -> Mc02ObservationBundle:
    """把板端临时 5-float 观测帧转成 HostRuntime 使用的固定观测结构。"""

    sensor_frame = SensorFrameV1(
        seq=int(seq),
        t_us=int(t_us),
        base_pos_rad=float(payload.base_pos_rad),
        base_vel_rad_s=float(payload.base_vel_rad_s),
        flex1_pos_rad=float(payload.flex_pos_rad),
        flex1_vel_rad_s=float(payload.flex_vel_rad_s),
        motor_temp_c=0.0,
        status_flags=0,
    )
    return Mc02ObservationBundle(
        sensor_frame=sensor_frame,
        raw_status_flags=0,
        raw_fault_flags=0,
    )


def position_delta_command_from_action(
    action: ActionFrameV1,
    *,
    motor_id: int,
) -> PositionDeltaControlCommandPayload:
    """把策略输出动作映射成板端当前位置增量控制命令。"""

    return PositionDeltaControlCommandPayload(
        motor_id=int(motor_id),
        pos_delta_rad=float(action.base_pos_delta_rad),
    )
