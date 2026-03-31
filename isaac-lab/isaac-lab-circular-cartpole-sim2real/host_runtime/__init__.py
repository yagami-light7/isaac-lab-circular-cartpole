"""Host-side runtime utilities for circular cartpole sim2real."""

from .history import ObservationHistory, build_history_frame, default_observation_features
from .mc02_bridge import (
    Mc02ObservationBundle,
    position_delta_command_from_action,
    sensor_frame_from_l1_sensor,
    sensor_frame_from_simple_pole_state,
)
from .mc02_usb_protocol import (
    A5UsbFrame,
    A5UsbFrameStreamParser,
    BoardStatusPayload,
    Dm4310StatePayload,
    EmergencyStopCommandPayload,
    HeartbeatPayload,
    L1SensorStatePayload,
    Mc02UsbFrameError,
    MitControlCommandPayload,
    MotorEnableCommandPayload,
    Mt6701StatePayload,
    PositionDeltaControlCommandPayload,
    SimplePoleStatePayload,
    UsbPcCommandId,
    append_crc16_check_sum,
    append_crc8_check_sum,
    decode_frame_to_dict,
    decode_known_payload,
    get_crc16_check_sum,
    get_crc8_check_sum,
    verify_crc16_check_sum,
    verify_crc8_check_sum,
)
from .mc02_usb_transport import Mc02SerialCdcLink, find_stm_virtual_com_port
from .policy import (
    CallablePolicyBackend,
    OnnxPolicyBackend,
    PolicyBackend,
    TorchScriptPolicyBackend,
    load_policy,
)
from .protocol import (
    ActionFrameV1,
    FrameCodecError,
    FrameStreamParser,
    SensorFrameV1,
    crc16_ccitt_false,
)
from .replay import ReplayReport, replay_log
from .runtime import HostRuntime, HostRuntimeConfig
from .session_log import JsonlLogger
