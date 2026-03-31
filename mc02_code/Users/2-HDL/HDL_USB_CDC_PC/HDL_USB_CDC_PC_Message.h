/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_USB_CDC_PC_Message.h
  * @brief      USB CDC PC消息模板定义
  * @note       统一管理PC <-> MC02通信的命令字与载荷格式
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. 新增电机/编码器/板级状态消息模板
  *
  @verbatim
  ==============================================================================
  * 本文件只描述各类消息载荷模板。
  * 实际传输帧格式为：
  * A5 + data_length(2B) + seq(1B) + CRC8(1B) + cmd_id(2B) + payload + CRC16(2B)
  *
  * 推荐使用方式：
  * 1. PC -> MC02 使用下方命令载荷结构体发送控制指令
  * 2. MC02 -> PC 使用统一状态结构体回传电机、编码器和板级状态
  * 3. L1实机部署可直接复用USB_PC_L1_Sensor_State_t
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdint.h>

#define USB_PC_PROTOCOL_VERSION                 0x0001U

#define USB_PC_CMD_ID_HEARTBEAT_REQ             0x0101U
#define USB_PC_CMD_ID_HEARTBEAT_ACK             0x0102U
#define USB_PC_CMD_ID_BOARD_STATUS              0x0110U

#define USB_PC_CMD_ID_DM4310_STATE              0x0201U
#define USB_PC_CMD_ID_MT6701_STATE              0x0202U
#define USB_PC_CMD_ID_L1_SENSOR_STATE           0x0203U

#define USB_PC_CMD_ID_MOTOR_ENABLE              0x0301U
#define USB_PC_CMD_ID_CUSTOM_CONTROL            0x0302U
#define USB_PC_CMD_ID_REMOTE_CONTROL            0x0304U
#define USB_PC_CMD_ID_MIT_CONTROL               0x0310U
#define USB_PC_CMD_ID_POSITION_DELTA_CONTROL    0x0311U
#define USB_PC_CMD_ID_EMERGENCY_STOP            0x03FFU

#define USB_PC_STATUS_MOTOR_ONLINE              0x0001U
#define USB_PC_STATUS_ENCODER_ONLINE            0x0002U
#define USB_PC_STATUS_USB_LINK_OK               0x0004U
#define USB_PC_STATUS_MOTOR_ENABLED             0x0008U
#define USB_PC_STATUS_MIT_MODE_ACTIVE           0x0010U

#define USB_PC_FAULT_MOTOR_OVERTEMP             0x0001U
#define USB_PC_FAULT_MOTOR_OVERSPEED            0x0002U
#define USB_PC_FAULT_ENCODER_INVALID            0x0004U
#define USB_PC_FAULT_USB_TIMEOUT                0x0008U
#define USB_PC_FAULT_FDCAN_OFFLINE              0x0010U

#pragma pack(push, 1)

/**
 * @brief 心跳请求/应答载荷
 */
typedef struct
{
    uint32_t tick_ms;
    uint16_t protocol_version;
    uint16_t status_flags;
} USB_PC_Heartbeat_t;

/**
 * @brief 板级状态上报载荷
 */
typedef struct
{
    uint32_t tick_ms;
    uint16_t status_flags;
    uint16_t fault_flags;
    float usb_rx_rate_hz;
    float usb_tx_rate_hz;
} USB_PC_Board_Status_t;

/**
 * @brief DM4310状态上报载荷
 */
typedef struct
{
    uint32_t tick_ms;
    uint8_t motor_id;
    uint8_t mode;
    uint16_t fault_flags;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} USB_PC_DM4310_State_t;

/**
 * @brief MT6701状态上报载荷
 */
typedef struct
{
    uint32_t tick_ms;
    uint8_t encoder_id;
    uint8_t valid_flag;
    uint16_t fault_flags;
    float single_turn_rad;
    float multi_turn_rad;
    float velocity_rad_s;
} USB_PC_MT6701_State_t;

/**
 * @brief 推荐用于L1部署的传感器状态上报载荷
 */
typedef struct
{
    uint32_t tick_ms;
    uint16_t status_flags;
    uint16_t fault_flags;
    float base_pos_rad;
    float base_vel_rad_s;
    float flex_pos_rad;
    float flex_vel_rad_s;
    float motor_temp_c;
} USB_PC_L1_Sensor_State_t;

/**
 * @brief 电机使能/失能命令载荷
 */
typedef struct
{
    uint8_t motor_id;
    uint8_t enable_flag;
    uint8_t clear_fault_flag;
    uint8_t reserved;
} USB_PC_Motor_Enable_Command_t;

/**
 * @brief MIT模式控制命令载荷
 */
typedef struct
{
    uint8_t motor_id;
    uint8_t reserved0;
    uint16_t reserved1;
    float kp;
    float kd;
    float pos_rad;
    float vel_rad_s;
    float torque_nm;
} USB_PC_MIT_Command_t;

/**
 * @brief 面向sim2real部署的位置增量控制命令载荷
 */
typedef struct
{
    uint8_t motor_id;
    float pos_delta_rad;
} USB_PC_Position_Delta_Command_t;

/**
 * @brief 急停命令载荷
 */
typedef struct
{
    uint32_t stop_code;
} USB_PC_Emergency_Stop_Command_t;

#pragma pack(pop)
