/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Damiao_Motor.h
  * @brief      达妙 DM4310 电机驱动层头文件
  * @note       Hardware Driver Layer
  *             1. 实现达妙电机 MIT / 位置 / 速度三类协议打包
  *             2. 实现达妙电机反馈帧解析
  *             3. 将一条 FDCAN 总线上的多个达妙电机组织成统一对象
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. refactor for MC02 motor side
  *
  @verbatim
  ==============================================================================
  * 该层只关注“达妙电机协议”本身，不负责线程、不负责业务状态机、不负责安全策略。
  * 上层如果需要急停、看门狗、限幅、状态机，请放在 0-APL 或更高层完成。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "HAL_FDCAN.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MIT 电机反馈结构体
 */
typedef struct
{
    uint8_t id;                                                  /*!< 电机 ID */
    uint8_t err_state;                                           /*!< 错误状态码 */
    int32_t p_int;                                               /*!< 位置原始量 */
    int32_t v_int;                                               /*!< 速度原始量 */
    int32_t t_int;                                               /*!< 力矩原始量 */
    int32_t kp_int;                                              /*!< Kp 原始量 */
    int32_t kd_int;                                              /*!< Kd 原始量 */
    float pos;                                                   /*!< 位置反馈 [rad] */
    float vel;                                                   /*!< 速度反馈 [rad/s] */
    float tor;                                                   /*!< 力矩反馈 [N*m] */
    float Kp;                                                    /*!< 最近一次下发的 Kp */
    float Kd;                                                    /*!< 最近一次下发的 Kd */
    float Temp;                                                  /*!< 兼容字段，默认使用 MOS 温度 */
    uint8_t mos_temp;                                            /*!< MOS 温度 */
    uint8_t rotor_temp;                                          /*!< 转子温度 */
    uint32_t update_count;                                       /*!< 累计更新次数 */
    uint32_t last_rx_tick;                                       /*!< 最近一次接收 tick */
    uint8_t online;                                              /*!< 在线标志 */
} HDL_Damiao_Motor_Measure_t;

/**
 * @brief 达妙电机模式偏移量
 */
#define DM_MIT_MODE                        0x000U
#define DM_POS_MODE                        0x100U
#define DM_SPEED_MODE                      0x200U

/**
 * @brief DM4310 MIT 模式量程
 * @note  参考达妙 4310 手册示例代码
 */
#define DM_MIT_P_MIN                       (-12.5f)
#define DM_MIT_P_MAX                       (12.5f)
#define DM_MIT_V_MIN                       (-30.0f)
#define DM_MIT_V_MAX                       (30.0f)
#define DM_MIT_KP_MIN                      (0.0f)
#define DM_MIT_KP_MAX                      (500.0f)
#define DM_MIT_KD_MIN                      (0.0f)
#define DM_MIT_KD_MAX                      (5.0f)
#define DM_MIT_T_MIN                       (-10.0f)
#define DM_MIT_T_MAX                       (10.0f)

/**
 * @brief 达妙电机特殊命令
 * @note  其中 0x01 为示例代码中常见的轮询反馈命令，不在当前手册正文中明确展开说明
 */
#define DM_MIT_CMD_ENABLE                  0xFCU
#define DM_MIT_CMD_DISABLE                 0xFDU
#define DM_MIT_CMD_SET_ZERO                0xFEU
#define DM_MIT_CMD_POLL_FEEDBACK           0x01U

/**
 * @brief 单条总线支持的最大达妙电机数量
 */
#define DM_MIT_MOTOR_NUM_MAX               3U

/**
 * @brief 达妙电机总线对象
 */
typedef struct
{
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object;              /*!< FDCAN 管理对象 */
    HDL_Damiao_Motor_Measure_t *motor_measure;                          /*!< 外部传入的反馈数组 */
    uint8_t motor_id_map[DM_MIT_MOTOR_NUM_MAX];                  /*!< index -> motor_id 映射 */
    uint8_t motor_num;                                           /*!< 总线上的电机数量 */
} HDL_Damiao_Motor_Bus_Object_t;

/**
 * @brief 默认 FDCAN2 达妙总线对象
 */
extern HDL_Damiao_Motor_Bus_Object_t HDL_Damiao_Motor_Bus_FDCAN1;

/**
 * @brief 默认 FDCAN2 达妙电机反馈数组
 */
extern HDL_Damiao_Motor_Measure_t HDL_Damiao_Motor_Measure_FDCAN1[DM_MIT_MOTOR_NUM_MAX];

/**
  * @brief          初始化默认 FDCAN2 达妙电机对象
  * @param[in]      none
  * @retval         none
  */
extern bool HDL_Damiao_Motor_FDCAN1_Init(void);

/**
  * @brief          初始化达妙电机总线对象
  * @param[in]      motor_bus_object    电机总线对象
  *                 fdcan_manage_object FDCAN 管理对象
  *                 motor_measure       电机反馈数组
  *                 motor_num           电机数量
  * @retval         none
  */
extern void HDL_Damiao_Motor_Init(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    HDL_Damiao_Motor_Measure_t *motor_measure,
    uint8_t motor_num);

/**
  * @brief          注册总线中的电机 ID
  * @param[in]      motor_bus_object 电机总线对象
  *                 index            电机索引
  *                 motor_id         电机 ID
  * @retval         none
  */
extern void HDL_Damiao_Motor_Register(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint8_t index,
    uint8_t motor_id);

/**
  * @brief          请求电机回传一帧反馈
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Request_Feedback(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id);

/**
  * @brief          使能电机
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Enable(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id);

/**
  * @brief          失能电机
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Disable(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id);

/**
  * @brief          设置当前位置为零点
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Set_Zero(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id);

/**
  * @brief          发送 MIT 模式控制命令
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  *                 pos              目标位置 [rad]
  *                 vel              目标速度 [rad/s]
  *                 kp               刚度系数
  *                 kd               阻尼系数
  *                 torq             前馈力矩 [N*m]
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Send_MIT_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id,
    float pos,
    float vel,
    float kp,
    float kd,
    float torq);

/**
  * @brief          发送位置/速度模式控制命令
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  *                 pos              目标位置 [rad]
  *                 vel              目标速度 [rad/s]
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Send_Pos_Vel_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id,
    float pos,
    float vel);

/**
  * @brief          发送速度模式控制命令
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  *                 vel              目标速度 [rad/s]
  * @retval         true 发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Send_Speed_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id,
    float vel);

/**
  * @brief          获取指定电机反馈
  * @param[in]      motor_bus_object 电机总线对象
  *                 index            电机索引
  * @retval         反馈结构体指针
  */
extern const HDL_Damiao_Motor_Measure_t *HDL_Damiao_Motor_Get_Measure_Point(
    const HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint8_t index);

/**
  * @brief          达妙电机接收回调
  * @param[in]      fdcan_manage_object FDCAN 管理对象
  *                 rx_header           接收帧头
  *                 rx_data             接收数据
  *                 rx_length           数据长度
  *                 user_arg            总线对象指针
  * @retval         none
  */
extern void HDL_Damiao_Motor_Rx_Callback(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    const FDCAN_RxHeaderTypeDef *rx_header,
    const uint8_t *rx_data,
    uint32_t rx_length,
    void *user_arg);

#ifdef __cplusplus
}
#endif
