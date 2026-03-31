/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Damiao_Motor_App.h
  * @brief      达妙 DM4310 应用侧轻量接口头文件
  * @note       Hardware Driver Layer
  *             1. 该头文件只暴露默认 hfdcan1 总线下的 DM4310 接口
  *             2. 不包含 fdcan.h / HAL_FDCAN.h，避免把底层句柄传播到应用层
  *             3. 主要给 App_Gimbal 等上层模块使用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. add app side lite interface
  *
  @verbatim
  ==============================================================================
  * 这里保留了与 HDL_Damiao_Motor_Measure_t 对齐的测量结构，
  * 这样应用层可以继续直接访问 pos / vel / tor 等字段，
  * 但不会因为 include 链把 fdcan.h 带入旧框架。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 达妙电机应用层测量结构体
 * @note  字段布局与 HDL_Damiao_Motor_Measure_t 保持一致
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
} HDL_Damiao_Motor_App_Measure_t;

/**
  * @brief          初始化默认 hfdcan1 达妙电机对象
  * @param[in]      none
  * @retval         true  初始化成功
  *                 false 初始化失败
  */
extern bool HDL_Damiao_Motor_FDCAN1_Init(void);

/**
  * @brief          请求默认总线上的电机回传一帧反馈
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Request_Default_Feedback(uint16_t motor_id);

/**
  * @brief          使能默认总线上的电机
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Enable_Default(uint16_t motor_id);

/**
  * @brief          失能默认总线上的电机
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Disable_Default(uint16_t motor_id);

/**
  * @brief          将默认总线上的电机当前位置设置为零点
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Set_Zero_Default(uint16_t motor_id);

/**
  * @brief          向默认总线上的电机发送 MIT 模式控制命令
  * @param[in]      motor_id 电机 ID
  *                 pos      目标位置 [rad]
  *                 vel      目标速度 [rad/s]
  *                 kp       刚度系数
  *                 kd       阻尼系数
  *                 torq     前馈力矩 [N*m]
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Send_Default_MIT_Command(
    uint16_t motor_id,
    float pos,
    float vel,
    float kp,
    float kd,
    float torq);

/**
  * @brief          向默认总线上的电机发送位置/速度模式命令
  * @param[in]      motor_id 电机 ID
  *                 pos      目标位置 [rad]
  *                 vel      目标速度 [rad/s]
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Send_Default_Pos_Vel_Command(
    uint16_t motor_id,
    float pos,
    float vel);

/**
  * @brief          向默认总线上的电机发送速度模式命令
  * @param[in]      motor_id 电机 ID
  *                 vel      目标速度 [rad/s]
  * @retval         true  发送成功
  *                 false 发送失败
  */
extern bool HDL_Damiao_Motor_Send_Default_Speed_Command(uint16_t motor_id, float vel);

/**
  * @brief          获取默认总线上的指定电机反馈
  * @param[in]      index 电机索引
  * @retval         反馈结构体指针
  */
extern const HDL_Damiao_Motor_App_Measure_t *HDL_Damiao_Motor_Get_Default_Measure_Point(uint8_t index);

#ifdef __cplusplus
}
#endif
