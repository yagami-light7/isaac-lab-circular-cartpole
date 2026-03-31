/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Damiao_Motor.cpp
  * @brief      达妙 DM4310 电机驱动层源文件
  * @note       Hardware Driver Layer
  *             1. 完成 MIT / 位置 / 速度模式控制帧打包
  *             2. 完成反馈帧解析与测量值更新
  *             3. 不引入业务线程，不绑定具体控制任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. refactor for MC02 motor side
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HDL_Damiao_Motor.h"
#include "HDL_Damiao_Motor_App.h"

#include <string.h>

static_assert(sizeof(HDL_Damiao_Motor_App_Measure_t) == sizeof(HDL_Damiao_Motor_Measure_t),
              "Application measure layout must match HDL measure layout");

/**
 * @brief 默认 FDCAN1 达妙总线对象
 */
HDL_Damiao_Motor_Bus_Object_t HDL_Damiao_Motor_Bus_FDCAN1 = {};

/**
 * @brief 默认 FDCAN1 达妙电机反馈数组
 */
HDL_Damiao_Motor_Measure_t HDL_Damiao_Motor_Measure_FDCAN1[DM_MIT_MOTOR_NUM_MAX] = {};

/**
 * @brief 默认 FDCAN1 达妙电机 ID 表
 */
static const uint8_t HDL_Damiao_Motor_Id_Table_FDCAN1[DM_MIT_MOTOR_NUM_MAX] =
{
    0x01U,
    0x02U,
    0x03U,
};

/**
  * @brief          约束浮点数范围
  * @param[in]      value     输入值
  *                 min_value 最小值
  *                 max_value 最大值
  * @retval         约束后的值
  */
static float HDL_Damiao_Motor_Constrain(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
  * @brief          浮点数转无符号整数
  * @param[in]      value     输入值
  *                 min_value 最小值
  *                 max_value 最大值
  *                 bits      目标位宽
  * @retval         转换结果
  */
static uint32_t HDL_Damiao_Motor_Float_To_Uint(float value, float min_value, float max_value, uint32_t bits)
{
    const float span = max_value - min_value;
    const float offset = min_value;
    const float constrained_value = HDL_Damiao_Motor_Constrain(value, min_value, max_value);

    return (uint32_t)((constrained_value - offset) * ((float)((1UL << bits) - 1UL)) / span);
}

/**
  * @brief          无符号整数转浮点数
  * @param[in]      value     输入值
  *                 min_value 最小值
  *                 max_value 最大值
  *                 bits      位宽
  * @retval         转换结果
  */
static float HDL_Damiao_Motor_Uint_To_Float(uint32_t value, float min_value, float max_value, uint32_t bits)
{
    const float span = max_value - min_value;
    const float offset = min_value;

    return ((float)value) * span / ((float)((1UL << bits) - 1UL)) + offset;
}

/**
  * @brief          根据 motor_id 查找数组索引
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         >=0 有效索引
  *                 -1  未找到
  */
static int32_t HDL_Damiao_Motor_Find_Index(const HDL_Damiao_Motor_Bus_Object_t *motor_bus_object, uint8_t motor_id)
{
    uint8_t i = 0U;

    if (motor_bus_object == NULL)
    {
        return -1;
    }

    for (i = 0U; i < motor_bus_object->motor_num; i++)
    {
        if (motor_bus_object->motor_id_map[i] == motor_id)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

/**
  * @brief          发送 8 字节特殊命令
  * @param[in]      motor_bus_object 电机总线对象
  *                 frame_id         帧 ID
  *                 command          特殊命令字节
  * @retval         true 发送成功
  *                 false 发送失败
  */
static bool HDL_Damiao_Motor_Send_Special_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t frame_id,
    uint8_t command)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};

    if (motor_bus_object == NULL)
    {
        return false;
    }

    return HAL_FDCAN_Send_Std_Data(
        motor_bus_object->fdcan_manage_object,
        frame_id,
        data,
        sizeof(data));
}

/**
  * @brief          初始化达妙电机总线对象
  * @param[in]      motor_bus_object    电机总线对象
  *                 fdcan_manage_object FDCAN 管理对象
  *                 motor_measure       电机反馈数组
  *                 motor_num           电机数量
  * @retval         none
  */
void HDL_Damiao_Motor_Init(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    HDL_Damiao_Motor_Measure_t *motor_measure,
    uint8_t motor_num)
{
    uint8_t i = 0U;

    if (motor_bus_object == NULL || fdcan_manage_object == NULL || motor_measure == NULL)
    {
        return;
    }

    memset(motor_bus_object, 0, sizeof(*motor_bus_object));
    motor_bus_object->fdcan_manage_object = fdcan_manage_object;
    motor_bus_object->motor_measure = motor_measure;
    motor_bus_object->motor_num = (motor_num > DM_MIT_MOTOR_NUM_MAX) ? DM_MIT_MOTOR_NUM_MAX : motor_num;

    for (i = 0U; i < DM_MIT_MOTOR_NUM_MAX; i++)
    {
        motor_bus_object->motor_id_map[i] = 0xFFU;
    }

    memset(motor_measure, 0, sizeof(HDL_Damiao_Motor_Measure_t) * motor_bus_object->motor_num);

    HAL_FDCAN_Register_Rx_Callback(
        fdcan_manage_object,
        HDL_Damiao_Motor_Rx_Callback,
        motor_bus_object);
}

/**
  * @brief          初始化默认 FDCAN1 达妙电机对象
  * @param[in]      none
  * @retval         none
  */
bool HDL_Damiao_Motor_FDCAN1_Init(void)
{
    uint8_t i = 0U;

    HAL_FDCAN_Object_Init(&HAL_FDCAN1_Manage_Object, &hfdcan1, FDCAN_RX_FIFO1);
    HDL_Damiao_Motor_Init(
        &HDL_Damiao_Motor_Bus_FDCAN1,
        &HAL_FDCAN1_Manage_Object,
        HDL_Damiao_Motor_Measure_FDCAN1,
        DM_MIT_MOTOR_NUM_MAX);

    if (!HAL_FDCAN_Start_And_Enable_Rx(&HAL_FDCAN1_Manage_Object, 0U, 0U))
    {
        return false;
    }

    for (i = 0U; i < DM_MIT_MOTOR_NUM_MAX; i++)
    {
//        HDL_Damiao_Motor_Enable_Default(HDL_Damiao_Motor_Id_Table_FDCAN1[i]);
        HDL_Damiao_Motor_Register(&HDL_Damiao_Motor_Bus_FDCAN1, i, HDL_Damiao_Motor_Id_Table_FDCAN1[i]);
    }

    return true;
}

/**
  * @brief          注册总线中的电机 ID
  * @param[in]      motor_bus_object 电机总线对象
  *                 index            电机索引
  *                 motor_id         电机 ID
  * @retval         none
  */
void HDL_Damiao_Motor_Register(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint8_t index,
    uint8_t motor_id)
{
    if (motor_bus_object == NULL || index >= motor_bus_object->motor_num)
    {
        return;
    }

    motor_bus_object->motor_id_map[index] = motor_id;
    motor_bus_object->motor_measure[index].id = motor_id;
}

/**
  * @brief          请求电机回传一帧反馈
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Request_Feedback(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id)
{
    return HDL_Damiao_Motor_Send_Special_Command(motor_bus_object, motor_id, DM_MIT_CMD_POLL_FEEDBACK);
}

/**
  * @brief          使能电机
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Enable(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id)
{
    return HDL_Damiao_Motor_Send_Special_Command(motor_bus_object, motor_id, DM_MIT_CMD_ENABLE);
}

/**
  * @brief          失能电机
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Disable(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id)
{
    return HDL_Damiao_Motor_Send_Special_Command(motor_bus_object, motor_id, DM_MIT_CMD_DISABLE);
}

/**
  * @brief          设置当前位置为零点
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  * @retval         true 发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Set_Zero(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id)
{
    return HDL_Damiao_Motor_Send_Special_Command(motor_bus_object, motor_id, DM_MIT_CMD_SET_ZERO);
}

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
bool HDL_Damiao_Motor_Send_MIT_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id,
    float pos,
    float vel,
    float kp,
    float kd,
    float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp = 0U;
    uint16_t vel_tmp = 0U;
    uint16_t kp_tmp = 0U;
    uint16_t kd_tmp = 0U;
    uint16_t tor_tmp = 0U;
    int32_t motor_index = -1;

    if (motor_bus_object == NULL)
    {
        return false;
    }

    pos_tmp = (uint16_t)HDL_Damiao_Motor_Float_To_Uint(pos, DM_MIT_P_MIN, DM_MIT_P_MAX, 16U);
    vel_tmp = (uint16_t)HDL_Damiao_Motor_Float_To_Uint(vel, DM_MIT_V_MIN, DM_MIT_V_MAX, 12U);
    kp_tmp = (uint16_t)HDL_Damiao_Motor_Float_To_Uint(kp, DM_MIT_KP_MIN, DM_MIT_KP_MAX, 12U);
    kd_tmp = (uint16_t)HDL_Damiao_Motor_Float_To_Uint(kd, DM_MIT_KD_MIN, DM_MIT_KD_MAX, 12U);
    tor_tmp = (uint16_t)HDL_Damiao_Motor_Float_To_Uint(torq, DM_MIT_T_MIN, DM_MIT_T_MAX, 12U);

    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp & 0xFFU);
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0x0FU) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)(kp_tmp & 0xFFU);
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0x0FU) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)(tor_tmp & 0xFFU);

    motor_index = HDL_Damiao_Motor_Find_Index(motor_bus_object, (uint8_t)motor_id);
    if (motor_index >= 0)
    {
        motor_bus_object->motor_measure[motor_index].Kp = kp;
        motor_bus_object->motor_measure[motor_index].Kd = kd;
    }

    return HAL_FDCAN_Send_Std_Data(
        motor_bus_object->fdcan_manage_object,
        (uint16_t)(motor_id + DM_MIT_MODE),
        data,
        sizeof(data));
}

/**
  * @brief          发送位置/速度模式控制命令
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  *                 pos              目标位置 [rad]
  *                 vel              目标速度 [rad/s]
  * @retval         true 发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Send_Pos_Vel_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id,
    float pos,
    float vel)
{
    uint8_t data[8];

    if (motor_bus_object == NULL)
    {
        return false;
    }

    memcpy(&data[0], &pos, sizeof(pos));
    memcpy(&data[4], &vel, sizeof(vel));

    return HAL_FDCAN_Send_Std_Data(
        motor_bus_object->fdcan_manage_object,
        (uint16_t)(motor_id + DM_POS_MODE),
        data,
        sizeof(data));
}

/**
  * @brief          发送速度模式控制命令
  * @param[in]      motor_bus_object 电机总线对象
  *                 motor_id         电机 ID
  *                 vel              目标速度 [rad/s]
  * @retval         true 发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Send_Speed_Command(
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint16_t motor_id,
    float vel)
{
    uint8_t data[4];

    if (motor_bus_object == NULL)
    {
        return false;
    }

    memcpy(data, &vel, sizeof(vel));

    return HAL_FDCAN_Send_Std_Data(
        motor_bus_object->fdcan_manage_object,
        (uint16_t)(motor_id + DM_SPEED_MODE),
        data,
        sizeof(data));
}

/**
  * @brief          请求默认总线上的电机回传反馈
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Request_Default_Feedback(uint16_t motor_id)
{
    return HDL_Damiao_Motor_Request_Feedback(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id);
}

/**
  * @brief          使能默认总线上的电机
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Enable_Default(uint16_t motor_id)
{
    return HDL_Damiao_Motor_Enable(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id);
}

/**
  * @brief          失能默认总线上的电机
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Disable_Default(uint16_t motor_id)
{
    return HDL_Damiao_Motor_Disable(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id);
}

/**
  * @brief          设置默认总线上的电机零点
  * @param[in]      motor_id 电机 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Set_Zero_Default(uint16_t motor_id)
{
    return HDL_Damiao_Motor_Set_Zero(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id);
}

/**
  * @brief          向默认总线发送 MIT 模式控制命令
  * @param[in]      motor_id 电机 ID
  *                 pos      目标位置 [rad]
  *                 vel      目标速度 [rad/s]
  *                 kp       刚度系数
  *                 kd       阻尼系数
  *                 torq     前馈力矩 [N*m]
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Send_Default_MIT_Command(
    uint16_t motor_id,
    float pos,
    float vel,
    float kp,
    float kd,
    float torq)
{
    return HDL_Damiao_Motor_Send_MIT_Command(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id, pos, vel, kp, kd, torq);
}

/**
  * @brief          向默认总线发送位置/速度模式命令
  * @param[in]      motor_id 电机 ID
  *                 pos      目标位置 [rad]
  *                 vel      目标速度 [rad/s]
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Send_Default_Pos_Vel_Command(uint16_t motor_id, float pos, float vel)
{
    return HDL_Damiao_Motor_Send_Pos_Vel_Command(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id, pos, vel);
}

/**
  * @brief          向默认总线发送速度模式命令
  * @param[in]      motor_id 电机 ID
  *                 vel      目标速度 [rad/s]
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool HDL_Damiao_Motor_Send_Default_Speed_Command(uint16_t motor_id, float vel)
{
    return HDL_Damiao_Motor_Send_Speed_Command(&HDL_Damiao_Motor_Bus_FDCAN1, motor_id, vel);
}

/**
  * @brief          获取指定电机反馈
  * @param[in]      motor_bus_object 电机总线对象
  *                 index            电机索引
  * @retval         反馈结构体指针
  */
const HDL_Damiao_Motor_Measure_t *HDL_Damiao_Motor_Get_Measure_Point(
    const HDL_Damiao_Motor_Bus_Object_t *motor_bus_object,
    uint8_t index)
{
    if (motor_bus_object == NULL || index >= motor_bus_object->motor_num)
    {
        return NULL;
    }

    return &motor_bus_object->motor_measure[index];
}

/**
  * @brief          获取默认总线上的指定电机反馈
  * @param[in]      index 电机索引
  * @retval         反馈结构体指针
  */
const HDL_Damiao_Motor_App_Measure_t *HDL_Damiao_Motor_Get_Default_Measure_Point(uint8_t index)
{
    return (const HDL_Damiao_Motor_App_Measure_t *)HDL_Damiao_Motor_Get_Measure_Point(&HDL_Damiao_Motor_Bus_FDCAN1, index);
}

/**
  * @brief          达妙电机接收回调
  * @param[in]      fdcan_manage_object FDCAN 管理对象
  *                 rx_header           接收帧头
  *                 rx_data             接收数据
  *                 rx_length           数据长度
  *                 user_arg            总线对象指针
  * @retval         none
  */
void HDL_Damiao_Motor_Rx_Callback(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    const FDCAN_RxHeaderTypeDef *rx_header,
    const uint8_t *rx_data,
    uint32_t rx_length,
    void *user_arg)
{
    HDL_Damiao_Motor_Bus_Object_t *motor_bus_object = (HDL_Damiao_Motor_Bus_Object_t *)user_arg;
    HDL_Damiao_Motor_Measure_t *motor_measure = NULL;
    uint8_t motor_id = 0U;
    int32_t motor_index = -1;

    if (motor_bus_object == NULL || rx_data == NULL || rx_length < 8U)
    {
        return;
    }

    motor_id = (uint8_t)(rx_data[0] & 0x0FU);
    motor_index = HDL_Damiao_Motor_Find_Index(motor_bus_object, motor_id);
    if (motor_index < 0)
    {
        return;
    }

    motor_measure = &motor_bus_object->motor_measure[motor_index];
    motor_measure->id = motor_id;
    motor_measure->err_state = (uint8_t)(rx_data[0] >> 4);
    motor_measure->p_int = (int32_t)(((uint16_t)rx_data[1] << 8) | rx_data[2]);
    motor_measure->v_int = (int32_t)(((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4));
    motor_measure->t_int = (int32_t)((((uint16_t)rx_data[4] & 0x0FU) << 8) | rx_data[5]);
    motor_measure->pos = HDL_Damiao_Motor_Uint_To_Float((uint32_t)motor_measure->p_int, DM_MIT_P_MIN, DM_MIT_P_MAX, 16U);
    motor_measure->vel = HDL_Damiao_Motor_Uint_To_Float((uint32_t)motor_measure->v_int, DM_MIT_V_MIN, DM_MIT_V_MAX, 12U);
    motor_measure->tor = HDL_Damiao_Motor_Uint_To_Float((uint32_t)motor_measure->t_int, DM_MIT_T_MIN, DM_MIT_T_MAX, 12U);
    motor_measure->mos_temp = rx_data[6];
    motor_measure->rotor_temp = rx_data[7];
    motor_measure->Temp = (float)rx_data[6];
    motor_measure->update_count++;
    motor_measure->last_rx_tick = HAL_GetTick();
    motor_measure->online = 1U;
}
