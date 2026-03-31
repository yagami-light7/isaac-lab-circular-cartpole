/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_FDCAN.h
  * @brief      FDCAN 硬件抽象层头文件
  * @note       Hardware Abstract Layer
  *             1. 统一管理 FDCAN1/FDCAN2/FDCAN3 的收发对象
  *             2. 将底层 FIFO 读取流程从业务层中剥离
  *             3. 为 HDL_Damiao_Motor 等上层模块提供稳定、可复用的接收回调入口
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. refactor for MC02 motor side
  *
  @verbatim
  ==============================================================================
  * 本层只处理“FDCAN 外设对象”和“FIFO 帧搬运”。
  * 业务协议解析不应继续放在本层中，而应放在 2-HDL / 0-APL 中完成。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FDCAN 管理对象前向声明
 */
typedef struct HAL_FDCAN_Manage_Object HAL_FDCAN_Manage_Object_t;

/**
 * @brief FDCAN 接收回调函数类型
 * @param[in] fdcan_manage_object FDCAN 管理对象
 *            rx_header           接收帧头
 *            rx_data             接收数据缓存
 *            rx_length           有效字节数
 *            user_arg            用户私有参数
 * @retval none
 */
typedef void (*HAL_FDCAN_Rx_Callback_t)(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    const FDCAN_RxHeaderTypeDef *rx_header,
    const uint8_t *rx_data,
    uint32_t rx_length,
    void *user_arg);

/**
 * @brief FDCAN 管理对象
 */
struct HAL_FDCAN_Manage_Object
{
    FDCAN_HandleTypeDef *hfdcan;                  /*!< FDCAN 外设句柄 */
    uint32_t rx_fifo;                            /*!< 当前对象默认读取的 RX FIFO */
    HAL_FDCAN_Rx_Callback_t rx_callback;         /*!< 上层注册的接收回调 */
    void *user_arg;                              /*!< 回调私有参数 */
    uint32_t rx_frame_count;                     /*!< 累计成功接收帧数 */
    uint32_t rx_error_count;                     /*!< 累计接收错误次数 */
    uint32_t rx_lost_count;                      /*!< 累计 FIFO 丢帧次数 */
};

/**
 * @brief FDCAN 默认管理对象
 */
extern HAL_FDCAN_Manage_Object_t HAL_FDCAN1_Manage_Object;

/**
  * @brief          初始化 FDCAN 管理对象
  * @param[in]      fdcan_manage_object  FDCAN 管理对象
  *                 hfdcan               底层 FDCAN 句柄
  *                 rx_fifo              默认接收 FIFO
  * @retval         none
  */
extern void HAL_FDCAN_Object_Init(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_fifo);

/**
  * @brief          配置 FDCAN 标准滤波器并启动外设
  * @param[in]      fdcan_manage_object FDCAN 管理对象
  *                 filter_index        标准滤波器索引
  *                 rx_buffer_index     通知配置时使用的 buffer index，经典 FIFO 模式下填 0
  * @retval         true  初始化并启动成功
  *                 false 初始化失败
  */
extern bool HAL_FDCAN_Start_And_Enable_Rx(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    uint32_t filter_index,
    uint32_t rx_buffer_index);

/**
  * @brief          为 FDCAN 管理对象注册接收回调
  * @param[in]      fdcan_manage_object  FDCAN 管理对象
  *                 rx_callback          接收回调函数
  *                 user_arg             回调私有参数
  * @retval         none
  */
extern void HAL_FDCAN_Register_Rx_Callback(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    HAL_FDCAN_Rx_Callback_t rx_callback,
    void *user_arg);

/**
  * @brief          发送标准帧数据
  * @param[in]      fdcan_manage_object  FDCAN 管理对象
  *                 std_id               标准帧 ID
  *                 tx_data              发送数据
  *                 tx_length            发送字节数
  * @retval         true  发送请求成功写入底层队列
  *                 false 参数非法或底层发送失败
  */
extern bool HAL_FDCAN_Send_Std_Data(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    uint16_t std_id,
    const uint8_t *tx_data,
    uint32_t tx_length);

/**
  * @brief          将 HAL DLC 常量转换为实际字节数
  * @param[in]      dlc HAL FDCAN DataLength 宏
  * @retval         实际字节数
  */
extern uint32_t HAL_FDCAN_Dlc_To_Bytes(uint32_t dlc);

/**
  * @brief          从指定 FDCAN FIFO 中持续搬运所有接收帧
  * @param[in]      fdcan_manage_object FDCAN 管理对象
  * @retval         none
  */
extern void HAL_FDCAN_Process_RxFifo(HAL_FDCAN_Manage_Object_t *fdcan_manage_object);

#ifdef __cplusplus
}
#endif
