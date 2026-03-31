/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_FDCAN.cpp
  * @brief      FDCAN 应用层中断回调实现
  * @note       Application Layer
  *             1. 该文件只保留 HAL 提供的弱回调重载
  *             2. 电机反馈统一由 hfdcan1 + FIFO1 进入新框架
  *             3. 不再兼容旧框架的 Dev_FDCAN 或多总线分发逻辑
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. refactor for MC02 motor side
  *
  @verbatim
  ==============================================================================
  * 这里刻意保持和 APL_USART 一样的职责边界:
  * APL 只接管 HAL 的弱回调入口，不额外封装对象，也不承载协议解析。
  * 实际的 FIFO 清空与帧分发交给 3-HAL/HAL_FDCAN，电机协议解析交给 2-HDL。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "APL_FDCAN.h"

#include "HAL_FDCAN.h"

/**
  * @brief          FDCAN FIFO0 接收回调
  * @param[in]      hfdcan     FDCAN 句柄
  *                 RxFifo0ITs FIFO0 中断标志
  * @retval         none
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    const uint32_t fdcan_fifo0_active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                            FDCAN_IT_RX_FIFO0_FULL |
                                            FDCAN_IT_RX_FIFO0_WATERMARK;

    if (hfdcan != HAL_FDCAN1_Manage_Object.hfdcan)
    {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        HAL_FDCAN1_Manage_Object.rx_lost_count++;
    }

    if ((RxFifo0ITs & fdcan_fifo0_active_its) == 0U)
    {
        return;
    }

    HAL_FDCAN_Process_RxFifo(&HAL_FDCAN1_Manage_Object);
}

/**
  * @brief          FDCAN FIFO1 接收回调
  * @param[in]      hfdcan     FDCAN 句柄
  *                 RxFifo1ITs FIFO1 中断标志
  * @retval         none
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    const uint32_t fdcan_fifo1_active_its = FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                            FDCAN_IT_RX_FIFO1_FULL |
                                            FDCAN_IT_RX_FIFO1_WATERMARK;

    if (hfdcan != HAL_FDCAN1_Manage_Object.hfdcan)
    {
        return;
    }

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U)
    {
        HAL_FDCAN1_Manage_Object.rx_lost_count++;
    }

    if ((RxFifo1ITs & fdcan_fifo1_active_its) == 0U)
    {
        return;
    }

    HAL_FDCAN_Process_RxFifo(&HAL_FDCAN1_Manage_Object);
}
