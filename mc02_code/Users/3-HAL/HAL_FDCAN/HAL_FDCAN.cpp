/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_FDCAN.cpp
  * @brief      FDCAN 硬件抽象层源文件
  * @note       Hardware Abstract Layer
  *             1. 负责 FDCAN 管理对象初始化
  *             2. 负责统一的标准帧发送封装
  *             3. 负责中断中 FIFO 帧批量读取，并转发给上层回调
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. refactor for MC02 motor side
  *
  @verbatim
  ==============================================================================
  * 该文件不解析任何业务协议，只负责把底层 CAN 帧完整送到上层。
  * 这样做的目的是避免 Dev_FDCAN 继续同时承担“外设层 + 协议层 + 业务层”三种职责。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HAL_FDCAN.h"

#include <string.h>

static uint32_t HAL_FDCAN_Length_To_Dlc(uint32_t tx_length);

/**
 * @brief FDCAN 默认管理对象
 */
HAL_FDCAN_Manage_Object_t HAL_FDCAN1_Manage_Object = {&hfdcan1, FDCAN_RX_FIFO1, NULL, NULL, 0U, 0U, 0U};

/**
  * @brief          初始化 FDCAN 管理对象
  * @param[in]      fdcan_manage_object  FDCAN 管理对象
  *                 hfdcan               底层 FDCAN 句柄
  *                 rx_fifo              默认接收 FIFO
  * @retval         none
  */
void HAL_FDCAN_Object_Init(HAL_FDCAN_Manage_Object_t *fdcan_manage_object, FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo)
{
    if (fdcan_manage_object == NULL)
    {
        return;
    }

    memset(fdcan_manage_object, 0, sizeof(*fdcan_manage_object));
    fdcan_manage_object->hfdcan = hfdcan;
    fdcan_manage_object->rx_fifo = rx_fifo;
}

bool HAL_FDCAN_Start_And_Enable_Rx(
    HAL_FDCAN_Manage_Object_t *fdcan_manage_object,
    uint32_t filter_index,
    uint32_t rx_buffer_index)
{
    FDCAN_FilterTypeDef filter_config;
    uint32_t rx_active_its;

    if (fdcan_manage_object == NULL || fdcan_manage_object->hfdcan == NULL)
    {
        return false;
    }

    memset(&filter_config, 0, sizeof(filter_config));
    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = filter_index;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = (fdcan_manage_object->rx_fifo == FDCAN_RX_FIFO0) ?
                                 FDCAN_FILTER_TO_RXFIFO0 :
                                 FDCAN_FILTER_TO_RXFIFO1;
    filter_config.FilterID1 = 0x000U;
    filter_config.FilterID2 = 0x000U;

    if (HAL_FDCAN_ConfigFilter(fdcan_manage_object->hfdcan, &filter_config) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            fdcan_manage_object->hfdcan,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigRxFifoOverwrite(fdcan_manage_object->hfdcan, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigRxFifoOverwrite(fdcan_manage_object->hfdcan, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_Start(fdcan_manage_object->hfdcan) != HAL_OK)
    {
        return false;
    }

    rx_active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                    FDCAN_IT_RX_FIFO0_FULL |
                    FDCAN_IT_RX_FIFO0_WATERMARK |
                    FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                    FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                    FDCAN_IT_RX_FIFO1_FULL |
                    FDCAN_IT_RX_FIFO1_WATERMARK |
                    FDCAN_IT_RX_FIFO1_MESSAGE_LOST;

    if (HAL_FDCAN_ActivateNotification(fdcan_manage_object->hfdcan, rx_active_its, rx_buffer_index) != HAL_OK)
    {
        return false;
    }

    return true;
}

/**
  * @brief          为 FDCAN 管理对象注册接收回调
  * @param[in]      fdcan_manage_object  FDCAN 管理对象
  *                 rx_callback          接收回调函数
  *                 user_arg             回调私有参数
  * @retval         none
  */
void HAL_FDCAN_Register_Rx_Callback(HAL_FDCAN_Manage_Object_t *fdcan_manage_object, HAL_FDCAN_Rx_Callback_t rx_callback, void *user_arg)
{
    if (fdcan_manage_object == NULL)
    {
        return;
    }

    fdcan_manage_object->rx_callback = rx_callback;
    fdcan_manage_object->user_arg = user_arg;
}

/**
  * @brief          发送标准帧数据
  * @param[in]      fdcan_manage_object  FDCAN 管理对象
  *                 std_id               标准帧 ID
  *                 tx_data              发送数据
  *                 tx_length            发送字节数
  * @retval         true  发送请求成功写入底层队列
  *                 false 参数非法或底层发送失败
  */
bool HAL_FDCAN_Send_Std_Data(HAL_FDCAN_Manage_Object_t *fdcan_manage_object, uint16_t std_id, const uint8_t *tx_data, uint32_t tx_length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    if (fdcan_manage_object == NULL || fdcan_manage_object->hfdcan == NULL || tx_data == NULL)
    {
        return false;
    }

    tx_header.Identifier = std_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = HAL_FDCAN_Length_To_Dlc(tx_length);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (tx_header.DataLength == 0xFFFFFFFFU)
    {
        return false;
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(fdcan_manage_object->hfdcan, &tx_header, (uint8_t *)tx_data) == HAL_OK);
}

/**
  * @brief          将 HAL DLC 常量转换为实际字节数
  * @param[in]      dlc HAL FDCAN DataLength 宏
  * @retval         实际字节数
  */
uint32_t HAL_FDCAN_Dlc_To_Bytes(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0:  return 0U;
        case FDCAN_DLC_BYTES_1:  return 1U;
        case FDCAN_DLC_BYTES_2:  return 2U;
        case FDCAN_DLC_BYTES_3:  return 3U;
        case FDCAN_DLC_BYTES_4:  return 4U;
        case FDCAN_DLC_BYTES_5:  return 5U;
        case FDCAN_DLC_BYTES_6:  return 6U;
        case FDCAN_DLC_BYTES_7:  return 7U;
        case FDCAN_DLC_BYTES_8:  return 8U;
        case FDCAN_DLC_BYTES_12: return 12U;
        case FDCAN_DLC_BYTES_16: return 16U;
        case FDCAN_DLC_BYTES_20: return 20U;
        case FDCAN_DLC_BYTES_24: return 24U;
        case FDCAN_DLC_BYTES_32: return 32U;
        case FDCAN_DLC_BYTES_48: return 48U;
        case FDCAN_DLC_BYTES_64: return 64U;
        default:                 return 0U;
    }
}

/**
  * @brief          从指定 FDCAN FIFO 中持续搬运所有接收帧
  * @param[in]      fdcan_manage_object FDCAN 管理对象
  * @retval         none
  */
void HAL_FDCAN_Process_RxFifo(HAL_FDCAN_Manage_Object_t *fdcan_manage_object)
{
    if (fdcan_manage_object == NULL || fdcan_manage_object->hfdcan == NULL)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(fdcan_manage_object->hfdcan, fdcan_manage_object->rx_fifo) > 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[64] = {0};

        // 搬运接收帧，若失败则记录错误并跳出循环（避免死循环）
        if (HAL_FDCAN_GetRxMessage(fdcan_manage_object->hfdcan,fdcan_manage_object->rx_fifo,&rx_header,rx_data) != HAL_OK)
        {
            fdcan_manage_object->rx_error_count++;
            break;
        }

        // 成功搬运接收帧
        fdcan_manage_object->rx_frame_count++;

        // 调用上层回调函数
        if (fdcan_manage_object->rx_callback != NULL)
        {
            fdcan_manage_object->rx_callback(fdcan_manage_object, &rx_header, rx_data,HAL_FDCAN_Dlc_To_Bytes(rx_header.DataLength), fdcan_manage_object->user_arg);
        }
    }
}

/**
  * @brief          将发送字节数转换为 DLC 宏
  * @param[in]      tx_length 发送字节数
  * @retval         DLC 宏，若长度非法则返回 0xFFFFFFFFU
  */
static uint32_t HAL_FDCAN_Length_To_Dlc(uint32_t tx_length)
{
    switch (tx_length)
    {
        case 0U:  return FDCAN_DLC_BYTES_0;
        case 1U:  return FDCAN_DLC_BYTES_1;
        case 2U:  return FDCAN_DLC_BYTES_2;
        case 3U:  return FDCAN_DLC_BYTES_3;
        case 4U:  return FDCAN_DLC_BYTES_4;
        case 5U:  return FDCAN_DLC_BYTES_5;
        case 6U:  return FDCAN_DLC_BYTES_6;
        case 7U:  return FDCAN_DLC_BYTES_7;
        case 8U:  return FDCAN_DLC_BYTES_8;
        case 12U: return FDCAN_DLC_BYTES_12;
        case 16U: return FDCAN_DLC_BYTES_16;
        case 20U: return FDCAN_DLC_BYTES_20;
        case 24U: return FDCAN_DLC_BYTES_24;
        case 32U: return FDCAN_DLC_BYTES_32;
        case 48U: return FDCAN_DLC_BYTES_48;
        case 64U: return FDCAN_DLC_BYTES_64;
        default:  return 0xFFFFFFFFU;
    }
}
