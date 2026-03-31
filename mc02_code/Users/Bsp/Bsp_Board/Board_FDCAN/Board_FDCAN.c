/**
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  * @file       Board_FDCAN.c
  * @brief      板级 FDCAN 配置与发送接口
  * @note       FDCAN1: 底盘 DJI 电机
  *             FDCAN2: 大关节达妙电机
  *             FDCAN3: 小关节 DJI 电机
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-28-2024     Light           1. done
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  */
#include "Board_FDCAN.h"

void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);

/**
  * @brief          FDCAN 过滤器初始化
  * @param[in]      none
  * @retval         none
  */
void FDCAN_Filter_Init(void)
{
    uint32_t FDCAN_RXActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
                                 FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                 FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL |
                                 FDCAN_IT_RX_FIFO1_WATERMARK | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;

    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();

    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_RXActiveITs, 0);

    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_RXActiveITs, 0);

    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_RXActiveITs, 0);
}

/**
 * @brief           FDCAN 数据发送接口
 * @param[in]       hfdcan     FDCAN 句柄
 * @param[in]       hfdcan_id  标准帧 ID
 * @param[in]       mes_data   发送数据
 * @param[in]       mes_len    发送数据长度
 * @retval          0 成功
 */
uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t hfdcan_id, uint8_t *mes_data, uint32_t mes_len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = hfdcan_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;

    if (mes_len <= 8)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    }
    else if (mes_len == 12)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    }
    else if (mes_len == 16)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_16;
    }
    else if (mes_len == 20)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_20;
    }
    else if (mes_len == 24)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    }
    else if (mes_len == 48)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_48;
    }
    else if (mes_len == 64)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    }

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, mes_data) != HAL_OK)
    {
        Error_Handler();
    }
    return 0;
}

/**
  * @brief          FDCAN1 过滤器配置，接收到 FIFO0
  * @param[in]      none
  * @retval         none
  */
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief          FDCAN2 过滤器配置，接收到 FIFO1
  * @param[in]      none
  * @retval         none
  */
void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief          FDCAN3 过滤器配置，接收到 FIFO1
  * @param[in]      none
  * @retval         none
  */
void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.IdType = FDCAN_STANDARD_ID;
    can_filter_st.FilterIndex = 0;
    can_filter_st.FilterType = FDCAN_FILTER_MASK;
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    can_filter_st.FilterID1 = 0x00000000;
    can_filter_st.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter_st) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}
