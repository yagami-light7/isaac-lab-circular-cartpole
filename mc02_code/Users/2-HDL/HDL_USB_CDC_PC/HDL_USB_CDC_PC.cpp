/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_USB_CDC_PC.cpp
  * @brief      PC端USB CDC协议类
  * @note       Hardware Driver Layer硬件驱动层
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     Mar-29-2026     Codex           1. 重构为驱动实例自持有与原始接收队列
  *
  @verbatim
  ==============================================================================
  * 本模块不参与 A5 协议拆包，只负责：
  * 1. 在线程中完成 USB CDC 驱动初始化
  * 2. 创建原始接收队列
  * 3. 将 APL 中断入口转交的原始包压入队列
  * 4. 在需要时将上层状态数据封装成 A5 帧并发送给 PC
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HDL_USB_CDC_PC.h"

#include "Board_CRC8_CRC16.h"

/**
 * @brief          初始化PC USB通信模块并创建原始接收队列
 * @param[in]      none
 * @retval         none
 */
void Class_PC_USB_Communication::Init(void)
{
    memset(&this->usb_cdc_manage_object, 0, sizeof(this->usb_cdc_manage_object));
    this->tx_seq = 0U;

    this->pc_comm.xraw_queue = xQueueCreate(USB_PC_RX_RAW_QUEUE_ITEM_NUM, sizeof(USB_PC_Raw_Rx_Packet_t));
    if (this->pc_comm.xraw_queue == NULL)
    {
        Error_Handler();
    }

    USB_CDC_Init(&this->usb_cdc_manage_object);
}

/**
 * @brief          在中断上下文中压入原始USB CDC接收数据包
 * @param[in]      data                    原始接收数据首地址
 *                 length                  原始接收数据长度
 *                 xHigherPriorityTaskWoken 任务切换标志
 * @retval         none
 */
void Class_PC_USB_Communication::Push_Raw_Data_From_ISR(uint8_t *data,
                                                        uint16_t length,
                                                        BaseType_t *xHigherPriorityTaskWoken)
{
    USB_PC_Raw_Rx_Packet_t rx_packet = {};

    if ((data == NULL) || (this->pc_comm.xraw_queue == NULL))
    {
        return;
    }

    rx_packet.rx_data_size = (uint16_t)((length > USB_CDC_RX_SINGLE_PACKET_SIZE) ? USB_CDC_RX_SINGLE_PACKET_SIZE : length);
    memcpy(rx_packet.rx_data, data, rx_packet.rx_data_size);
    xQueueSendFromISR(this->pc_comm.xraw_queue, &rx_packet, xHigherPriorityTaskWoken);
}

/**
 * @brief          发送一帧PC USB协议数据
 * @param[in]      data        数据载荷首地址
 *                 data_length 数据载荷长度
 * @retval         发送结果
 */
bool Class_PC_USB_Communication::PC_Data_Transmit(uint8_t *data, uint16_t data_length)
{
    return USB_CDC_Send_Data(&this->usb_cdc_manage_object, data, data_length);
}
