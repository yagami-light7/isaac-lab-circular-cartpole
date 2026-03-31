/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_USB_CDC_PC.h
  * @brief      PC端USB CDC协议类
  * @note       Hardware Driver Layer硬件驱动层
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     Mar-29-2026     Codex           1. 重构为驱动实例自持有与原始接收队列
  *
  @verbatim
  ==============================================================================
  * 本模块只负责三件事：
  * 1. 持有 HAL_USB_CDC 驱动实例，并在线程上下文完成初始化
  * 2. 创建原始 USB CDC 接收队列，供 APL 层搬运中断收到的二进制包
  * 3. 提供统一的 A5 帧发送接口，供上层回传状态与调试数据
  *
  * A5 协议的流缓冲拼帧、CRC 校验、命令分发均放在 APL_Commu_Hub 中实现。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <string.h>

#include "main.h"
#include "HAL_USB_CDC.h"
#include "HDL_USB_CDC_PC_Message.h"

#define USB_PC_SOF                         0xA5U
#define USB_PC_FRAME_HEADER_LENGTH         5U
#define USB_PC_CMD_ID_LENGTH               2U
#define USB_PC_FRAME_TAIL_LENGTH           2U
#define USB_PC_MAX_DATA_LENGTH             USB_CDC_RX_SINGLE_PACKET_SIZE
#define USB_PC_RX_RAW_QUEUE_ITEM_NUM       8U
#define USB_PC_STREAM_BUFFER_SIZE          256U

#pragma pack(push, 1)
/**
 * @brief USB CDC PC协议使用的A5帧头
 */
typedef __packed struct
{
    uint8_t SOF;            // 起始字节，固定值为0xA5
    uint16_t data_length;   // 数据帧中data的长度
    uint8_t seq;            // 包序号
    uint8_t CRC8;           // 帧头CRC8校验

} usb_pc_frame_header_t;    // USB CDC帧头结构体

#pragma pack(pop)

/**
 * @brief USB CDC协议解析后的数据帧对象
 */
typedef struct
{
    usb_pc_frame_header_t frame_header;
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[USB_PC_MAX_DATA_LENGTH];    // 数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验

} USB_PC_Data_Frame_t; // USB CDC 消息结构体

/**
 * @brief USB CDC原始接收数据包
 */
typedef struct
{
    uint16_t rx_data_size;
    uint8_t rx_data[USB_CDC_RX_SINGLE_PACKET_SIZE];
} USB_PC_Raw_Rx_Packet_t;

/**
 * @brief USB CDC PC通信队列对象
 */
typedef struct
{
    QueueHandle_t xraw_queue;
} usb_pc_comm_t;

#ifdef __cplusplus
class Class_PC_USB_Communication
{
public:
    void Init(void);
    void Push_Raw_Data_From_ISR(uint8_t *data, uint16_t length, BaseType_t *xHigherPriorityTaskWoken);
    bool PC_Data_Transmit(uint8_t *data, uint16_t data_length);

    usb_pc_comm_t pc_comm;

private:
    USB_CDC_Manage_Object_t usb_cdc_manage_object;
    uint8_t tx_seq;
};
#endif
