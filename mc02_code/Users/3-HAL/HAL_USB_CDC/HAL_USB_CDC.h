/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_USB_CDC.h
  * @brief      USB CDC外设再封装
  * @note       Hardware Abstract Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. 基于当前CubeMX生成的USB OTG驱动重建
  *
  @verbatim
  ==============================================================================
  * 本模块仅完成 USB CDC 设备层初始化、设备驱动以及线程中的数据发送。
  * USB 接收中断不在 HAL 层直接做协议解析或队列搬运，而是通过项目回调钩子
  * 将原始二进制数据转交给 APL 层继续处理。
  *
  * 当前 USB OTG HS 工作在 Full-Speed 模式，单次 CDC OUT 回调的有效载荷按 64 字节处理。
  * 若后续上层协议帧长度大于 64 字节，应在通信线程中自行做分包重组。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "cmsis_os.h"

#define USB_CDC_RX_SINGLE_PACKET_SIZE   64U     // USB FS单个CDC包最大有效载荷

/**
 * @brief USB CDC处理结构体
 */
typedef struct
{
    uint8_t init_flag;
} USB_CDC_Manage_Object_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief          USB CDC初始化
* @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
* @retval         none
*/
extern void USB_CDC_Init(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object);

/**
 * @brief          USB CDC接收回调钩子
 * @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
 *                 data                    原始接收数据首地址
 *                 length                  原始接收数据长度
 *                 xHigherPriorityTaskWoken 任务切换标志
 * @retval         none
 */
extern void USB_CDC_RxCallback(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object,
                               uint8_t *data,
                               uint16_t length,
                               BaseType_t *xHigherPriorityTaskWoken);

/**
 * @brief          USB CDC发送数据
 * @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
 *                 data                    数据首地址
 *                 length                  数据长度
 * @retval         发送结果
 */
extern bool USB_CDC_Send_Data(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif
