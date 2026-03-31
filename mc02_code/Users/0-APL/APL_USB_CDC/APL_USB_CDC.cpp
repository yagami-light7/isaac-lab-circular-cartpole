/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_USB_CDC.cpp
  * @brief      USB CDC应用层回调实现
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-29-2026     Codex           1. 新增USB CDC中断入口重定向层
  *
  @verbatim
  ==============================================================================
  * 本文件与 APL_USART / APL_FDCAN 的职责边界保持一致：
  * 1. APL 只承接 HAL 的弱回调入口
  * 2. 中断中仅转发原始包，不做任何 A5 协议解析
  * 3. 原始包队列由 HDL 层创建，协议解析由 Commu_Hub 在线程中完成
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "APL_USB_CDC.h"

#include "APL_Commu_Hub.h"
#include "HAL_USB_CDC.h"

/**
  * @brief          USB CDC接收回调
  * @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
  *                 data                    原始接收数据首地址
  *                 length                  原始接收数据长度
  *                 xHigherPriorityTaskWoken 任务切换标志
  * @retval         none
  */
extern "C" void USB_CDC_RxCallback(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object,
                                   uint8_t *data,
                                   uint16_t length,
                                   BaseType_t *xHigherPriorityTaskWoken)
{
    (void)USB_CDC_Manage_Object;
    Commu_Hub.PC_USB_Module.Push_Raw_Data_From_ISR(data, length, xHigherPriorityTaskWoken);
}
