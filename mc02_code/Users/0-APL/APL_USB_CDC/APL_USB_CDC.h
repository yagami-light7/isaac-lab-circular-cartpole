/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_USB_CDC.h
  * @brief      USB CDC应用层回调头文件
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-29-2026     Codex           1. 新增USB CDC中断入口重定向层
  *
  @verbatim
  ==============================================================================
  * 本层只接管 HAL_USB_CDC 提供的接收回调钩子，将原始 USB 二进制数据转交给
  * Commu_Hub 持有的 PC_USB_Module。协议拼帧和业务解析统一放在 APL_Commu_Hub。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
