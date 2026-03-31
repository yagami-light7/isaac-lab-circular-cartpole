/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_USB_CDC_Conf.h
  * @brief      USB Device中间件配置头文件
  * @note       本文件由HAL_USB_CDC模块维护，用于适配当前工程中的USB CDC设备栈
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-29-2026     Codex           1. 按新框架命名重构USB Device配置头文件
  *
  @verbatim
  ==============================================================================
  * 本文件保存USB Device Core / CDC Class所需的基础配置项、内存接口和调试宏。
  * 为了兼容ST中间件的默认include链，目录下仍保留一个usbd_conf.h壳文件，
  * 该壳文件仅负责转发到本文件。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define USBD_MAX_NUM_INTERFACES         1U
#define USBD_MAX_NUM_CONFIGURATION      1U
#define USBD_MAX_STR_DESC_SIZ           128U
#define USBD_DEBUG_LEVEL                0U
#define USBD_LPM_ENABLED                0U
#define USBD_SELF_POWERED               1U

#define DEVICE_FS                       0U
#define DEVICE_HS                       1U

#define USBD_malloc                     (void *)USBD_static_malloc
#define USBD_free                       USBD_static_free
#define USBD_memset                     memset
#define USBD_memcpy                     memcpy
#define USBD_Delay                      HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)                printf(__VA_ARGS__); printf("\n")
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...)                printf("ERROR: "); printf(__VA_ARGS__); printf("\n")
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...)                printf("DEBUG : "); printf(__VA_ARGS__); printf("\n")
#else
#define USBD_DbgLog(...)
#endif

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#ifdef __cplusplus
}
#endif
