/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       App_LED_RGB_Flow_Task.h
  * @brief      RGB LED 灯效任务头文件
  * @note       Application Layer
  *             1. 保持与 freertos.c 中原有任务函数名一致
  *             2. 具体灯效逻辑迁移到新框架 0-APL
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate LED task to new framework
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_DELAY_TIME 10U

/**
  * @brief          LED RGB 灯效任务
  * @param[in]      pvParameters 任务参数
  * @retval         none
  */
extern void LED_RGB_Flow_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif
