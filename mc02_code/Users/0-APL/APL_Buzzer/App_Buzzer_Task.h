/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       App_Buzzer_Task.h
  * @brief      蜂鸣器任务头文件
  * @note       Application Layer
  *             1. 保持与 freertos.c 中原有任务函数名一致
  *             2. 开机提示音与基础报警逻辑迁移到新框架 0-APL
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate buzzer task to new framework
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief          蜂鸣器任务，负责播放开机音与报警音
  * @param[in]      pvParameters 任务参数
  * @retval         none
  */
extern void Buzzer_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif
