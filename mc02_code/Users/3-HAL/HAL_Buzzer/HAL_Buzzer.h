/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_Buzzer.h
  * @brief      蜂鸣器硬件抽象层头文件
  * @note       Hardware Abstract Layer
  *             1. 负责 TIM12 CH2 PWM 蜂鸣器输出
  *             2. 对上保留旧工程的 Buzzer_On / Buzzer_OFF 接口名称
  *             3. 兼容校准流程中按次数提示的蜂鸣器函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate buzzer driver to new framework
  *
  @verbatim
  ==============================================================================
  * 这里刻意保留旧接口名字，是为了让 App_Calibrate / App_Gimbal_Behaviour
  * 和 Buzzer_Task 的调用点不需要做大面积重命名，只替换头文件来源即可。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief          打开蜂鸣器
  * @param[in]      psc  TIM12 预分频系数
  *                 pwm  PWM 比较值
  * @retval         none
  */
extern void Buzzer_On(uint16_t psc, uint16_t pwm);

/**
  * @brief          关闭蜂鸣器
  * @param[in]      none
  * @retval         none
  */
extern void Buzzer_OFF(void);

/**
  * @brief          蜂鸣器校准提示函数
  * @param[in]      num 对应的提示次数
  * @retval         none
  */
extern void joint_cali_start_buzzer(uint8_t num);

#ifdef __cplusplus
}
#endif
