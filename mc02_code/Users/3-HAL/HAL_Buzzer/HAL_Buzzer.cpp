/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_Buzzer.cpp
  * @brief      蜂鸣器硬件抽象层源文件
  * @note       Hardware Abstract Layer
  *             1. 使用 CubeMX 生成的 TIM12 CH2 输出蜂鸣器 PWM
  *             2. 保留旧工程蜂鸣器接口，方便上层平滑迁移
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate buzzer driver to new framework
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HAL_Buzzer.h"

#include "tim.h"

/**
  * @brief          打开蜂鸣器
  * @param[in]      psc  TIM12 预分频系数
  *                 pwm  PWM 比较值
  * @retval         none
  */
void Buzzer_On(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_SET_PRESCALER(&htim12, psc);
    __HAL_TIM_SET_COUNTER(&htim12, 0U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm);
}

/**
  * @brief          关闭蜂鸣器
  * @param[in]      none
  * @retval         none
  */
void Buzzer_OFF(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
}

/**
  * @brief          蜂鸣器校准提示函数
  * @param[in]      num 对应的提示次数
  * @retval         none
  */
void joint_cali_start_buzzer(uint8_t num)
{
    static uint8_t show_num = 0U;
    static uint8_t stop_num = 100U;
    static uint8_t tick = 0U;

    if (show_num == 0U && stop_num == 0U)
    {
        show_num = (uint8_t)(num + 1U);
        stop_num = 100U;
    }
    else if (show_num == 0U)
    {
        stop_num--;
        Buzzer_OFF();
    }
    else
    {
        tick++;
        if (tick < 50U)
        {
            Buzzer_OFF();
        }
        else if (tick < 100U)
        {
            Buzzer_On(1U, 30000U);
        }
        else
        {
            tick = 0U;
            show_num--;
        }
    }
}
