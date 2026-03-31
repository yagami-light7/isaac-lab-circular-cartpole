/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       App_Buzzer_Task.cpp
  * @brief      蜂鸣器任务源文件
  * @note       Application Layer
  *             1. 保留旧工程开机提示音
  *             2. 当前错误报警逻辑保留接口，但默认不启用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate buzzer task to new framework
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "App_Buzzer_Task.h"

#include "cmsis_os.h"

#include "HAL_Buzzer.h"

static void buzzer_warn_error(uint8_t num);
static void play_startupsound(void);
static void low_power_warning(void);

/**
 * @brief 蜂鸣器任务，负责播放开机音与报警音
 * @param pvParameters 任务参数
 */
extern "C" void Buzzer_Task(void *pvParameters)
{
    static uint8_t error = 0U;
    static uint8_t last_error = 0U;
    static uint8_t error_num = 0U;

    (void)pvParameters;
    (void)error;
    (void)last_error;
    (void)error_num;

    play_startupsound();
    osDelay(1000U);

    while (1)
    {
        osDelay(10U);
    }
}

/**
 * @brief 使得蜂鸣器按次数报警
 * @param num 响声次数
 */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0U;
    static uint8_t stop_num = 100U;
    static uint8_t tick = 0U;

    if (show_num == 0U && stop_num == 0U)
    {
        show_num = num;
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

/**
 * @brief 播放开机提示音
 */
static void play_startupsound(void)
{
    Buzzer_On(100U, 1500U);
    osDelay(200U);
    Buzzer_OFF();

    Buzzer_On(200U, 1500U);
    osDelay(200U);
    Buzzer_OFF();

    Buzzer_On(150U, 1500U);
    osDelay(200U);
    Buzzer_OFF();

    Buzzer_On(100U, 1500U);
    osDelay(200U);
    Buzzer_OFF();

    Buzzer_On(50U, 1500U);
    osDelay(200U);
    Buzzer_OFF();

    osDelay(500U);

    Buzzer_On(300U, 1000U);
    osDelay(200U);
    Buzzer_OFF();
}

/**
 * @brief 低电量报警
 */
static void low_power_warning(void)
{
    Buzzer_On(1U, 30000U);
    osDelay(500U);
    Buzzer_OFF();
    osDelay(500U);
    Buzzer_On(1U, 30000U);
    osDelay(500U);
    Buzzer_OFF();
    osDelay(500U);
}
