/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       App_LED_RGB_Flow_Task.cpp
  * @brief      RGB LED 灯效任务源文件
  * @note       Application Layer
  *             1. 保留旧任务的基础流光效果
  *             2. 底层颜色刷新由 HAL_LED 完成
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate LED task to new framework
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "App_LED_RGB_Flow_Task.h"

#include "cmsis_os.h"

#include "HAL_LED.h"

/**
  * @brief          LED RGB 灯效任务
  * @param[in]      pvParameters 任务参数
  * @retval         none
  */
extern "C" void LED_RGB_Flow_Task(void *pvParameters)
{
    uint8_t red = 1U;
    uint8_t green = 1U;
    uint8_t blue = 1U;

    (void)pvParameters;

    while (1)
    {
        HAL_LED_WS2812_Set_RGB(red, green, blue);

        red++;
        green = (uint8_t)(100U - red);
        blue += (uint8_t)(100U - red - green);
        vTaskDelay(LED_DELAY_TIME);

        red++;
        green++;
        blue++;
    }
}
