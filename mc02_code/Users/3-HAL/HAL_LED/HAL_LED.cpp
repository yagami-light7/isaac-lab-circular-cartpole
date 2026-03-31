/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_LED.cpp
  * @brief      LED 硬件抽象层源文件
  * @note       Hardware Abstract Layer
  *             1. 使用 CubeMX 生成的 SPI6 发送 WS2812 码流
  *             2. 不参与灯效逻辑，只负责单次颜色刷新
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate LED driver to new framework
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HAL_LED.h"

#include "spi.h"

/**
 * @brief WS2812 编码中的低电平码
 */
#define HAL_LED_WS2812_LOW_LEVEL   0xC0U

/**
 * @brief WS2812 编码中的高电平码
 */
#define HAL_LED_WS2812_HIGH_LEVEL  0xF0U

/**
  * @brief          设置 WS2812 RGB 颜色
  * @param[in]      red   红色分量，范围 0~255
  *                 green 绿色分量，范围 0~255
  *                 blue  蓝色分量，范围 0~255
  * @retval         none
  */
void HAL_LED_WS2812_Set_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t tx_buffer[24];
    uint8_t reset_data = 0U;
    uint8_t i = 0U;

    for (i = 0U; i < 8U; i++)
    {
        tx_buffer[7U - i] = (uint8_t)((((green >> i) & 0x01U) != 0U) ? (HAL_LED_WS2812_HIGH_LEVEL >> 1U) : (HAL_LED_WS2812_LOW_LEVEL >> 1U));
        tx_buffer[15U - i] = (uint8_t)((((red >> i) & 0x01U) != 0U) ? (HAL_LED_WS2812_HIGH_LEVEL >> 1U) : (HAL_LED_WS2812_LOW_LEVEL >> 1U));
        tx_buffer[23U - i] = (uint8_t)((((blue >> i) & 0x01U) != 0U) ? (HAL_LED_WS2812_HIGH_LEVEL >> 1U) : (HAL_LED_WS2812_LOW_LEVEL >> 1U));
    }

    while (hspi6.State != HAL_SPI_STATE_READY)
    {
    }

    HAL_SPI_Transmit(&hspi6, tx_buffer, (uint16_t)sizeof(tx_buffer), 0xFFFFU);

    for (i = 0U; i < 100U; i++)
    {
        HAL_SPI_Transmit(&hspi6, &reset_data, 1U, 0xFFFFU);
    }
}
