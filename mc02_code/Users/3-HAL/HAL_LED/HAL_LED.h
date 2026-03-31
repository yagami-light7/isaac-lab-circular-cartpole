/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_LED.h
  * @brief      LED 硬件抽象层头文件
  * @note       Hardware Abstract Layer
  *             1. 负责 WS2812 灯珠的底层时序发送
  *             2. 直接绑定 CubeMX 生成的 SPI6 外设
  *             3. 不包含灯效状态机，灯效逻辑由 0-APL 层完成
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. migrate LED driver to new framework
  *
  @verbatim
  ==============================================================================
  * 这里保留旧工程中 WS2812 的 SPI 发送策略，但把驱动职责收敛到 3-HAL。
  * 上层只需要传入 RGB 三个分量，不关心底层 SPI 码型编码细节。
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
  * @brief          设置 WS2812 RGB 颜色
  * @param[in]      red   红色分量，范围 0~255
  *                 green 绿色分量，范围 0~255
  *                 blue  蓝色分量，范围 0~255
  * @retval         none
  */
extern void HAL_LED_WS2812_Set_RGB(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif
