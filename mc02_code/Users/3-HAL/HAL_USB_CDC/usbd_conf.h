/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       usbd_conf.h
  * @brief      USB Device中间件兼容头文件
  * @note       该文件仅用于兼容ST USB Device中间件默认包含的usbd_conf.h
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-29-2026     Codex           1. 重写为兼容转发壳文件
  *
  @verbatim
  ==============================================================================
  * 工程内部实际使用的配置文件为 HAL_USB_CDC_Conf.h。
  * 保留本文件仅为了兼容ST中间件源码中的 #include "usbd_conf.h"。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include "HAL_USB_CDC_Conf.h"