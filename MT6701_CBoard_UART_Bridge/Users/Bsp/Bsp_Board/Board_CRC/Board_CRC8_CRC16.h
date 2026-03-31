/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       crc8_crc16.c/h
  * @brief      crc8 and crc16 calculate function, verify function, append function.
  *             crc8鍜宑rc16璁＄畻鍑芥暟,鏍￠獙鍑芥暟,娣诲姞鍑芥暟
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef INFANTRY_ROBOT_BOARD_CRC8_CRC16_H
#define INFANTRY_ROBOT_BOARD_CRC8_CRC16_H

#include "main.h"

/**
  * @brief          calculate the crc8
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      ucCRC8: init CRC8
  * @retval         calculated crc8
  */
/**
  * @brief          璁＄畻CRC8
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @param[in]      ucCRC8:鍒濆CRC8
  * @retval         璁＄畻瀹岀殑CRC8
  */
extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);

/**
  * @brief          CRC8 verify function
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
/**
  * @brief          CRC8鏍￠獙鍑芥暟
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         鐪熸垨鑰呭亣
  */
extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
  * @brief          append CRC8 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          娣诲姞CRC8鍒版暟鎹殑缁撳熬
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         none
  */
extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
  * @brief          calculate the crc16
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      wCRC: init CRC16
  * @retval         calculated crc16
  */
/**
  * @brief          璁＄畻CRC16
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @param[in]      wCRC:鍒濆CRC16
  * @retval         璁＄畻瀹岀殑CRC16
  */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

/**
  * @brief          CRC16 verify function
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
/**
  * @brief          CRC16鏍￠獙鍑芥暟
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         鐪熸垨鑰呭亣
  */
extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

/**
  * @brief          append CRC16 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          娣诲姞CRC16鍒版暟鎹殑缁撳熬
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         none
  */
extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);

#endif

