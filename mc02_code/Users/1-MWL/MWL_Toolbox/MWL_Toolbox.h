/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     03-31-2026     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once
#include <type_traits>



/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"


/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */


/**
 * @brief CPP部分
 */
#ifdef __cplusplus

template <typename T>
constexpr T MWL_Clamp(T value, T min_value, T max_value)
{
    static_assert(std::is_arithmetic<T>::value, "MWL_Clamp only supports arithmetic types");

    // 若上下限传反，自动交换，避免调用方出错
    if (min_value > max_value)
    {
        T temp = min_value;
        min_value = max_value;
        max_value = temp;
    }

    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
