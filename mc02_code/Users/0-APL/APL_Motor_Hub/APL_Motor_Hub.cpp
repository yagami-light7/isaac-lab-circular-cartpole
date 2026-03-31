/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     03-28-2026     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "APL_Motor_Hub.h"
#include "HDL_Damiao_Motor.h"
#include "arm_math.h"

/**
 * @brief   电机控制中心线程
 */
void _Thread_Motor_Hub_(void *pvParameters)
{
    HDL_Damiao_Motor_FDCAN1_Init();

    while (1)
    {
//        static double tick = 0;
//        float pos = arm_sin_f32(tick);
//        tick += 0.01f;
//        HDL_Damiao_Motor_Send_MIT_Command(&HDL_Damiao_Motor_Bus_FDCAN1,0x01U,
//                                          pos, 0.0f,
//                                          10.0f, 1.0f,
//                                          0.0f);
        vTaskDelay(1);
    }
}
