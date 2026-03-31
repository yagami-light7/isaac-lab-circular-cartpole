#ifndef MT6701_CBOARD_UART_BRIDGE_APP_MT6701_TASK_H
#define MT6701_CBOARD_UART_BRIDGE_APP_MT6701_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

#define MT6701_UART_TX_CMD_ID              0x0202U
#define MT6701_UART_FRAME_SOF              0xA5U
#define MT6701_UART_FRAME_HEADER_LENGTH    5U
#define MT6701_UART_FRAME_CMD_LENGTH       2U
#define MT6701_UART_FRAME_TAIL_LENGTH      2U
#define MT6701_TASK_PERIOD_MS              4U
#define MT6701_TX_UART_HANDLE              huart1
#define MT6701_PI_F                        3.14159265358979f
#define MT6701_SPEED_FILTER_ALPHA          0.30f

/* 串口负载仅发送两个浮点量：角度和角速度。 */
#define MT6701_UART_PAYLOAD_LENGTH         (sizeof(float) + sizeof(float))
#define MT6701_UART_FRAME_LENGTH           (MT6701_UART_FRAME_HEADER_LENGTH + MT6701_UART_FRAME_CMD_LENGTH + MT6701_UART_PAYLOAD_LENGTH + MT6701_UART_FRAME_TAIL_LENGTH)

typedef struct
{
    float angle_rad;
    float speed_rad_s;
} MT6701_Uart_Payload_t;

void MT6701_Task(void *argument);

#endif
