#include "App_MT6701_Task.h"

#include "Board_CRC8_CRC16.h"
#include "Dev_MT6701.h"
#include "i2c.h"
#include "string.h"

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef MT6701_TX_UART_HANDLE;

static float MT6701_Wrap_Radian_Delta(float delta);
static uint16_t MT6701_Build_Tx_Frame(uint8_t *tx_buffer, float angle_rad, float speed_rad_s);

Dev_MT6701_Angle_t angle = {0U};
/**
 * @brief          MT6701 采样与串口发送任务
 * @param[in]      argument 未使用
 * @retval         none
 */
void MT6701_Task(void *argument)
{
    uint8_t tx_buffer[MT6701_UART_FRAME_LENGTH] = {0U};
    uint16_t tx_length = 0U;
    uint32_t current_tick = 0U;
    uint32_t previous_tick = 0U;
    float previous_angle_rad = 0.0f;
    float raw_speed_rad_s = 0.0f;
    float filtered_speed_rad_s = 0.0f;
    uint8_t has_previous_sample = 0U;

    osDelay(50U);

    while (1)
    {
        if (Dev_MT6701_Read_Angle(&hi2c2, &angle) == HAL_OK)
        {
            current_tick = HAL_GetTick();

            if (has_previous_sample != 0U)
            {
                float delta_time_s = (float)(current_tick - previous_tick) * 0.001f;
                if (delta_time_s > 0.0f)
                {
                    float delta_angle = MT6701_Wrap_Radian_Delta(angle.angle_rad - previous_angle_rad);
                    raw_speed_rad_s = delta_angle / delta_time_s;
                    filtered_speed_rad_s += MT6701_SPEED_FILTER_ALPHA * (raw_speed_rad_s - filtered_speed_rad_s);
                }
            }
            else
            {
                has_previous_sample = 1U;
                filtered_speed_rad_s = 0.0f;
            }

            previous_angle_rad = angle.angle_rad;
            previous_tick = current_tick;

            tx_length = MT6701_Build_Tx_Frame(tx_buffer, angle.angle_rad, filtered_speed_rad_s);
            if (tx_length > 0U)
            {
                HAL_UART_Transmit(&MT6701_TX_UART_HANDLE, tx_buffer, tx_length, 100U);
            }
        }

        osDelay(MT6701_TASK_PERIOD_MS);
    }
}

/**
 * @brief          将角度差约束到 [-pi, pi]，避免跨圈时速度突变
 * @param[in]      delta 输入角度差，单位 rad
 * @retval         约束后的角度差
 */
static float MT6701_Wrap_Radian_Delta(float delta)
{
    while (delta > MT6701_PI_F)
    {
        delta -= (2.0f * MT6701_PI_F);
    }

    while (delta < -MT6701_PI_F)
    {
        delta += (2.0f * MT6701_PI_F);
    }

    return delta;
}

/**
 * @brief          组装 MT6701 串口发送帧
 * @param[in]      tx_buffer 发送缓存区
 * @param[in]      angle_rad 当前角度，单位 rad
 * @param[in]      speed_rad_s 当前角速度，单位 rad/s
 * @retval         组装后的完整帧长度
 */
static uint16_t MT6701_Build_Tx_Frame(uint8_t *tx_buffer, float angle_rad, float speed_rad_s)
{
    static uint8_t seq = 0U;
    uint16_t cmd_id = MT6701_UART_TX_CMD_ID;
    uint16_t payload_length = MT6701_UART_PAYLOAD_LENGTH;
    uint16_t offset = 0U;

    if (tx_buffer == NULL)
    {
        return 0U;
    }

    memset(tx_buffer, 0, MT6701_UART_FRAME_LENGTH);

    tx_buffer[0] = MT6701_UART_FRAME_SOF;
    memcpy(&tx_buffer[1], &payload_length, sizeof(payload_length));
    tx_buffer[3] = seq++;
    append_CRC8_check_sum(tx_buffer, MT6701_UART_FRAME_HEADER_LENGTH);

    offset = MT6701_UART_FRAME_HEADER_LENGTH;
    memcpy(&tx_buffer[offset], &cmd_id, sizeof(cmd_id));
    offset += sizeof(cmd_id);

    memcpy(&tx_buffer[offset], &angle_rad, sizeof(angle_rad));
    offset += sizeof(angle_rad);
    memcpy(&tx_buffer[offset], &speed_rad_s, sizeof(speed_rad_s));

    append_CRC16_check_sum(tx_buffer, MT6701_UART_FRAME_LENGTH);
    return MT6701_UART_FRAME_LENGTH;
}
