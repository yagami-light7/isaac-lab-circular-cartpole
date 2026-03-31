#include "Dev_MT6701.h"

/**
 * @brief          读取 MT6701 的 14bit 原始角度值
 * @param[in]      hi2c I2C 句柄
 * @param[out]     raw_angle 原始角度值，范围 0~16383
 * @retval         HAL 状态码
 */
HAL_StatusTypeDef Dev_MT6701_Read_Raw_Angle(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t rx_buffer[2] = {0U};

    if ((hi2c == NULL) || (raw_angle == NULL))
    {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(
        hi2c,
        DEV_MT6701_SLAVE_ADDR,
        DEV_MT6701_REG_ANGLE_14BIT,
        I2C_MEMADD_SIZE_8BIT,
        rx_buffer,
        2U,
        DEV_MT6701_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        return status;
    }

    *raw_angle = (uint16_t)(((uint16_t)rx_buffer[0] << 6U) | ((uint16_t)rx_buffer[1] >> 2U));
    return HAL_OK;
}

/**
 * @brief          读取 MT6701 角度信息，并同时换算为角度制和弧度制
 * @param[in]      hi2c I2C 句柄
 * @param[out]     angle 角度信息结构体
 * @retval         HAL 状态码
 */
HAL_StatusTypeDef Dev_MT6701_Read_Angle(I2C_HandleTypeDef *hi2c, Dev_MT6701_Angle_t *angle)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint16_t raw_angle = 0U;

    if (angle == NULL)
    {
        return HAL_ERROR;
    }

    status = Dev_MT6701_Read_Raw_Angle(hi2c, &raw_angle);
    if (status != HAL_OK)
    {
        return status;
    }

    angle->raw_angle = raw_angle;
    angle->angle_deg = (float)raw_angle * DEV_MT6701_DEG_PER_COUNT;
    angle->angle_rad = (float)raw_angle * DEV_MT6701_RAD_PER_COUNT;
    return HAL_OK;
}