#ifndef INFANTRY_ROBOT_DEV_MT6701_H
#define INFANTRY_ROBOT_DEV_MT6701_H

#include "main.h"
#include "i2c.h"

#define DEV_MT6701_SLAVE_ADDR          (0x06U << 1)
#define DEV_MT6701_TIMEOUT_MS          50U
#define DEV_MT6701_REG_ANGLE_14BIT     0x03U
#define DEV_MT6701_RAW_RESOLUTION      16384.0f
#define DEV_MT6701_DEG_PER_COUNT       (360.0f / DEV_MT6701_RAW_RESOLUTION)
#define DEV_MT6701_RAD_PER_COUNT       (6.28318530718f / DEV_MT6701_RAW_RESOLUTION)

typedef struct
{
    uint16_t raw_angle;
    float angle_deg;
    float angle_rad;
} Dev_MT6701_Angle_t;

HAL_StatusTypeDef Dev_MT6701_Read_Raw_Angle(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle);
HAL_StatusTypeDef Dev_MT6701_Read_Angle(I2C_HandleTypeDef *hi2c, Dev_MT6701_Angle_t *angle);

#endif
