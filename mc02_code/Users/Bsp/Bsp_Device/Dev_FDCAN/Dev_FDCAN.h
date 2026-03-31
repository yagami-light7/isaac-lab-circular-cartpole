/**
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  * @file       Dev_FDCAN.h
  * @brief      旧框架 FDCAN 设备层接口
  * @note       该文件保留给旧工程使用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-28-2024     Light           1. done
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  */
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "string.h"
#include "Board_FDCAN.h"
#include "cmsis_os.h"
#include "Alg_UserLib.h"

/******************************* CAN 总线别名 *******************************/
#define CHASSIS_CAN      hfdcan1
#define Large_Joint_CAN  hfdcan2
#define Small_Joint_CAN  hfdcan3

/******************************* 达妙电机模式与量程 *******************************/
#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPEED_MODE 0x200

#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -20.5f
#define V_MAX   20.5f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN  -32.0f
#define T_MAX   32.0f

/******************************* CAN ID 定义 *******************************/
typedef enum
{
    FDCAN_CHASSIS_RESET_ID = 0x700,
    FDCAN_CHASSIS_ALL_ID   = 0x200,
    FDCAN_3508_M1_ID       = 0x201,
    FDCAN_3508_M2_ID       = 0x202,
    FDCAN_3508_M3_ID       = 0x203,
    FDCAN_3508_M4_ID       = 0x204,

    FDCAN_LARGE_JOINT_ALL_ID = 0x000,
    FDCAN_LARGE_JOINT_1_ID   = 0x001,
    FDCAN_LARGE_JOINT_2_ID   = 0x002,
    FDCAN_LARGE_JOINT_3_ID   = 0x003,

    FDCAN_SMALL_JOINT_ALL_ID = 0x1FF,
    FDCAN_SMALL_JOINT_1_ID   = 0x205,
    FDCAN_SMALL_JOINT_2_ID   = 0x206,
    FDCAN_SMALL_JOINT_3_ID   = 0x207,

    FDCAN_SUPERCAP_TX_ID = 0x210,
    FDCAN_SUPERCAP_RX_ID = 0x211,
} can_msg_id_e;

/******************************* 达妙电机模式 *******************************/
typedef enum
{
    MENU_MODE,
    MADA_MODE,
} MIT_Motor_Mode;

typedef struct
{
    uint8_t id;
    uint8_t err_state;
    int32_t p_int;
    int32_t v_int;
    int32_t t_int;
    int32_t kp_int;
    int32_t kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Temp;
} MIT_Motor_Measure_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    float totalAngle;
    int turnCount;
} DJI_Motor_Measure_t;

typedef struct
{
    uint16_t battery_voltage;
    int16_t capacitance_voltage;
    int16_t given_current;
    uint8_t capacitance_percentage;
    uint8_t power_set;
} SupCap_Measure_t;

extern DJI_Motor_Measure_t motor_chassis[4];
extern DJI_Motor_Measure_t motor_s_joint[3];
extern MIT_Motor_Measure_t motor_l_joint[3];
extern SupCap_Measure_t sup_cap[1];

#ifdef __cplusplus
extern "C" {
#endif

extern void MIT_Motor_Init(void);
extern void Awake_MIT_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
extern void Enable_MIT_Motor_Mode(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t mode_id);
extern void Disable_MIT_Motor_Mode(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t mode_id);
extern void MIT_Ctrl_L_Joint(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
extern void Pos_Speed_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel);
extern void Speed_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float vel);
extern void MIT_Motor_Set_Zero(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
extern void FDCAN_Cmd_Chassis_Reset_ID(void);
extern void FDCAN_cmd_Chassis(int16_t dji_motor1, int16_t dji_motor2, int16_t dji_motor3, int16_t dji_motor4);
extern void FDCAN_cmd_Small_Joint(int16_t wrist_6020, int16_t wrist_2006_l, int16_t wrist_2006_r);
extern void FDCAN_cmd_SuperCap(int16_t power_set);
extern const DJI_Motor_Measure_t *Get_Chassis_Motor_Measure_Point(uint8_t i);
extern const MIT_Motor_Measure_t *Get_Large_Joint_Motor_Measure_Point(uint8_t i);
extern const DJI_Motor_Measure_t *Get_Small_Joint_Motor_Measure_Point(uint8_t i);
extern const SupCap_Measure_t *Get_Supercap_Measure_Point(void);
extern int32_t float_to_uint(float x_float, float x_min, float x_max, uint32_t bits);
extern float uint_to_float(uint32_t x_int, float x_min, float x_max, uint32_t bits);
extern float Hex_To_Float(uint32_t *Byte, uint32_t num);
extern uint32_t FloatTohex(float HEX);

#ifdef __cplusplus
}
#endif

#endif
