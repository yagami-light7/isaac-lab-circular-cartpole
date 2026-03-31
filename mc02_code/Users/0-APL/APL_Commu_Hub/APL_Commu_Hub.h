#pragma once

#include "main.h"
#include "HDL_VT.h"
#include "HDL_USB_CDC_PC.h"
#include "HDL_Damiao_Motor_App.h"

#define JOINTS_NUM                              6
#define COMMU_HUB_INIT_TIME                     501
#define COMMU_HUB_CONTROL_TIME                  4
#define COMMU_HUB_MIT_FIXED_KP                  8.0f
#define COMMU_HUB_MIT_FIXED_KD                  1.0f
#define COMMU_HUB_MT6701_UART_RX_DATA_SIZE      32U
#define COMMU_HUB_MT6701_UART_RX_QUEUE_ITEM_NUM 8U
#define COMMU_HUB_MT6701_UART_STREAM_BUFFER_SIZE 128U
#define COMMU_HUB_MT6701_UART_PAYLOAD_LENGTH    (2U * sizeof(float))
#define COMMU_HUB_MT6701_ZERO_OFFSET_RAD        2.4785f
#define COMMU_HUB_MT6701_SPEED_FILTER_ALPHA     0.35f
#define COMMU_HUB_PI_F                          3.14159265358979f

#ifndef RC_CH_VALUE_MIN
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#endif

#ifndef RC_CH_VALUE_OFFSET
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#endif

#ifndef RC_CH_VALUE_MAX
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
#endif

#ifndef MOUSE_VALUE_MIN
#define MOUSE_VALUE_MIN         ((int16_t)-32768)
#endif

#ifndef MOUSE_VALUE_MAX
#define MOUSE_VALUE_MAX         ((int16_t)32767)
#endif

/**
 * @brief DR16
 */
typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
        char last_s[2];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
        uint16_t last_v;
    } key;

} dr16_control_t;

/**
 * @brief 自定义控制器
 */
typedef struct
{
    float custom_angle_set[JOINTS_NUM];
} custom_control_t;

/**
 * @brief 0x304键鼠信息
 */
typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;

    int8_t last_left_button_down;
    int8_t last_right_button_down;
    uint16_t last_keyboard_value;

} remote_control_t;

/**
 * @brief VT13图传接收端
 */
typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;

    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    uint8_t mode_sw;
    uint8_t pause;
    uint8_t fn_1;
    uint8_t fn_2;
    int16_t wheel;
    uint8_t trigger;

    float mouse_x;
    float mouse_y;
    float mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t mouse_middle;
    uint16_t key;

    uint16_t crc16;

    uint16_t chassis_key_last;
    uint16_t gimbal_key_last;
    uint16_t mec_arm_key_last;

    uint8_t last_fn_1;
    uint8_t last_fn_2;
    uint8_t last_pause;
    uint8_t last_trigger;

} vt_rc_control_t;

/**
 * @brief 串口10原始接收数据包
 */
typedef struct
{
    uint16_t rx_data_size;
    uint8_t rx_data[COMMU_HUB_MT6701_UART_RX_DATA_SIZE];
} MT6701_UART_Raw_Rx_Packet_t;

/**
 * @brief 串口10解析后的MT6701状态
 */
typedef struct
{
    float angle_rad;
    float speed_rad_s;
} MT6701_UART_State_t;

typedef struct
{
    uint8_t pole_frame[USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + 5U * sizeof(float) + USB_PC_FRAME_TAIL_LENGTH];
    uint16_t pole_frame_length;
} tx_data_t;

#ifdef __cplusplus

class Class_Commu_Hub
{
public:
    void Init(void);
    void Data_Analysis(void);   // 解析PC USB原始数据流并更新控制结构体
    void Data_Processing(HDL_Damiao_Motor_App_Measure_t *motor_data);

    void UART10_MT6701_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
    void UART10_MT6701_ErrorCallback(void);

    Class_Video_Transmission VT_Module;
    Class_PC_USB_Communication PC_USB_Module;

    tx_data_t tx_data;

    dr16_control_t dr16_control;
    custom_control_t custom_control;
    remote_control_t remote_control;
    vt_rc_control_t  vt_rc_control;

private:
    void Update_Custom_Control(uint8_t *custom_data);
    void Update_Remote_Control(uint8_t *remote_data);
    void Update_VT_RC_Control(uint8_t *vt_rc_data);

    // 处理PC USB原始数据流，解析出完整帧并进行命令分发
    void USB_PC_Data_Processing(uint8_t *data, uint16_t length);
    void USB_PC_Frame_Analysis(const USB_PC_Data_Frame_t *usb_pc_frame);
    void USB_PC_Stream_Buffer_Shift(uint16_t shift_length);

    // 处理串口10接收到的MT6701 A5数据流，解析出角度和角速度
    void UART10_MT6701_Data_Processing(uint8_t *data, uint16_t length);
    void UART10_MT6701_Frame_Analysis(const USB_PC_Data_Frame_t *mt6701_frame);
    void UART10_MT6701_Stream_Buffer_Shift(uint16_t shift_length);

    uint8_t pc_usb_stream_rx_buffer[USB_PC_STREAM_BUFFER_SIZE]; // PC USB原始数据流缓冲区
    uint16_t pc_usb_stream_rx_length;  // PC USB原始数据流当前长度

    uint8_t pc_usb_stream_tx_data[USB_PC_STREAM_BUFFER_SIZE]; // PC USB原始数据流缓冲区
    uint16_t pc_usb_stream_tx_length;  // PC USB原始数据流当前长度

    QueueHandle_t xmt6701_uart_raw_queue;
    uint8_t mt6701_uart_stream_rx_buffer[COMMU_HUB_MT6701_UART_STREAM_BUFFER_SIZE];
    uint16_t mt6701_uart_stream_rx_length;
    MT6701_UART_State_t mt6701_uart_state;
    float mt6701_uart_speed_history[3];
    uint8_t mt6701_uart_speed_history_count;
    uint8_t mt6701_uart_valid_flag;
};

/**
 * @brief 构造通信中心
 */
extern Class_Commu_Hub Commu_Hub;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   PC通信中心线程
 */
extern void _Thread_Commu_Hub_(void *pvParameters);

#ifdef __cplusplus
}
#endif