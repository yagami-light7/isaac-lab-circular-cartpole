/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_Commu_Hub.cpp
  * @brief      瑙ｆ瀽閫氫俊鏁版嵁锛屾帴鏀堕槦鍒楁暟鎹苟瀹屾垚鍗忚瑙ｆ瀽锛屽疄鐜颁笌ISR瑙ｈ€?  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     Mar-29-2026     Light           閲嶆瀯涓?Commu_Hub 骞舵帴绠?USB A5 瑙ｆ瀽
  *
  @verbatim
  ==============================================================================
  * 褰撳墠閫氫俊涓績鑱岃矗濡備笅锛?  * 1. 缁熶竴鎸佹湁鍥句紶妯″潡涓?PC USB 閫氳妯″潡瀹炰緥
  * 2. 鍦ㄧ嚎绋嬩腑瑙ｆ瀽 PC USB 鍘熷鏁版嵁闃熷垪涓殑 A5 鍗忚甯?  * 3. 鍦ㄧ嚎绋嬩腑瑙ｆ瀽涓插彛10鎺ユ敹鍒扮殑 MT6701 A5 鍗忚甯?  * 4. 涓哄簲鐢ㄥ眰鎻愪緵鍞竴鐨勯€氫俊鐘舵€佸叆鍙ｏ紝闄嶄綆澶栭儴妯″潡鑰﹀悎
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "APL_Commu_Hub.h"

#include <string.h>

#include "HDL_Damiao_Motor_App.h"
#include "MWL_Data_Utils.h"
#include "MWL_Toolbox.h"
#include "HAL_USART.h"
#include "App_Detect_Task.h"
#include "Board_CRC8_CRC16.h"

/**
 * @brief          涓夌偣涓€兼护娉紝浼樺厛鎶戝埗瑙掗€熷害涓殑鍋跺彂灏栧嘲
 * @param[in]      a, b, c 涓変釜杩炵画閲囨牱鍊? * @retval         涓€肩粨鏋? */
static float Commu_Hub_Median3(float a, float b, float c)
{
    float temp = 0.0f;

    if (a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }

    if (b > c)
    {
        temp = b;
        b = c;
        c = temp;
    }

    if (a > b)
    {
        b = a;
    }

    return b;
}

/**
 * @brief          灏嗚搴︾害鏉熷埌 [-pi, pi]锛屼究浜庡拰鍥哄畾鏉嗙姸鎬佺粺涓€琛ㄨ揪
 * @param[in]      angle_rad 鍘熷瑙掑害
 * @retval         褰掍竴鍖栧悗鐨勮搴? */
static float Commu_Hub_Wrap_Radian(float angle_rad)
{
    while (angle_rad > COMMU_HUB_PI_F)
    {
        angle_rad -= (2.0f * COMMU_HUB_PI_F);
    }

    while (angle_rad < -COMMU_HUB_PI_F)
    {
        angle_rad += (2.0f * COMMU_HUB_PI_F);
    }

    return angle_rad;
}

/**
 * @brief 鏋勯€犳暟鎹В鏋愬璞″疄渚? */
Class_Commu_Hub Commu_Hub;

/**
 * @brief   PC閫氫俊涓績绾跨▼
 */
void _Thread_Commu_Hub_(void *pvParameters)
{
    vTaskDelay(COMMU_HUB_INIT_TIME);

    Commu_Hub.Init();

    while (1)
    {
        // 鏁版嵁瑙ｆ瀽
        Commu_Hub.Data_Analysis();

        // 鏁版嵁鎷兼帴澶勭悊
        HDL_Damiao_Motor_App_Measure_t *motor_data = (HDL_Damiao_Motor_App_Measure_t *)HDL_Damiao_Motor_Get_Default_Measure_Point(0);
        Commu_Hub.Data_Processing(motor_data);

        // 鏁版嵁鍙戦€?        if (Commu_Hub.tx_data.pole_frame_length > 0U)
        {
            Commu_Hub.PC_USB_Module.PC_Data_Transmit(Commu_Hub.tx_data.pole_frame, Commu_Hub.tx_data.pole_frame_length);
        }

        vTaskDelay(COMMU_HUB_CONTROL_TIME);
    }
}

/**
 * @brief          鍒濆鍖栨暟鎹В鏋愮被
 * @param[in]      none
 * @retval         none
 */
void Class_Commu_Hub::Init(void)
{
    memset(&this->dr16_control, 0, sizeof(this->dr16_control));
    memset(&this->custom_control, 0, sizeof(this->custom_control));
    memset(&this->remote_control, 0, sizeof(this->remote_control));
    memset(&this->vt_rc_control, 0, sizeof(this->vt_rc_control));
    memset(&this->tx_data, 0, sizeof(this->tx_data));
    memset(this->pc_usb_stream_rx_buffer, 0, sizeof(this->pc_usb_stream_rx_buffer));
    memset(this->pc_usb_stream_tx_data, 0, sizeof(this->pc_usb_stream_tx_data));
    memset(this->mt6701_uart_stream_rx_buffer, 0, sizeof(this->mt6701_uart_stream_rx_buffer));
    memset(&this->mt6701_uart_state, 0, sizeof(this->mt6701_uart_state));
    memset(this->mt6701_uart_speed_history, 0, sizeof(this->mt6701_uart_speed_history));

    this->pc_usb_stream_rx_length = 0U;
    this->pc_usb_stream_tx_length = 0U;
    this->mt6701_uart_stream_rx_length = 0U;
    this->mt6701_uart_speed_history_count = 0U;
    this->mt6701_uart_valid_flag = 0U;

    this->VT_Module.Init(&UART7_Manage_Object);
    this->PC_USB_Module.Init();

    if (this->xmt6701_uart_raw_queue == NULL)
    {
        this->xmt6701_uart_raw_queue = xQueueCreate(COMMU_HUB_MT6701_UART_RX_QUEUE_ITEM_NUM, sizeof(MT6701_UART_Raw_Rx_Packet_t));
    }

    UART_Init(&UART10_Manage_Object, COMMU_HUB_MT6701_UART_RX_DATA_SIZE);
}

/**
 * @brief          瑙ｆ瀽鏈哄櫒浜轰氦浜掓暟鎹紝鏇存柊瀵瑰簲鎺у埗缁撴瀯浣? * @param[in]      none
 * @retval         none
 */
void Class_Commu_Hub::Data_Analysis()
{
    USB_PC_Raw_Rx_Packet_t usb_raw_packet = {};
    MT6701_UART_Raw_Rx_Packet_t mt6701_raw_packet = {};
    uint8_t custom_data[30] = {0U};
    uint8_t remote_data[12] = {0U};
    uint8_t vt_rc_data[21] = {0U};

    while (pdPASS == xQueueReceive(this->PC_USB_Module.pc_comm.xraw_queue, &usb_raw_packet, 0))
    {
        this->USB_PC_Data_Processing(usb_raw_packet.rx_data, usb_raw_packet.rx_data_size);
    }

    if (this->xmt6701_uart_raw_queue != NULL)
    {
        while (pdPASS == xQueueReceive(this->xmt6701_uart_raw_queue, &mt6701_raw_packet, 0))
        {
            this->UART10_MT6701_Data_Processing(mt6701_raw_packet.rx_data, mt6701_raw_packet.rx_data_size);
        }
    }

    if (pdPASS == xQueueReceive(this->VT_Module.custom_robot.xcustom_queue, custom_data, 0))
    {
        this->Update_Custom_Control(custom_data);
    }

    if (pdPASS == xQueueReceive(this->VT_Module.remote_robot.xremote_queue, remote_data, 0))
    {
        this->Update_Remote_Control(remote_data);
    }

    if (pdPASS == xQueueReceive(this->VT_Module.vt_rc_robot.xrc_vt_queue, vt_rc_data, 0))
    {
        this->Update_VT_RC_Control(vt_rc_data);
    }
}

/**
 * @brief          灏嗗彂閫佺粰PC鐨勬暟鎹繘琛屽姞宸ュ鐞嗭紝娣诲姞鍗忚甯уご銆佸懡浠ょ爜銆丆RC鏍￠獙绛? * @param[in]      motor_data 鐢垫満鍙嶉鏁版嵁
 * @retval         none
 */
void Class_Commu_Hub::Data_Processing(HDL_Damiao_Motor_App_Measure_t *motor_data)
{
    static uint8_t seq = 0;
    uint16_t cmd_id = 0x0100U;
    float pole_data[5] = {0.0f};
    uint16_t payload_length = (uint16_t)sizeof(pole_data);

    if (motor_data == NULL)
    {
        this->tx_data.pole_frame_length = 0U;
        return;
    }

    // 褰撳墠缁熶竴涓婃姤 5 涓?float锛?    // 1. 鍥哄畾鏉嗚搴?    // 2. 鍥哄畾鏉嗚閫熷害
    // 3. 鐢垫満鍔涚煩
    // 4. 娲诲姩鏉嗚搴︼紙宸插噺鍘诲畨瑁呴浂鐐瑰苟褰掍竴鍖栧埌 [-pi, pi]锛?    // 5. 娲诲姩鏉嗚閫熷害锛堝湪閫氫俊绾跨▼涓仛鎸囨暟骞冲潎婊ゆ尝锛?    pole_data[0] = motor_data->pos;
    pole_data[1] = motor_data->vel;
    pole_data[2] = motor_data->tor;
    pole_data[3] = (this->mt6701_uart_valid_flag != 0U) ? this->mt6701_uart_state.angle_rad : 0.0f;
    pole_data[4] = (this->mt6701_uart_valid_flag != 0U) ? this->mt6701_uart_state.speed_rad_s : 0.0f;

    memset(this->tx_data.pole_frame, 0, sizeof(this->tx_data.pole_frame));

    // 鍙戦€佺粰 PC 鐨勭湡瀹炵嚎鍗忚甯ч暱搴﹀浐瀹氫负 29 瀛楄妭锛?    // 5 瀛楄妭甯уご + 2 瀛楄妭鍛戒护瀛?+ 20 瀛楄妭杞借嵎 + 2 瀛楄妭 CRC16
    this->tx_data.pole_frame_length = (uint16_t)(USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + payload_length + USB_PC_FRAME_TAIL_LENGTH);

    // 杩欓噷涓嶈兘鍐嶆妸鍘熷瀛楄妭缂撳啿鍖哄己杞垚甯уご缁撴瀯浣撱€?    // 褰撳墠缂栬瘧鍣ㄤ笅 __packed 琚拷鐣ワ紝缁撴瀯浣撲細琚嚜鍔ㄥ榻愶紝瀵艰嚧甯уご瀛楁鍋忕Щ閿欒銆?    this->tx_data.pole_frame[0] = USB_PC_SOF;
    this->tx_data.pole_frame[1] = (uint8_t)(payload_length & 0x00FFU);
    this->tx_data.pole_frame[2] = (uint8_t)((payload_length >> 8U) & 0x00FFU);
    this->tx_data.pole_frame[3] = seq++;
    this->tx_data.pole_frame[4] = 0U;
    append_CRC8_check_sum(this->tx_data.pole_frame, USB_PC_FRAME_HEADER_LENGTH);

    memcpy(&this->tx_data.pole_frame[USB_PC_FRAME_HEADER_LENGTH], &cmd_id, sizeof(cmd_id));
    memcpy(&this->tx_data.pole_frame[USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH], pole_data, payload_length);

    append_CRC16_check_sum(this->tx_data.pole_frame, this->tx_data.pole_frame_length);
}

/**
 * @brief          鏇存柊鑷畾涔夋帶鍒跺櫒鏁版嵁
 * @param[in]      custom_data 鑷畾涔夋帶鍒跺櫒鍘熷杞借嵎
 * @retval         none
 */
void Class_Commu_Hub::Update_Custom_Control(uint8_t *custom_data)
{
    if (custom_data == NULL)
    {
        return;
    }

    uint8_to_float(custom_data, this->custom_control.custom_angle_set, JOINTS_NUM);
    detect_hook(VT_TOE);
    detect_hook(CUSTOM_TOE);
}

/**
 * @brief          鏇存柊0x0304閿紶鎺у埗鏁版嵁
 * @param[in]      remote_data 閿紶鍘熷杞借嵎
 * @retval         none
 */
void Class_Commu_Hub::Update_Remote_Control(uint8_t *remote_data)
{
    if (remote_data == NULL)
    {
        return;
    }

    this->remote_control.last_left_button_down = this->remote_control.left_button_down;
    this->remote_control.last_right_button_down = this->remote_control.right_button_down;
    this->remote_control.last_keyboard_value = this->remote_control.keyboard_value;

    memcpy(&this->remote_control.mouse_x, &remote_data[0], sizeof(this->remote_control.mouse_x));
    memcpy(&this->remote_control.mouse_y, &remote_data[2], sizeof(this->remote_control.mouse_y));
    memcpy(&this->remote_control.mouse_z, &remote_data[4], sizeof(this->remote_control.mouse_z));
    memcpy(&this->remote_control.left_button_down, &remote_data[6], sizeof(this->remote_control.left_button_down));
    memcpy(&this->remote_control.right_button_down, &remote_data[7], sizeof(this->remote_control.right_button_down));
    memcpy(&this->remote_control.keyboard_value, &remote_data[8], sizeof(this->remote_control.keyboard_value));

    detect_hook(VT_TOE);
}

/**
 * @brief          鏇存柊鏂板浘浼犻敭榧犱笌鎽囨潌鏁版嵁
 * @param[in]      vt_rc_data 鍥句紶鍘熷杞借嵎
 * @retval         none
 */
void Class_Commu_Hub::Update_VT_RC_Control(uint8_t *vt_rc_data)
{
    uint64_t bitfield = 0U;
    int16_t tmp_mouse_x = 0;
    int16_t tmp_mouse_y = 0;
    int16_t tmp_mouse_z = 0;
    uint8_t mouse_byte = 0U;
    uint8_t i = 0U;

    if (vt_rc_data == NULL)
    {
        return;
    }

    this->vt_rc_control.sof_1 = vt_rc_data[0];
    this->vt_rc_control.sof_2 = vt_rc_data[1];

    for (i = 0U; i < 8U; i++)
    {
        bitfield |= ((uint64_t)vt_rc_data[i + 2U]) << (8U * i);
    }

    this->vt_rc_control.ch_0 = (int16_t)((bitfield >> 0U) & 0x7FFU);
    this->vt_rc_control.ch_1 = (int16_t)((bitfield >> 11U) & 0x7FFU);
    this->vt_rc_control.ch_2 = (int16_t)((bitfield >> 22U) & 0x7FFU);
    this->vt_rc_control.ch_3 = (int16_t)((bitfield >> 33U) & 0x7FFU);
    this->vt_rc_control.mode_sw = (uint8_t)((bitfield >> 44U) & 0x03U);
    this->vt_rc_control.pause = (uint8_t)((bitfield >> 46U) & 0x01U);
    this->vt_rc_control.fn_1 = (uint8_t)((bitfield >> 47U) & 0x01U);
    this->vt_rc_control.fn_2 = (uint8_t)((bitfield >> 48U) & 0x01U);
    this->vt_rc_control.wheel = (int16_t)((bitfield >> 49U) & 0x7FFU);
    this->vt_rc_control.trigger = (uint8_t)((bitfield >> 60U) & 0x01U);

    tmp_mouse_x = (int16_t)(vt_rc_data[10] | (vt_rc_data[11] << 8));
    tmp_mouse_y = (int16_t)(vt_rc_data[12] | (vt_rc_data[13] << 8));
    tmp_mouse_z = (int16_t)(vt_rc_data[14] | (vt_rc_data[15] << 8));

    this->vt_rc_control.mouse_x = (float)tmp_mouse_x;
    this->vt_rc_control.mouse_y = (float)tmp_mouse_y;
    this->vt_rc_control.mouse_z = (float)tmp_mouse_z;

    mouse_byte = vt_rc_data[16];
    this->vt_rc_control.mouse_left = (uint8_t)((mouse_byte >> 0U) & 0x03U);
    this->vt_rc_control.mouse_right = (uint8_t)((mouse_byte >> 2U) & 0x03U);
    this->vt_rc_control.mouse_middle = (uint8_t)((mouse_byte >> 4U) & 0x03U);

    this->vt_rc_control.key = (uint16_t)(vt_rc_data[17] | (vt_rc_data[18] << 8));
    this->vt_rc_control.crc16 = (uint16_t)(vt_rc_data[19] | (vt_rc_data[20] << 8));

    detect_hook(VT_TOE);

    this->vt_rc_control.ch_0 -= RC_CH_VALUE_OFFSET;
    this->vt_rc_control.ch_1 -= RC_CH_VALUE_OFFSET;
    this->vt_rc_control.ch_2 -= RC_CH_VALUE_OFFSET;
    this->vt_rc_control.ch_3 -= RC_CH_VALUE_OFFSET;
    this->vt_rc_control.wheel -= RC_CH_VALUE_OFFSET;
}

/**
 * @brief          涓插彛10涓嶅畾闀挎帴鏀跺洖璋冿紝灏嗗師濮嬫暟鎹斁鍏ラ槦鍒? * @param[in]      huart 瀵瑰簲涓插彛鍙ユ焺
 * @param[in]      Size  褰撳墠鎺ユ敹鍒扮殑鏁版嵁闀垮害
 * @retval         none
 */
void Class_Commu_Hub::UART10_MT6701_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    MT6701_UART_Raw_Rx_Packet_t raw_packet = {};
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((huart == NULL) || (huart->Instance != USART10))
    {
        return;
    }

    if (UART10_Manage_Object.rx_data_size == 0U)
    {
        return;
    }

    if (Size > COMMU_HUB_MT6701_UART_RX_DATA_SIZE)
    {
        Size = COMMU_HUB_MT6701_UART_RX_DATA_SIZE;
    }

    raw_packet.rx_data_size = Size;
    if (Size > 0U)
    {
        memcpy(raw_packet.rx_data, UART10_Manage_Object.rx_buffer, Size);
    }

    if (this->xmt6701_uart_raw_queue != NULL)
    {
        xQueueSendFromISR(this->xmt6701_uart_raw_queue, &raw_packet, &xHigherPriorityTaskWoken);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.rx_buffer, UART10_Manage_Object.rx_data_size);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief          涓插彛10閿欒鍥炶皟锛岄噸鏂版媺璧风┖闂蹭腑鏂璂MA鎺ユ敹
 * @param[in]      none
 * @retval         none
 */
void Class_Commu_Hub::UART10_MT6701_ErrorCallback(void)
{
    if (UART10_Manage_Object.huart == NULL)
    {
        return;
    }

    __HAL_UNLOCK(UART10_Manage_Object.huart);

    if (UART10_Manage_Object.rx_data_size > 0U)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(UART10_Manage_Object.huart, UART10_Manage_Object.rx_buffer, UART10_Manage_Object.rx_data_size);
    }
}

/**
 * @brief          鍦ˋPL灞傚鐞哖C USB鍘熷鏁版嵁娴? * @param[in]      data    鍘熷鏁版嵁棣栧湴鍧€
 *                 length  鍘熷鏁版嵁闀垮害
 * @retval         none
 */
void Class_Commu_Hub::USB_PC_Data_Processing(uint8_t *data, uint16_t length)
{
    USB_PC_Data_Frame_t usb_pc_frame = {};
    uint16_t data_length = 0U;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    // 濡傛灉鏂版暟鎹暱搴﹁秴杩囩紦鍐插尯鍓╀綑绌洪棿锛屼涪寮冩暣涓紦鍐插尯鍐呭浠ラ伩鍏嶆孩鍑?    if ((this->pc_usb_stream_rx_length + length) > USB_PC_STREAM_BUFFER_SIZE)
    {
        this->pc_usb_stream_rx_length = 0U;
        memset(this->pc_usb_stream_rx_buffer, 0, sizeof(this->pc_usb_stream_rx_buffer));
    }

    // 灏嗘柊鏁版嵁杩藉姞鍒版祦缂撳啿鍖烘湯灏?    memcpy(&this->pc_usb_stream_rx_buffer[this->pc_usb_stream_rx_length], data, length);
    this->pc_usb_stream_rx_length = (uint16_t)(this->pc_usb_stream_rx_length + length);

    // 瑙ｆ瀽娴佺紦鍐插尯涓殑瀹屾暣甯э紝鐩村埌鍓╀綑鏁版嵁涓嶈冻浠ユ瀯鎴愭渶灏忓抚涓烘
    while (this->pc_usb_stream_rx_length >= (USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + USB_PC_FRAME_TAIL_LENGTH))
    {
        uint16_t full_frame_length = 0U;

        if (this->pc_usb_stream_rx_buffer[0] != USB_PC_SOF)
        {
            this->USB_PC_Stream_Buffer_Shift(1U);
            continue;
        }

        if (verify_CRC8_check_sum(this->pc_usb_stream_rx_buffer, USB_PC_FRAME_HEADER_LENGTH) == 0U)
        {
            this->USB_PC_Stream_Buffer_Shift(1U);
            continue;
        }

        data_length = (uint16_t)(this->pc_usb_stream_rx_buffer[1] | ((uint16_t)this->pc_usb_stream_rx_buffer[2] << 8U));

        if (data_length > USB_PC_MAX_DATA_LENGTH)
        {
            this->USB_PC_Stream_Buffer_Shift(1U);
            continue;
        }

        full_frame_length = (uint16_t)(USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + data_length + USB_PC_FRAME_TAIL_LENGTH);
        if (this->pc_usb_stream_rx_length < full_frame_length)
        {
            break;
        }

        if (verify_CRC16_check_sum(this->pc_usb_stream_rx_buffer, full_frame_length) == 0U)
        {
            this->USB_PC_Stream_Buffer_Shift(1U);
            continue;
        }

        usb_pc_frame.frame_header.SOF = this->pc_usb_stream_rx_buffer[0];
        usb_pc_frame.frame_header.data_length = data_length;
        usb_pc_frame.frame_header.seq = this->pc_usb_stream_rx_buffer[3];
        usb_pc_frame.frame_header.CRC8 = this->pc_usb_stream_rx_buffer[4];
        memcpy(&usb_pc_frame.cmd_id, &this->pc_usb_stream_rx_buffer[USB_PC_FRAME_HEADER_LENGTH], sizeof(usb_pc_frame.cmd_id));
        memset(usb_pc_frame.data, 0, sizeof(usb_pc_frame.data));
        memcpy(usb_pc_frame.data, &this->pc_usb_stream_rx_buffer[USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH], data_length);

        this->USB_PC_Frame_Analysis(&usb_pc_frame);
        this->USB_PC_Stream_Buffer_Shift(full_frame_length);
    }
}

/**
 * @brief          瑙ｆ瀽涓插彛10鎺ユ敹鍒扮殑 MT6701 鍘熷鏁版嵁娴? * @param[in]      data    鍘熷鏁版嵁棣栧湴鍧€
 *                 length  鍘熷鏁版嵁闀垮害
 * @retval         none
 */
void Class_Commu_Hub::UART10_MT6701_Data_Processing(uint8_t *data, uint16_t length)
{
    USB_PC_Data_Frame_t mt6701_frame = {};
    uint16_t data_length = 0U;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    if ((this->mt6701_uart_stream_rx_length + length) > COMMU_HUB_MT6701_UART_STREAM_BUFFER_SIZE)
    {
        this->mt6701_uart_stream_rx_length = 0U;
        memset(this->mt6701_uart_stream_rx_buffer, 0, sizeof(this->mt6701_uart_stream_rx_buffer));
    }

    memcpy(&this->mt6701_uart_stream_rx_buffer[this->mt6701_uart_stream_rx_length], data, length);
    this->mt6701_uart_stream_rx_length = (uint16_t)(this->mt6701_uart_stream_rx_length + length);

    while (this->mt6701_uart_stream_rx_length >= (USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + USB_PC_FRAME_TAIL_LENGTH))
    {
        uint16_t full_frame_length = 0U;

        if (this->mt6701_uart_stream_rx_buffer[0] != USB_PC_SOF)
        {
            this->UART10_MT6701_Stream_Buffer_Shift(1U);
            continue;
        }

        if (verify_CRC8_check_sum(this->mt6701_uart_stream_rx_buffer, USB_PC_FRAME_HEADER_LENGTH) == 0U)
        {
            this->UART10_MT6701_Stream_Buffer_Shift(1U);
            continue;
        }

        data_length = (uint16_t)(this->mt6701_uart_stream_rx_buffer[1] | ((uint16_t)this->mt6701_uart_stream_rx_buffer[2] << 8U));
        if (data_length > USB_PC_MAX_DATA_LENGTH)
        {
            this->UART10_MT6701_Stream_Buffer_Shift(1U);
            continue;
        }

        full_frame_length = (uint16_t)(USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH + data_length + USB_PC_FRAME_TAIL_LENGTH);
        if (this->mt6701_uart_stream_rx_length < full_frame_length)
        {
            break;
        }

        if (verify_CRC16_check_sum(this->mt6701_uart_stream_rx_buffer, full_frame_length) == 0U)
        {
            this->UART10_MT6701_Stream_Buffer_Shift(1U);
            continue;
        }

        mt6701_frame.frame_header.SOF = this->mt6701_uart_stream_rx_buffer[0];
        mt6701_frame.frame_header.data_length = data_length;
        mt6701_frame.frame_header.seq = this->mt6701_uart_stream_rx_buffer[3];
        mt6701_frame.frame_header.CRC8 = this->mt6701_uart_stream_rx_buffer[4];
        memcpy(&mt6701_frame.cmd_id, &this->mt6701_uart_stream_rx_buffer[USB_PC_FRAME_HEADER_LENGTH], sizeof(mt6701_frame.cmd_id));
        memset(mt6701_frame.data, 0, sizeof(mt6701_frame.data));
        memcpy(mt6701_frame.data, &this->mt6701_uart_stream_rx_buffer[USB_PC_FRAME_HEADER_LENGTH + USB_PC_CMD_ID_LENGTH], data_length);

        this->UART10_MT6701_Frame_Analysis(&mt6701_frame);
        this->UART10_MT6701_Stream_Buffer_Shift(full_frame_length);
    }
}

/**
 * @brief          瑙ｆ瀽鍗曞抚PC USB鍗忚鏁版嵁
 * @param[in]      usb_pc_frame 宸插畬鎴?CRC 鏍￠獙鐨勬暟鎹抚
 * @retval         none
 */
void Class_Commu_Hub::USB_PC_Frame_Analysis(const USB_PC_Data_Frame_t *usb_pc_frame)
{
    USB_PC_Motor_Enable_Command_t motor_enable_command = {};
    USB_PC_MIT_Command_t mit_command = {};
    USB_PC_Position_Delta_Command_t position_delta_command = {};
    const HDL_Damiao_Motor_App_Measure_t *motor_measure = NULL;
    float target_pos_rad = 0.0f;

    if (usb_pc_frame == NULL)
    {
        return;
    }

    switch (usb_pc_frame->cmd_id)
    {
        case USB_PC_CMD_ID_MOTOR_ENABLE:
            if (usb_pc_frame->frame_header.data_length == sizeof(USB_PC_Motor_Enable_Command_t))
            {
                memcpy(&motor_enable_command, usb_pc_frame->data, sizeof(motor_enable_command));

                if (motor_enable_command.enable_flag != 0U)
                {
                    HDL_Damiao_Motor_Enable_Default(motor_enable_command.motor_id);
                }
                else
                {
                    HDL_Damiao_Motor_Disable_Default(motor_enable_command.motor_id);
                }
            }
            break;

        case USB_PC_CMD_ID_HEARTBEAT_REQ:
        case USB_PC_CMD_ID_MIT_CONTROL:
            if (usb_pc_frame->frame_header.data_length == sizeof(USB_PC_MIT_Command_t))
            {
                memcpy(&mit_command, usb_pc_frame->data, sizeof(mit_command));

                // 褰撳墠鏉跨榛樿浣跨敤鏈湴鍥哄畾 Kp/Kd銆?                // PC 渚у彧闇€瑕佺ǔ瀹氱粰鍑?pos / vel / tor 鍗冲彲瀹屾垚 MIT 璋冭瘯銆?                mit_command.pos_rad = MWL_Clamp(mit_command.pos_rad, -3.2f, 3.2f);
                mit_command.vel_rad_s = MWL_Clamp(mit_command.vel_rad_s, -5.0f, 5.0f);
                mit_command.torque_nm = MWL_Clamp(mit_command.torque_nm, -2.5f, 2.5f);

                HDL_Damiao_Motor_Send_Default_MIT_Command(
                    mit_command.motor_id,
                    mit_command.pos_rad,
                    mit_command.vel_rad_s,
                    COMMU_HUB_MIT_FIXED_KP,
                    COMMU_HUB_MIT_FIXED_KD,
                    mit_command.torque_nm);
            }
            break;

        case USB_PC_CMD_ID_POSITION_DELTA_CONTROL:
            if (usb_pc_frame->frame_header.data_length == sizeof(USB_PC_Position_Delta_Command_t))
            {
                memcpy(&position_delta_command, usb_pc_frame->data, sizeof(position_delta_command));

                if (position_delta_command.motor_id == 0U)
                {
                    break;
                }

                motor_measure = HDL_Damiao_Motor_Get_Default_Measure_Point((uint8_t)(position_delta_command.motor_id - 1U));
                if (motor_measure == NULL)
                {
                    break;
                }

                // 浣嶇疆澧為噺鎺у埗鍙繚鐣欎竴涓牳蹇冮噺锛歱os_delta_rad銆?                // 鏉跨鍥哄畾浣跨敤鏈湴 Kp/Kd锛屾妸绛栫暐杈撳嚭鏄庣‘绾︽潫涓哄崟缁翠綅缃閲忋€?                target_pos_rad = motor_measure->pos + position_delta_command.pos_delta_rad;
                target_pos_rad = MWL_Clamp(target_pos_rad, -3.2f, 3.2f);

                HDL_Damiao_Motor_Send_Default_MIT_Command(
                    position_delta_command.motor_id,
                    target_pos_rad,
                    0.0f,
                    COMMU_HUB_MIT_FIXED_KP,
                    COMMU_HUB_MIT_FIXED_KD,
                    0.0f);
            }
            break;

        default:
            break;
    }
}

/**
 * @brief          瑙ｆ瀽鍗曞抚涓插彛10 MT6701 鍗忚鏁版嵁
 * @param[in]      mt6701_frame 宸插畬鎴?CRC 鏍￠獙鐨勬暟鎹抚
 * @retval         none
 */
void Class_Commu_Hub::UART10_MT6701_Frame_Analysis(const USB_PC_Data_Frame_t *mt6701_frame)
{
    MT6701_UART_State_t mt6701_state = {};

    if (mt6701_frame == NULL)
    {
        return;
    }

    if ((mt6701_frame->cmd_id == USB_PC_CMD_ID_MT6701_STATE) &&
        (mt6701_frame->frame_header.data_length == COMMU_HUB_MT6701_UART_PAYLOAD_LENGTH))
    {
        float speed_filtered_input = 0.0f;

        memcpy(&mt6701_state, mt6701_frame->data, sizeof(mt6701_state));

        // C鏉垮洖浼犵殑鍘熷瑙掑害鑼冨洿涓?[0, 2pi]銆?        // 杩欓噷灏嗏€滄椿鍔ㄦ潌涓庡浐瀹氭潌閲嶅悎浣嶇疆鈥濆搴旂殑瀹夎闆剁偣 2.4785 rad 鏄犲皠涓?0锛?        // 鍐嶅皢瑙掑害绾︽潫鍒?[-pi, pi]锛屼究浜庡悗缁笌鍥哄畾鏉嗙姸鎬佺粺涓€澶勭悊銆?        mt6701_state.angle_rad = Commu_Hub_Wrap_Radian(mt6701_state.angle_rad - COMMU_HUB_MT6701_ZERO_OFFSET_RAD);

        // 瀵硅閫熷害鍏堝仛 3 鐐逛腑鍊兼护娉紝鎶戝埗涓插彛閾捐矾涓殑鍋跺彂灏栧嘲锛?        // 鍐嶅仛涓€闃舵寚鏁板钩鍧囷紝鍏奸【鍝嶅簲閫熷害涓庣ǔ瀹氭€с€?        this->mt6701_uart_speed_history[0] = this->mt6701_uart_speed_history[1];
        this->mt6701_uart_speed_history[1] = this->mt6701_uart_speed_history[2];
        this->mt6701_uart_speed_history[2] = mt6701_state.speed_rad_s;

        if (this->mt6701_uart_speed_history_count < 3U)
        {
            this->mt6701_uart_speed_history_count++;
        }

        if (this->mt6701_uart_speed_history_count >= 3U)
        {
            speed_filtered_input = Commu_Hub_Median3(this->mt6701_uart_speed_history[0],
                                                     this->mt6701_uart_speed_history[1],
                                                     this->mt6701_uart_speed_history[2]);
        }
        else
        {
            speed_filtered_input = mt6701_state.speed_rad_s;
        }

        if (this->mt6701_uart_valid_flag == 0U)
        {
            // 璐熷彿锛氱‘淇濋€嗘椂閽堟棆杞负姝ｈ搴︼紝椤烘椂閽堟棆杞负璐熻搴︼紝涓庡浐瀹氭潌鐘舵€佽〃杈句竴鑷淬€?            this->mt6701_uart_state.angle_rad = -mt6701_state.angle_rad;
            this->mt6701_uart_state.speed_rad_s = -speed_filtered_input;
            this->mt6701_uart_valid_flag = 1U;
        }
        else
        {
            this->mt6701_uart_state.angle_rad = mt6701_state.angle_rad;
            this->mt6701_uart_state.speed_rad_s += COMMU_HUB_MT6701_SPEED_FILTER_ALPHA * (speed_filtered_input - this->mt6701_uart_state.speed_rad_s);
        }
    }
}

/**
 * @brief          骞崇ЩUSB娴佺紦鍐插尯锛屼涪寮冨凡澶勭悊鎴栨棤鏁堢殑鍓嶅瀛楄妭
 * @param[in]      shift_length 闇€瑕佷涪寮冪殑瀛楄妭鏁? * @retval         none
 */
void Class_Commu_Hub::USB_PC_Stream_Buffer_Shift(uint16_t shift_length)
{
    if (shift_length >= this->pc_usb_stream_rx_length)
    {
        this->pc_usb_stream_rx_length = 0U;
        return;
    }

    memmove(this->pc_usb_stream_rx_buffer,
            &this->pc_usb_stream_rx_buffer[shift_length],
            this->pc_usb_stream_rx_length - shift_length);
    this->pc_usb_stream_rx_length = (uint16_t)(this->pc_usb_stream_rx_length - shift_length);
}

/**
 * @brief          骞崇Щ涓插彛10娴佺紦鍐插尯锛屼涪寮冨凡澶勭悊鎴栨棤鏁堢殑鍓嶅瀛楄妭
 * @param[in]      shift_length 闇€瑕佷涪寮冪殑瀛楄妭鏁? * @retval         none
 */
void Class_Commu_Hub::UART10_MT6701_Stream_Buffer_Shift(uint16_t shift_length)
{
    if (shift_length >= this->mt6701_uart_stream_rx_length)
    {
        this->mt6701_uart_stream_rx_length = 0U;
        return;
    }

    memmove(this->mt6701_uart_stream_rx_buffer,
            &this->mt6701_uart_stream_rx_buffer[shift_length],
            this->mt6701_uart_stream_rx_length - shift_length);
    this->mt6701_uart_stream_rx_length = (uint16_t)(this->mt6701_uart_stream_rx_length - shift_length);
}
