/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_USB_CDC.cpp
  * @brief      USB CDC外设再封装
  * @note       Hardware Abstract Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2026     Codex           1. 基于当前CubeMX生成的USB OTG驱动重建
  *
  @verbatim
  ==============================================================================
  * 本文件不再直接使用例程拷贝来的 USB_DEVICE/App/Target 胶水代码，
  * 而是基于当前工程中 CubeMX 生成的 usb_otg.c / usb_otg.h 自行完成：
  * 1. USB CDC 设备层初始化
  * 2. Device Descriptor 与 CDC Interface 注册
  * 3. USB Device Core 与 HAL PCD 的桥接
  * 4. 在 OUT 回调中将原始数据包通过项目回调钩子转交给 APL 层
  *
  * 这样做的目的是让 USB 通信部分尽量贴合当前工程原有分层和代码风格，
  * 同时避免继续引入额外的例程胶水文件。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HAL_USB_CDC.h"

#include <string.h>

#include "usb_otg.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_ctlreq.h"

#define USB_CDC_VID                        0x0483U    // USB厂商ID
#define USB_CDC_PID                        0x5740U    // USB产品ID
#define USB_CDC_LANGID_STRING              1033U      // 英语（美国）
#define USB_CDC_DEVICE_RELEASE             0x0200U    // 设备版本号2.00

#define USB_CDC_MANUFACTURER_STRING        "Robot_Z"
#define USB_CDC_PRODUCT_STRING             "MC02 USB CDC"
#define USB_CDC_CONFIGURATION_STRING       "USB CDC Config"
#define USB_CDC_INTERFACE_STRING           "USB CDC Interface"

#define USB_CDC_RX_FIFO_WORD_SIZE          0x200U
#define USB_CDC_TX_EP0_FIFO_WORD_SIZE      0x80U
#define USB_CDC_TX_EP1_FIFO_WORD_SIZE      0x174U
#define USB_CDC_TX_BUFFER_SIZE             256U
#define USB_CDC_STRING_SERIAL_SIZE         26U

/**
 * @brief USB设备核心实例
 */
static USBD_HandleTypeDef USB_CDC_Device = {};

/**
 * @brief 当前生效的USB CDC管理对象
 */
static USB_CDC_Manage_Object_t *USB_CDC_Active_Manage_Object = NULL;

/**
 * @brief USB CDC类收发缓冲区
 */
static uint8_t USB_CDC_Class_Rx_Buffer[USB_CDC_RX_SINGLE_PACKET_SIZE] = {0U};
static uint8_t USB_CDC_Class_Tx_Buffer[USB_CDC_TX_BUFFER_SIZE] = {0U};

/**
 * @brief USB字符串描述符缓冲区
 */
__ALIGN_BEGIN static uint8_t USB_CDC_String_Descriptor[USBD_MAX_STR_DESC_SIZ] __ALIGN_END = {0U};
__ALIGN_BEGIN static uint8_t USB_CDC_Serial_String[USB_CDC_STRING_SERIAL_SIZE] __ALIGN_END =
{
    USB_CDC_STRING_SERIAL_SIZE,
    USB_DESC_TYPE_STRING,
};

/**
 * @brief USB设备描述符
 */
__ALIGN_BEGIN static uint8_t USB_CDC_Device_Descriptor_Buffer[USB_LEN_DEV_DESC] __ALIGN_END =
{
    0x12U,
    USB_DESC_TYPE_DEVICE,
    0x00U,
    0x02U,
    0x02U,
    0x02U,
    0x00U,
    USB_MAX_EP0_SIZE,
    LOBYTE(USB_CDC_VID),
    HIBYTE(USB_CDC_VID),
    LOBYTE(USB_CDC_PID),
    HIBYTE(USB_CDC_PID),
    LOBYTE(USB_CDC_DEVICE_RELEASE),
    HIBYTE(USB_CDC_DEVICE_RELEASE),
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION,
};

/**
 * @brief USB语言描述符
 */
__ALIGN_BEGIN static uint8_t USB_CDC_LangID_Descriptor[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USB_CDC_LANGID_STRING),
    HIBYTE(USB_CDC_LANGID_STRING),
};

/**
 * @brief USB描述符接口声明
 */
static uint8_t *USB_CDC_Device_Descriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USB_CDC_LangID_Descriptor_Get(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USB_CDC_Manufacturer_String_Get(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USB_CDC_Product_String_Get(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USB_CDC_Serial_String_Get(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USB_CDC_Config_String_Get(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USB_CDC_Interface_String_Get(USBD_SpeedTypeDef speed, uint16_t *length);

/**
 * @brief CDC接口函数声明
 */
static int8_t USB_CDC_Class_Init(void);
static int8_t USB_CDC_Class_DeInit(void);
static int8_t USB_CDC_Class_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t USB_CDC_Class_Receive_Callback(uint8_t *Buf, uint32_t *Len);
static int8_t USB_CDC_Class_Transmit_Complete_Callback(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

/**
 * @brief 工具函数声明
 */
static void USB_CDC_Get_Serial_Number(void);
static void USB_CDC_Int_To_Unicode(uint32_t value, uint8_t *buffer, uint8_t length);
static USBD_StatusTypeDef USB_CDC_Get_USB_Status(HAL_StatusTypeDef hal_status);

/**
 * @brief USB描述符对象
 */
static USBD_DescriptorsTypeDef USB_CDC_Descriptors =
{
    USB_CDC_Device_Descriptor,
    USB_CDC_LangID_Descriptor_Get,
    USB_CDC_Manufacturer_String_Get,
    USB_CDC_Product_String_Get,
    USB_CDC_Serial_String_Get,
    USB_CDC_Config_String_Get,
    USB_CDC_Interface_String_Get,
};

/**
 * @brief CDC接口对象
 */
static USBD_CDC_ItfTypeDef USB_CDC_Interface =
{
    USB_CDC_Class_Init,
    USB_CDC_Class_DeInit,
    USB_CDC_Class_Control,
    USB_CDC_Class_Receive_Callback,
    USB_CDC_Class_Transmit_Complete_Callback,
};

extern "C"
{

/**
 * @brief          USB CDC初始化
 * @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
 * @retval         none
 */
void USB_CDC_Init(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object)
{
    if (USB_CDC_Manage_Object == NULL)
    {
        return;
    }

    if (USB_CDC_Manage_Object->init_flag == 1U)
    {
        return;
    }

    memset(USB_CDC_Manage_Object, 0, sizeof(*USB_CDC_Manage_Object));

    // 底层PCD已由CubeMX生成的main.c -> MX_USB_OTG_HS_PCD_Init()完成初始化
    // 这里仅负责初始化USB设备核心、注册CDC类并启动设备
    if (USBD_Init(&USB_CDC_Device, &USB_CDC_Descriptors, DEVICE_HS) != USBD_OK)
    {
        Error_Handler();
    }

    if (USBD_RegisterClass(&USB_CDC_Device, &USBD_CDC) != USBD_OK)
    {
        Error_Handler();
    }

    if (USBD_CDC_RegisterInterface(&USB_CDC_Device, &USB_CDC_Interface) != USBD_OK)
    {
        Error_Handler();
    }

    if (USBD_Start(&USB_CDC_Device) != USBD_OK)
    {
        Error_Handler();
    }

    USB_CDC_Active_Manage_Object = USB_CDC_Manage_Object;
    USB_CDC_Manage_Object->init_flag = 1U;
}

/**
 * @brief          USB CDC接收回调钩子弱定义
 * @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
 *                 data                    原始接收数据首地址
 *                 length                  原始接收数据长度
 *                 xHigherPriorityTaskWoken 任务切换标志
 * @retval         none
 */
__weak void USB_CDC_RxCallback(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object,
                               uint8_t *data,
                               uint16_t length,
                               BaseType_t *xHigherPriorityTaskWoken)
{
    UNUSED(USB_CDC_Manage_Object);
    UNUSED(data);
    UNUSED(length);
    UNUSED(xHigherPriorityTaskWoken);
}

/**
 * @brief          USB CDC发送数据
 * @param[in]      USB_CDC_Manage_Object   USB CDC实例对象
 *                 data                    数据首地址
 *                 length                  数据长度
 * @retval         发送结果
 */
bool USB_CDC_Send_Data(USB_CDC_Manage_Object_t *USB_CDC_Manage_Object, uint8_t *data, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc_handle = NULL;

    if (USB_CDC_Manage_Object == NULL || data == NULL || length == 0U)
    {
        return false;
    }

    if ((USB_CDC_Manage_Object->init_flag == 0U) || (USB_CDC_Active_Manage_Object != USB_CDC_Manage_Object))
    {
        return false;
    }

    if (USB_CDC_Device.dev_state != USBD_STATE_CONFIGURED)
    {
        return false;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)USB_CDC_Device.pClassData;
    if (cdc_handle == NULL || cdc_handle->TxState != 0U)
    {
        return false;
    }

    USBD_CDC_SetTxBuffer(&USB_CDC_Device, data, length);
    return (USBD_CDC_TransmitPacket(&USB_CDC_Device) == USBD_OK);
}

/**
 * @brief  Setup阶段回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
    }
}

/**
 * @brief  OUT阶段数据回调
 * @param  hpcd: PCD句柄
 * @param  epnum: 端点号
 * @retval None
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
    }
}

/**
 * @brief  IN阶段数据回调
 * @param  hpcd: PCD句柄
 * @param  epnum: 端点号
 * @retval None
 */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
    }
}

/**
 * @brief  SOF回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
    }
}

/**
 * @brief  Reset回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

    if (hpcd->pData == NULL)
    {
        return;
    }

    if (hpcd->Init.speed == PCD_SPEED_HIGH)
    {
        speed = USBD_SPEED_HIGH;
    }
    else if (hpcd->Init.speed == PCD_SPEED_FULL)
    {
        speed = USBD_SPEED_FULL;
    }
    else
    {
        Error_Handler();
    }

    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, speed);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

/**
 * @brief  Suspend回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
        __HAL_PCD_GATE_PHYCLOCK(hpcd);
    }
}

/**
 * @brief  Resume回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
    }
}

/**
 * @brief  ISO OUT不完整回调
 * @param  hpcd: PCD句柄
 * @param  epnum: 端点号
 * @retval None
 */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
    }
}

/**
 * @brief  ISO IN不完整回调
 * @param  hpcd: PCD句柄
 * @param  epnum: 端点号
 * @retval None
 */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
    }
}

/**
 * @brief  Connect回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
    }
}

/**
 * @brief  Disconnect回调
 * @param  hpcd: PCD句柄
 * @retval None
 */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->pData != NULL)
    {
        USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
    }
}

/**
 * @brief  初始化设备驱动的底层部分
 * @param  pdev: 设备句柄
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    if (pdev->id != DEVICE_HS)
    {
        return USBD_FAIL;
    }

    // 复用CubeMX已经创建并初始化好的USB OTG HS PCD句柄，不在此重复调用HAL_PCD_Init
    hpcd_USB_OTG_HS.pData = pdev;
    pdev->pData = &hpcd_USB_OTG_HS;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_SOF_CB_ID, HAL_PCD_SOFCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_SETUPSTAGE_CB_ID, HAL_PCD_SetupStageCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_RESET_CB_ID, HAL_PCD_ResetCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_SUSPEND_CB_ID, HAL_PCD_SuspendCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_RESUME_CB_ID, HAL_PCD_ResumeCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_CONNECT_CB_ID, HAL_PCD_ConnectCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_HS, HAL_PCD_DISCONNECT_CB_ID, HAL_PCD_DisconnectCallback);

    HAL_PCD_RegisterDataOutStageCallback(&hpcd_USB_OTG_HS, HAL_PCD_DataOutStageCallback);
    HAL_PCD_RegisterDataInStageCallback(&hpcd_USB_OTG_HS, HAL_PCD_DataInStageCallback);
    HAL_PCD_RegisterIsoOutIncpltCallback(&hpcd_USB_OTG_HS, HAL_PCD_ISOOUTIncompleteCallback);
    HAL_PCD_RegisterIsoInIncpltCallback(&hpcd_USB_OTG_HS, HAL_PCD_ISOINIncompleteCallback);
#endif

    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS, USB_CDC_RX_FIFO_WORD_SIZE);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 0U, USB_CDC_TX_EP0_FIFO_WORD_SIZE);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 1U, USB_CDC_TX_EP1_FIFO_WORD_SIZE);

    return USBD_OK;
}

/**
 * @brief  反初始化设备驱动的底层部分
 * @param  pdev: 设备句柄
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData));
}

/**
 * @brief  启动设备驱动的底层部分
 * @param  pdev: 设备句柄
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData));
}

/**
 * @brief  停止设备驱动的底层部分
 * @param  pdev: 设备句柄
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData));
}

/**
 * @brief  打开端点
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @param  ep_type: 端点类型
 * @param  ep_mps: 端点最大包长
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep_addr, ep_mps, ep_type));
}

/**
 * @brief  关闭端点
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

/**
 * @brief  清空端点
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

/**
 * @brief  设置端点Stall状态
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

/**
 * @brief  清除端点Stall状态
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

/**
 * @brief  Sets the USB address
 * @param  pdev: 设备句柄
 * @param  dev_addr: 设备地址
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, dev_addr));
}

/**
 * @brief  通过端点发送数据
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @param  pbuf: 数据缓冲区指针
 * @param  size: 数据长度
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size));
}

/**
 * @brief  为端点准备接收缓冲区
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @param  pbuf: 数据缓冲区指针
 * @param  size: 数据长度
 * @retval USBD状态
 */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return USB_CDC_Get_USB_Status(HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size));
}

/**
 * @brief  获取端点Stall状态
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @retval Stall状态
 */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *pcd_handle = (PCD_HandleTypeDef *)pdev->pData;

    if ((ep_addr & 0x80U) == 0x80U)
    {
        return (uint8_t)pcd_handle->IN_ep[ep_addr & 0x7FU].is_stall;
    }

    return (uint8_t)pcd_handle->OUT_ep[ep_addr & 0x7FU].is_stall;
}

/**
 * @brief  获取端点实际接收数据长度
 * @param  pdev: 设备句柄
 * @param  ep_addr: 端点地址
 * @retval 实际接收长度
 */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

/**
 * @brief  USB设备核心延时回调
 * @param  Delay: 延时毫秒数
 * @retval None
 */
void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

/**
 * @brief  USB设备核心使用的静态malloc
 * @param  size: 申请大小
 * @retval 静态缓冲区指针
 */
void *USBD_static_malloc(uint32_t size)
{
    static uint32_t USB_CDC_Static_Memory[(sizeof(USBD_CDC_HandleTypeDef) / 4U) + 1U];

    UNUSED(size);
    return USB_CDC_Static_Memory;
}

/**
 * @brief  USB设备核心使用的静态free
 * @param  p: 内存指针
 * @retval None
 */
void USBD_static_free(void *p)
{
    UNUSED(p);
}

} // extern "C"

/**
 * @brief          CDC类初始化
 * @param[in]      none
 * @retval         USBD状态
 */
static int8_t USB_CDC_Class_Init(void)
{
    USBD_CDC_SetTxBuffer(&USB_CDC_Device, USB_CDC_Class_Tx_Buffer, 0U);
    USBD_CDC_SetRxBuffer(&USB_CDC_Device, USB_CDC_Class_Rx_Buffer);
    return (int8_t)USBD_OK;
}

/**
 * @brief          CDC类反初始化
 * @param[in]      none
 * @retval         USBD状态
 */
static int8_t USB_CDC_Class_DeInit(void)
{
    return (int8_t)USBD_OK;
}

/**
 * @brief          CDC控制请求处理
 * @param[in]      cmd     控制命令
 *                 pbuf    控制缓冲区
 *                 length  数据长度
 * @retval         USBD状态
 */
static int8_t USB_CDC_Class_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    UNUSED(cmd);
    UNUSED(pbuf);
    UNUSED(length);
    return (int8_t)USBD_OK;
}

/**
 * @brief          USB CDC接收回调
 * @param[in]      Buf   当前收到的数据首地址
 *                 Len   当前收到的数据长度
 * @retval         USBD状态
 */
static int8_t USB_CDC_Class_Receive_Callback(uint8_t *Buf, uint32_t *Len)
{
    BaseType_t xhigher_priority_task_woken = pdFALSE;

    if ((Buf != NULL) && (Len != NULL) && (USB_CDC_Active_Manage_Object != NULL))
    {
        uint16_t rx_length = (uint16_t)((*Len > USB_CDC_RX_SINGLE_PACKET_SIZE) ? USB_CDC_RX_SINGLE_PACKET_SIZE : *Len);

        // USB中断上下文不在HAL层直接解析协议，仅将原始包转交给APL层入口
        USB_CDC_RxCallback(USB_CDC_Active_Manage_Object, Buf, rx_length, &xhigher_priority_task_woken);
    }

    USBD_CDC_SetRxBuffer(&USB_CDC_Device, USB_CDC_Class_Rx_Buffer);
    USBD_CDC_ReceivePacket(&USB_CDC_Device);

    portYIELD_FROM_ISR(xhigher_priority_task_woken);
    return (int8_t)USBD_OK;
}

/**
 * @brief          USB CDC发送完成回调
 * @param[in]      Buf    数据缓冲区
 *                 Len    数据长度
 *                 epnum  端点号
 * @retval         USBD状态
 */
static int8_t USB_CDC_Class_Transmit_Complete_Callback(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);
    return (int8_t)USBD_OK;
}

/**
 * @brief          获取设备描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_Device_Descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    *length = sizeof(USB_CDC_Device_Descriptor_Buffer);
    return USB_CDC_Device_Descriptor_Buffer;
}

/**
 * @brief          获取语言ID描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_LangID_Descriptor_Get(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    *length = sizeof(USB_CDC_LangID_Descriptor);
    return USB_CDC_LangID_Descriptor;
}

/**
 * @brief          获取制造商字符串描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_Manufacturer_String_Get(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    USBD_GetString((uint8_t *)USB_CDC_MANUFACTURER_STRING, USB_CDC_String_Descriptor, length);
    return USB_CDC_String_Descriptor;
}

/**
 * @brief          获取产品字符串描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_Product_String_Get(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    USBD_GetString((uint8_t *)USB_CDC_PRODUCT_STRING, USB_CDC_String_Descriptor, length);
    return USB_CDC_String_Descriptor;
}

/**
 * @brief          获取序列号字符串描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_Serial_String_Get(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    *length = USB_CDC_STRING_SERIAL_SIZE;
    USB_CDC_Get_Serial_Number();
    return USB_CDC_Serial_String;
}

/**
 * @brief          获取配置字符串描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_Config_String_Get(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    USBD_GetString((uint8_t *)USB_CDC_CONFIGURATION_STRING, USB_CDC_String_Descriptor, length);
    return USB_CDC_String_Descriptor;
}

/**
 * @brief          获取接口字符串描述符
 * @param[in]      speed   当前USB速率
 * @param[out]     length  描述符长度
 * @retval         描述符首地址
 */
static uint8_t *USB_CDC_Interface_String_Get(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);
    USBD_GetString((uint8_t *)USB_CDC_INTERFACE_STRING, USB_CDC_String_Descriptor, length);
    return USB_CDC_String_Descriptor;
}

/**
 * @brief          生成USB序列号字符串
 * @param[in]      none
 * @retval         none
 */
static void USB_CDC_Get_Serial_Number(void)
{
    uint32_t deviceserial0 = HAL_GetUIDw0();
    uint32_t deviceserial1 = HAL_GetUIDw1();
    uint32_t deviceserial2 = HAL_GetUIDw2();

    deviceserial0 += deviceserial2;

    if (deviceserial0 != 0U)
    {
        USB_CDC_Int_To_Unicode(deviceserial0, &USB_CDC_Serial_String[2], 8U);
        USB_CDC_Int_To_Unicode(deviceserial1, &USB_CDC_Serial_String[18], 4U);
    }
}

/**
 * @brief          将32位数值转为USB Unicode字符串
 * @param[in]      value   原始数值
 *                 buffer  输出缓冲区
 *                 length  转换长度
 * @retval         none
 */
static void USB_CDC_Int_To_Unicode(uint32_t value, uint8_t *buffer, uint8_t length)
{
    for (uint8_t index = 0U; index < length; index++)
    {
        if (((value >> 28) & 0xFU) < 0xAU)
        {
            buffer[2U * index] = (uint8_t)(((value >> 28) & 0xFU) + '0');
        }
        else
        {
            buffer[2U * index] = (uint8_t)(((value >> 28) & 0xFU) + 'A' - 10U);
        }

        buffer[(2U * index) + 1U] = 0U;
        value <<= 4U;
    }
}

/**
 * @brief          HAL状态转USB设备状态
 * @param[in]      hal_status HAL函数返回值
 * @retval         USB设备状态
 */
static USBD_StatusTypeDef USB_CDC_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return USBD_OK;

        case HAL_ERROR:
            return USBD_FAIL;

        case HAL_BUSY:
            return USBD_BUSY;

        case HAL_TIMEOUT:
        default:
            return USBD_FAIL;
    }
}

