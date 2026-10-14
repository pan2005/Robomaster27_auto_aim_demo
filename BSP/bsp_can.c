#include "bsp_can.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static bsp_can_rx_hook_t rx_hooks[BSP_CAN_MAX_RX_CALLBACKS];
static BspCanDebug_t can_debug[3];

static BspCanDebug_t *CanDebugForHandle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return &can_debug[0];
    if (hfdcan == &hfdcan2) return &can_debug[1];
    if (hfdcan == &hfdcan3) return &can_debug[2];
    return NULL;
}

static uint32_t CanDlcFromLength(uint8_t length)
{
    static const uint32_t dlc[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    return dlc[length];
}

static uint8_t CanLengthFromDlc(uint32_t dlc)
{
    /* This HAL version already returns the decoded DLC value (0..15).
     * Older STM32 HAL releases exposed it in bits 19:16, which is why the
     * original F4-era code used a 16-bit shift.  Shifting here made every
     * classic CAN frame appear to have length zero. */
    return (dlc <= FDCAN_DLC_BYTES_8) ? (uint8_t)dlc : 8U;
}

HAL_StatusTypeDef BSP_CAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) return HAL_ERROR;

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return HAL_ERROR;
    return HAL_FDCAN_ActivateNotification(hfdcan,
                                          FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                          0U);
}

HAL_StatusTypeDef BSP_CAN_Init(void)
{
    memset(rx_hooks, 0, sizeof(rx_hooks));
    memset(can_debug, 0, sizeof(can_debug));

    if (BSP_CAN_Start(&hfdcan1) != HAL_OK) return HAL_ERROR;
    if (BSP_CAN_Start(&hfdcan2) != HAL_OK) return HAL_ERROR;
    if (BSP_CAN_Start(&hfdcan3) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

int BSP_CAN_RegisterRxCallback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t id,
                               uint32_t id_type,
                               bsp_can_rx_callback_t callback,
                               void *device)
{
    if ((hfdcan == NULL) || (callback == NULL)) return -1;
    if ((id_type != FDCAN_STANDARD_ID) && (id_type != FDCAN_EXTENDED_ID)) return -1;
    if (((id_type == FDCAN_STANDARD_ID) && (id > 0x7FFU)) ||
        ((id_type == FDCAN_EXTENDED_ID) && (id > 0x1FFFFFFFU))) return -1;

    taskENTER_CRITICAL();
    int free_index = -1;
    for (uint32_t i = 0U; i < BSP_CAN_MAX_RX_CALLBACKS; ++i)
    {
        if (rx_hooks[i].registered != 0U)
        {
            if ((rx_hooks[i].hfdcan == hfdcan) &&
                (rx_hooks[i].id == id) &&
                (rx_hooks[i].id_type == id_type))
            {
                rx_hooks[i].callback = callback;
                rx_hooks[i].device = device;
                taskEXIT_CRITICAL();
                return 0;
            }
        }
        else if (free_index < 0)
        {
            free_index = (int)i;
        }
    }

    if (free_index >= 0)
    {
        bsp_can_rx_hook_t *hook = &rx_hooks[free_index];
        hook->hfdcan = hfdcan;
        hook->id = id;
        hook->id_type = id_type;
        hook->callback = callback;
        hook->device = device;
        hook->registered = 1U;
    }
    taskEXIT_CRITICAL();
    return (free_index >= 0) ? 0 : -1;
}

HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t id,
                               uint32_t id_type,
                               const uint8_t *data,
                               uint8_t length)
{
    if ((hfdcan == NULL) || ((data == NULL) && (length != 0U)) || (length > 8U))
        return HAL_ERROR;

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = id_type;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = CanDlcFromLength(length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    taskENTER_CRITICAL();
    HAL_StatusTypeDef status = (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
                             ? HAL_BUSY
                             : HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header,
                                                             (uint8_t *)data);
    BspCanDebug_t *debug = CanDebugForHandle(hfdcan);
    if (debug != NULL)
    {
        if (status == HAL_OK) ++debug->tx_ok_count;
        else ++debug->tx_fail_count;
    }
    taskEXIT_CRITICAL();
    return status;
}

void BSP_CAN_GetDebug(FDCAN_HandleTypeDef *hfdcan, BspCanDebug_t *debug)
{
    if (debug == NULL) return;

    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    BspCanDebug_t *source = CanDebugForHandle(hfdcan);

    taskENTER_CRITICAL();
    if (source != NULL) *debug = *source;
    else memset(debug, 0, sizeof(*debug));
    taskEXIT_CRITICAL();

    if ((hfdcan != NULL) &&
        (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol) == HAL_OK))
    {
        debug->bus_off = (uint8_t)protocol.BusOff;
        debug->error_passive = (uint8_t)protocol.ErrorPassive;
        debug->warning = (uint8_t)protocol.Warning;
        debug->activity = (uint8_t)protocol.Activity;
        debug->last_error_code = (uint8_t)protocol.LastErrorCode;
    }
    if ((hfdcan != NULL) &&
        (HAL_FDCAN_GetErrorCounters(hfdcan, &errors) == HAL_OK))
    {
        debug->tx_error_count = (uint8_t)errors.TxErrorCnt;
        debug->rx_error_count = (uint8_t)errors.RxErrorCnt;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
            break;

        BspCanDebug_t *debug = CanDebugForHandle(hfdcan);
        if (debug != NULL)
        {
            ++debug->rx_count;
            debug->last_rx_id = header.Identifier;
        }

        uint8_t length = CanLengthFromDlc(header.DataLength);
        for (uint32_t i = 0U; i < BSP_CAN_MAX_RX_CALLBACKS; ++i)
        {
            const bsp_can_rx_hook_t *hook = &rx_hooks[i];
            if ((hook->registered != 0U) &&
                (hook->hfdcan == hfdcan) &&
                (hook->id == header.Identifier) &&
                (hook->id_type == header.IdType))
            {
                hook->callback(hook->device, data, length);
                break;
            }
        }
    }
}
