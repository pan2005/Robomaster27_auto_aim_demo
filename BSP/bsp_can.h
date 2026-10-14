#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "fdcan.h"
#include <stdint.h>

#define BSP_CAN_MAX_RX_CALLBACKS 24U

typedef void (*bsp_can_rx_callback_t)(void *device,
                                      const uint8_t *data,
                                      uint8_t length);

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t id;
    uint32_t id_type;
    bsp_can_rx_callback_t callback;
    void *device;
    uint8_t registered;
} bsp_can_rx_hook_t;

typedef struct
{
    uint32_t rx_count;
    uint32_t last_rx_id;
    uint32_t tx_ok_count;
    uint32_t tx_fail_count;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t bus_off;
    uint8_t error_passive;
    uint8_t warning;
    uint8_t activity;
    uint8_t last_error_code;
} BspCanDebug_t;

HAL_StatusTypeDef BSP_CAN_Init(void);
HAL_StatusTypeDef BSP_CAN_Start(FDCAN_HandleTypeDef *hfdcan);

int BSP_CAN_RegisterRxCallback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t id,
                               uint32_t id_type,
                               bsp_can_rx_callback_t callback,
                               void *device);

HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t id,
                               uint32_t id_type,
                               const uint8_t *data,
                               uint8_t length);

void BSP_CAN_GetDebug(FDCAN_HandleTypeDef *hfdcan, BspCanDebug_t *debug);

#endif /* BSP_CAN_H */
