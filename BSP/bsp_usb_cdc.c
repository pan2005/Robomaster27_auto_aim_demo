#include "bsp_usb_cdc.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define USB_RX_BUFFER_SIZE 256
#define USB_RX_QUEUE_LEN 512

extern USBD_HandleTypeDef hUsbDeviceHS;

struct usb_data {
    USBD_HandleTypeDef *handle;
    uint8_t rxdatas[USB_RX_BUFFER_SIZE];
    uint16_t rxlen;
    QueueHandle_t xRxQueue;
};

static struct usb_data g_usb_data = {
    .handle = &hUsbDeviceHS,
};

static int usb_cdc_init(struct usb_device *pDev)
{
    struct usb_data *pData = (struct usb_data *)pDev->priv_data;
    pData->rxlen = 0;
    pData->rxdatas[0] = 0;
    pData->xRxQueue = xQueueCreate(USB_RX_QUEUE_LEN, sizeof(uint8_t));

    // 初始化接收缓冲区
    USBD_CDC_SetRxBuffer(pData->handle, pData->rxdatas);
    USBD_CDC_ReceivePacket(pData->handle);

    return 0;
}

static int usb_cdc_send(struct usb_device *pDev, char *data, int len)
{
    (void)pDev;

    if (CDC_Transmit_HS((uint8_t *)data, len) == USBD_OK)
        return len;
    else
        return 0;
}

static int usb_cdc_printf(struct usb_device *pDev, const char *fmt, ...)
{
    (void)pDev;
    uint8_t tx_buf[256];
    va_list ap;
    uint16_t len;

    va_start(ap, fmt);
    len = vsnprintf((char *)tx_buf, sizeof(tx_buf), fmt, ap);
    va_end(ap);

    if (CDC_Transmit_HS(tx_buf, len) == USBD_OK)
        return len;
    else
        return 0;
}

static int usb_cdc_recv(struct usb_device *pDev, char *data, int max_len, int timeout_ms)
{
    struct usb_data *pData = (struct usb_data *)pDev->priv_data;
    int count = 0;
    memset(data, 0, max_len);

    // 等待第一个字符
    if(pdPASS == xQueueReceive(pData->xRxQueue, (void *)&data[count], pdMS_TO_TICKS(timeout_ms))) {
        count++;

        // 继续接收剩余字符(非阻塞)
        while(count < max_len &&
              pdPASS == xQueueReceive(pData->xRxQueue, (void *)&data[count], 0)) {
            count++;
        }

        return count; // 返回实际接收到的字符数
    }

    return -1; // 超时
}

// USB接收回调处理函数
void usb_cdc_receive_callback(uint8_t* Buf, uint16_t Len)
{
    struct usb_data *pData = &g_usb_data;

    // 将接收到的数据放入队列
    for (uint32_t i = 0; i < Len; i++) {
        xQueueSendFromISR(pData->xRxQueue, &Buf[i], NULL);
    }

    // 重新启动接收
    USBD_CDC_SetRxBuffer(pData->handle, pData->rxdatas);
    USBD_CDC_ReceivePacket(pData->handle);
}

struct usb_device g_usb_cdc_dev = {
    "usb_cdc",
    usb_cdc_init,
    usb_cdc_send,
    usb_cdc_printf,
    usb_cdc_recv,
    &g_usb_data
};

struct usb_device *usb_get_device(void)
{
    return &g_usb_cdc_dev;
}
