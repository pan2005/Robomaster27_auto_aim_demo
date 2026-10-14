#ifndef INFANTRY_01_USB_CDC_H
#define INFANTRY_01_USB_CDC_H


#include "main.h"

struct usb_device
{
    char *name;
    int (*Init)(struct usb_device *pDev);
    int (*Send)(struct usb_device *pDev, char *data, int len);
    int (*Print)(struct usb_device *pDev, const char *fmt, ...);
    int (*Recv)(struct usb_device *pDev, char *data, int max_len, int timeout_ms);
    void *priv_data;
};

extern void usb_cdc_receive_callback(uint8_t* Buf, uint16_t Len);

extern struct usb_device *usb_get_device(void);


#endif //INFANTRY_01_USB_CDC_H
