#include "usb_task.h"

#include "bsp_usb_cdc.h"
#include "cmsis_os.h"

#include <stdint.h>

#define USB_ECHO_BUFFER_SIZE 256U

static void UsbTask_Entry(void *argument);

static const osThreadAttr_t usb_task_attributes = {
    .name = "UsbTask",
    .stack_size = 128U * 4U,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

void UsbTask_Init(void)
{
    (void)osThreadNew(UsbTask_Entry, NULL, &usb_task_attributes);
}

static void UsbTask_Entry(void *argument)
{
    (void)argument;

    struct usb_device *usb_dev = usb_get_device();
    uint8_t echo_buffer[USB_ECHO_BUFFER_SIZE];

    for (;;)
    {
        int received = usb_dev->Recv(usb_dev, (char *)echo_buffer,
                                     sizeof(echo_buffer), 1000);

        if (received > 0)
        {
            while (usb_dev->Send(usb_dev, (char *)echo_buffer, received) == 0)
            {
                osDelay(1);
            }

            /* CDC keeps this buffer until the asynchronous transfer finishes. */
            osDelay(10);
        }
    }
}
