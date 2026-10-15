//
// USB CDC serial console - implementation
//
// - syscalls.c calls __io_putchar once per character; that only appends here,
//   because one USB transfer per character would be slower than the printf
// - the buffer goes out on a newline, or when it fills up
// - two buffers, not one: the USB stack reads the buffer in the background
//   after CDC_Transmit_FS returns, so printf has to fill the other one
// - every send is gated on the port being open and bounded by a timeout, or a
//   board with no terminal attached would sit here forever
//

#include "usb_serial.h"

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdio.h>

// the device handle usb_device.c defines; it is not declared in usb_device.h
extern USBD_HandleTypeDef hUsbDeviceFS;

// one debug line, comfortably
#define USB_SERIAL_BUF_SIZE 128

// give up on a stalled transfer instead of blocking the firmware
#define USB_SERIAL_TX_TIMEOUT_MS 10

static uint8_t buf[2][USB_SERIAL_BUF_SIZE];
static uint8_t active; // the buffer printf is filling right now
static uint16_t used;
static uint32_t drops;

void usb_serial_init(void) {
    active = 0;
    used = 0;
    drops = 0;

    // newlib keeps printf output in a buffer of its own otherwise, and a
    // debug line that arrives a kilobyte later is no use
    setvbuf(stdout, NULL, _IONBF, 0);
}

bool usb_serial_ready(void) {
    // CDC_Transmit_FS dereferences pClassData with no NULL check, and that
    // pointer is only set once the host configures the device
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

void usb_serial_flush(void) {
    if (used == 0u) {
        return;
    }

    if (!usb_serial_ready()) {
        drops += used; // nothing listening, drop the line
        used = 0u;
        return;
    }

    uint8_t res;
    uint32_t start = HAL_GetTick();

    // BUSY means the previous transfer has not finished; it never will if the
    // terminal is closed, so stop trying once the timeout is up
    for (;;) {
        res = CDC_Transmit_FS(buf[active], used);
        if (res != USBD_BUSY) {
            break;
        }
        if ((HAL_GetTick() - start) >= USB_SERIAL_TX_TIMEOUT_MS) {
            drops += used;
            used = 0u;
            return; // nothing was handed over, keep the same buffer
        }
    }

    if (res != USBD_OK) {
        drops += used;
        used = 0u;
        return;
    }

    // the stack is reading buf[active] in the background now, so move printf
    // to the other one and leave this one alone until it is done
    active ^= 1u;
    used = 0u;
}

uint32_t usb_serial_drops(void) { return drops; }

// put one byte in the buffer, flushing first if it is full
static void append(uint8_t c) {
    if (used >= USB_SERIAL_BUF_SIZE) {
        usb_serial_flush();
    }
    if (used < USB_SERIAL_BUF_SIZE) {
        buf[active][used++] = c;
    } else {
        drops++;
    }
}

// syscalls.c calls this once per character coming out of printf
int __io_putchar(int ch) {
    // screen leaves the cursor in place on a bare newline, so lines would
    // stair-step across the terminal; send the carriage return with it
    if (ch == '\n') {
        append((uint8_t)'\r');
    }
    append((uint8_t)ch);

    if (ch == '\n') {
        usb_serial_flush();
    }
    return ch;
}
