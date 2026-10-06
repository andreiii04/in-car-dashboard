//
// USB CDC serial console - public interface
//
// - retargets printf onto the USB-C cable already used for power and DFU; the
//   Mac sees a virtual COM port, so no debugger and no extra wiring
// - printf lands in a line buffer here and only reaches USB on a newline, so
//   one transfer carries a whole line instead of one per character
// - main-loop use only; printing from an interrupt would corrupt the buffer
//

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

// call once after MX_USB_DEVICE_Init
void usb_serial_init(void);

// true once the host has enumerated and configured the port
bool usb_serial_ready(void);

// push whatever is buffered out now, without waiting for a newline
void usb_serial_flush(void);

// characters thrown away because nothing was listening - should stay at 0
// while a terminal is open
uint32_t usb_serial_drops(void);

#endif // USB_SERIAL_H
