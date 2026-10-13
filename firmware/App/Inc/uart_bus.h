//
// UART bus wrapper - public interface
//
// - wraps USART1, which is where the GPS talks; USART6 would need its own
//   instance of this
// - the GPS sends whenever it wants, so an interrupt drops each byte into a
//   ring buffer and the main loop takes them out later
//

#ifndef UART_BUS_H
#define UART_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    UART_BUS_OK = 0,
    UART_BUS_ERR_HAL,
} uart_bus_status_t;

// start listening on USART1; call after MX_USART1_UART_Init
uart_bus_status_t uart_bus_start(void);

// take one byte out of the buffer; false when there is nothing waiting
bool uart_bus_read_byte(uint8_t *out);

// send bytes out on TX, blocking until the last one has left; the GPS only
// gets a few short commands at boot, so blocking is fine
uart_bus_status_t uart_bus_send(const uint8_t *data, uint16_t len);

// change the baud rate on the fly - stops reception, re-sets the UART at the
// new rate, empties the buffer (what was in it came at the old rate) and
// starts listening again. CubeMX keeps 9600, which is what the GPS boots at
uart_bus_status_t uart_bus_set_baud(uint32_t baud);

// bytes thrown away because the buffer was full - should stay at 0
uint32_t uart_bus_overruns(void);

// every byte the UART has received since start, stored or not - the first
// thing to look at when nothing parses; stuck at 0 means no signal at all
uint32_t uart_bus_rx_count(void);

#endif // UART_BUS_H