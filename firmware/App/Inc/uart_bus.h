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

// bytes thrown away because the buffer was full - should stay at 0
uint32_t uart_bus_overruns(void);

// every byte the UART has received since start, stored or not - the first
// thing to look at when nothing parses; stuck at 0 means no signal at all
uint32_t uart_bus_rx_count(void);

#endif // UART_BUS_H