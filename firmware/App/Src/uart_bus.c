//
// UART bus wrapper - implementation
//
// - HAL_UART_Receive_IT asks for one byte; when it lands the interrupt fires,
//   store it and immediately ask for the next one
// - head is only written by the interrupt, tail only by the main loop, so the
//   two never fight over the same variable and no locking is needed
//

#include "uart_bus.h"
#include "stm32f4xx_hal.h"

// the USART1 handle CubeMX generates in Core/Src/usart.c
extern UART_HandleTypeDef huart1;

// power of two so the wrap is a cheap mask instead of a divide. 2048 bytes is
// ~2.1 s at 9600 baud - a record write is 5 blocks and the SD spec lets each
// one stay busy 250 ms, and sessions 6 and 7 already saw a 245 ms write
// against the ~265 ms a 256-byte ring lasted
#define UART_BUS_RING_SIZE 2048
#define UART_BUS_RING_MASK (UART_BUS_RING_SIZE - 1)

static uint8_t rx_byte;                        // HAL drops each new byte here
static volatile uint8_t ring[UART_BUS_RING_SIZE];
static volatile uint16_t head;                 // written by the interrupt only
static volatile uint16_t tail;                 // written by the main loop only
static volatile uint32_t overruns;
static volatile uint32_t rx_count;

uart_bus_status_t uart_bus_start(void) {
    head = 0;
    tail = 0;
    overruns = 0;
    rx_count = 0;

    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) {
        return UART_BUS_ERR_HAL;
    }
    return UART_BUS_OK;
}

// HAL calls this from the USART1 interrupt once a byte has arrived
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) {
        return;
    }

    rx_count++;  // counted before the ring check, so it includes the lost ones

    uint16_t next = (uint16_t)((head + 1u) & UART_BUS_RING_MASK);
    if (next != tail) {
        ring[head] = rx_byte;
        head = next;
    } else {
        overruns++;  // buffer full, this byte is lost
    }

    // ask for the next byte, or reception stops here for good
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

// a noise or framing error stops HAL's reception
// restart it or the GPS goes silent until the next reboot
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) {
        return;
    }
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

bool uart_bus_read_byte(uint8_t *out) {
    if (out == NULL) {
        return false;
    }
    if (tail == head) {
        return false;  // nothing waiting
    }

    *out = ring[tail];
    tail = (uint16_t)((tail + 1u) & UART_BUS_RING_MASK);
    return true;
}

uint32_t uart_bus_overruns(void) {
    return overruns;
}

uint32_t uart_bus_rx_count(void) {
    return rx_count;
}