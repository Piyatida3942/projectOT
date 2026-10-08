/*******************************************************************************
 * File Name    : uart.c
 * Description  : UART2 TX ring buffer
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "uart.h"

/*==============================================================================
 * UART TX Ring Buffer
 *============================================================================*/
void UART2_TxString(const char strOut[])
{
    uint16_t idx = 0u;

    while (strOut[idx] != '\0') {
        uint16_t next_head = (uint16_t)((tx_head + 1u) % TX_BUFFER_SIZE);

        if (next_head == tx_tail) {
            break;
        }
        else {
            /* No action */
        }

        tx_buffer[tx_head] = strOut[idx];
        tx_head = next_head;
        idx++;
    }

    USART2->CR1 |= USART_CR1_TXEIE;
}
