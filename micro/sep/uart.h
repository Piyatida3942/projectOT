/*******************************************************************************
 * File Name    : uart.h
 * Description  : UART2 TX ring buffer API
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#ifndef UART_H
#define UART_H

#include "app_config.h"

void UART2_TxString(const char strOut[]);

#endif /* UART_H */
