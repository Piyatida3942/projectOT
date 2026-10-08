/*******************************************************************************
 * File Name    : interrupts.c
 * Description  : Interrupt handlers
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "app_config.h"
#include "uart.h"
#include "sorter.h"

/*==============================================================================
 * Interrupt Handlers
 *============================================================================*/
void SysTick_Handler(void)
{
    msTicks++;

    if ((current_state == STATE_RUNNING) ||
        (current_state == STATE_WAIT_LDR) ||
        (current_state == STATE_SETTLE) ||
        (current_state == STATE_EVALUATE) ||
        (current_state == STATE_ROUTE_ACCEPT) ||
        (current_state == STATE_ROUTE_REJECT) ||
        (current_state == STATE_WAIT_LDR_CLEAR)) {
        if ((msTicks % 1000u) == 0u) {
            elapsed_time++;
        }
        else {
            /* No action */
        }
    }
    else {
        /* No action */
    }
}

void USART2_IRQHandler(void)
{
    if ((USART2->SR & USART_SR_RXNE) != 0u) {
        char rx_data = (char)USART2->DR;

        if ((rx_data == '\n') || (rx_data == '\r')) {
            if (rx_index > 0u) {
                rx_buffer[rx_index] = '\0';
                rx_index = 0u;
                config_received = 1u;
            }
            else {
                /* No action */
            }
        }
        else if (rx_index < (RX_BUFFER_SIZE - 1u)) {
            rx_buffer[rx_index] = rx_data;
            rx_index++;
        }
        else {
            /* No action */
        }
    }
    else {
        /* No action */
    }

    if (((USART2->SR & USART_SR_TXE) != 0u) && ((USART2->CR1 & USART_CR1_TXEIE) != 0u)) {
        if (tx_head != tx_tail) {
            USART2->DR = tx_buffer[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) % TX_BUFFER_SIZE);
        }
        else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    else {
        /* No action */
    }
}

/* PB3 = Reset Button */
void EXTI3_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR3) != 0u) {
        if ((msTicks - last_btn_reset) >= BUTTON_DEBOUNCE_MS) {
            last_btn_reset = msTicks;
            Request_Reset_Report();
        }
        else {
            /* No action */
        }
        EXTI->PR |= EXTI_PR_PR3;
    }
    else {
        /* No action */
    }
}

/* PB4 = Start Button */
void EXTI4_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR4) != 0u) {
        if ((msTicks - last_btn_start) >= BUTTON_DEBOUNCE_MS) {
            last_btn_start = msTicks;

            if (current_state == STATE_READY) {
                elapsed_time = 0u;
                last_printed_sec = TIME_SENTINEL_UNSET;
                current_state = STATE_RUNNING;
                UART2_TxString("\r\n[SYS] Sorting Started!\r\n");
            }
            else {
                /* No action */
            }
        }
        else {
            /* No action */
        }
        EXTI->PR |= EXTI_PR_PR4;
    }
    else {
        /* No action */
    }
}

/* PB5 = Pause / Resume Button */
void EXTI9_5_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR5) != 0u) {
        if ((msTicks - last_btn_pause) >= BUTTON_DEBOUNCE_MS) {
            last_btn_pause = msTicks;

            if ((current_state == STATE_RUNNING) ||
                (current_state == STATE_WAIT_LDR) ||
                (current_state == STATE_SETTLE) ||
                (current_state == STATE_EVALUATE) ||
                (current_state == STATE_ROUTE_ACCEPT) ||
                (current_state == STATE_ROUTE_REJECT) ||
                (current_state == STATE_WAIT_LDR_CLEAR)) {
                Pause_System();
            }
            else if (current_state == STATE_PAUSED) {
                Resume_System();
            }
            else if (current_state == STATE_FAULT) {
                Resume_From_Fault();
            }
            else {
                /* No action */
            }
        }
        else {
            /* No action */
        }
        EXTI->PR |= EXTI_PR_PR5;
    }
    else {
        /* No action */
    }
}

/* PA10 = Emergency Button Module */
void TIM1_CC_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_CC3IF) != 0u) {
        TIM1->SR &= ~TIM_SR_CC3IF;

        if ((msTicks - last_btn_emergency) >= BUTTON_DEBOUNCE_MS) {
            last_btn_emergency = msTicks;
            Enter_Emergency();
        }
        else {
            /* No action */
        }
    }
    else {
        /* No action */
    }
}

/* PC10 = IR Sensor */
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR10) != 0u) {
        EXTI->PR |= EXTI_PR_PR10;

        if ((msTicks - last_manual_trigger) >= IR_COOLDOWN_MS) {
            last_manual_trigger = msTicks;

            if ((current_state == STATE_RUNNING) && (package_active == 0u)) {
                ir_event_pending = 1u;
            }
            else {
                /* No action */
            }
        }
        else {
            /* No action */
        }
    }
    else {
        /* No action */
    }
}
