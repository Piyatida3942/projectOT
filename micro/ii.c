/*******************************************************************************
 * File Name    : main.c
 * Description  : FSM Conveyor System - Non-Blocking (Fixed Reset & Reject LED)
 * Board        : Training Shield 1 Rev 02.00
 * Hardware     : HW-480 Reject LED on PB8
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* --- Hardware Pin Mapping --- */
#define LIGHT_SENSOR_PIN 1      // PA1  (LDR Light Sensor - ADC Channel 1)
#define POT_PIN          4      // PA4  (Potentiometer - ADC Channel 4)

/* --- LED Mapping --- */
#define LED_STATUS_PIN   5      // PA5  (Blue - System Status)
#define LED_RED_PIN      6      // PA6  (Red - Size S)
#define LED_YELLOW_PIN   7      // PA7  (Yellow - Size M)
#define LED_GREEN_PIN    6      // PB6  (Green - Size L)
#define LED_REJECT_PIN   8      // PB8  (HW-480 Reject LED)

/* --- Thresholds & Timers --- */
#define THRESHOLD_STAGE1 1365
#define THRESHOLD_STAGE2 2730
#define LDR_THRESHOLD_DARK  2500

#define LED_HOLD_TIME_MS    500   // ระยะเวลาค้างไฟ LED (0.5 วินาที)
#define IR_DEBOUNCE_MS      250
#define IR_COOLDOWN_MS      1200  // ป้องกันการรับค่า IR ซ้ำซ้อน
#define LDR_TIMEOUT_MS      3000
#define IR_SETTLE_TIME_MS   300

/* --- Ring Buffer Config --- */
#define TX_BUFFER_SIZE      512

/* --- FSM States --- */
typedef enum {
    STATE_IDLE,
    STATE_READY,
    STATE_RUNNING,
    STATE_WAIT_LDR,
    STATE_SETTLE,
    STATE_EVALUATE,
    STATE_PAUSED,
    STATE_REPORT
} SystemState;

/* --- Global Variables --- */
volatile SystemState current_state = STATE_IDLE;
volatile uint32_t msTicks = 0;

volatile uint16_t adc_buffer[2];

// Config Target
volatile uint16_t target_S = 0, target_M = 0, target_L = 0;
volatile uint32_t target_time = 0;

// Metrics
volatile uint16_t count_S = 0, count_M = 0, count_L = 0;
volatile uint16_t reject_S = 0, reject_M = 0, reject_L = 0;
volatile uint16_t err_object_lost = 0, err_ghost_ir = 0;
volatile uint32_t elapsed_time = 0, last_printed_sec = 0xFFFFFFFF;

// Timers Control
volatile uint32_t size_led_off_time = 0, wait_ldr_start_time = 0, settle_start_time = 0;
volatile uint8_t size_led_active = 0;

// Button / Sensor Debounce Timers
volatile uint32_t last_btn_start = 0, last_btn_pause = 0, last_btn_reset = 0, last_ir_trigger = 0;

// Serial Buffers
char rx_buffer[50];
volatile uint8_t rx_index = 0, config_received = 0;

char tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_head = 0, tx_tail = 0;

char stringOut[250];

/* --- Prototypes --- */
void System_Init(void);
static void UART2_TxString(char strOut[]);
static void Process_Evaluate(uint16_t pot_val);
static void Reset_Metrics(void);
static void Clear_Size_LEDs(void);
static void Flash_All_LEDs(void);
static void Parse_Config(char* str);

/*==============================================================================
 * Main Function
 *============================================================================*/
int main(void)
{
    System_Init();
    UART2_TxString("\r\n=== System Booted (Non-Blocking Mode: IR -> LDR) ===\r\nWaiting for PC Config...\r\n");

    while (1)
    {
        // 1. จัดการดับไฟแสดงผลลัพธ์ (LEDs) เมื่อครบเวลา
        if (size_led_active && msTicks >= size_led_off_time) {
            Clear_Size_LEDs();
            size_led_active = 0;
        }

        // 2. ควบคุมไฟ Status LED (PA5)
        if (current_state == STATE_RUNNING || current_state == STATE_WAIT_LDR || current_state == STATE_SETTLE) {
            GPIOA->BSRR = (1 << LED_STATUS_PIN);
        } else if (current_state == STATE_PAUSED) {
            if ((msTicks / 250) % 2) GPIOA->BSRR = (1 << LED_STATUS_PIN);
            else GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16));
        } else {
            GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16));
        }

        // 3. Finite State Machine (FSM)
        switch (current_state)
        {
            case STATE_IDLE:
                if (config_received) {
                    Parse_Config(rx_buffer);
                    config_received = 0;
                    current_state = STATE_READY;
                    sprintf(stringOut, "Config Loaded -> S:%d, M:%d, L:%d, Time:%lus\r\nPress START (PB4)...\r\n", target_S, target_M, target_L, target_time);
                    UART2_TxString(stringOut);
                }
                break;

            case STATE_READY:
                break;

            case STATE_RUNNING:
                if (target_time > 0 && elapsed_time >= target_time) {
                    UART2_TxString("\r\n[TIME EXPIRED]\r\n");
                    current_state = STATE_REPORT;
                    break;
                }

                if (target_time > 0 && elapsed_time != last_printed_sec) {
                    last_printed_sec = elapsed_time;
                    sprintf(stringOut, "[TIME REMAINING: %lu s]\r\n", target_time - elapsed_time);
                    UART2_TxString(stringOut);
                }
                break;

            case STATE_WAIT_LDR:
                if (adc_buffer[0] > LDR_THRESHOLD_DARK) {
                    settle_start_time = msTicks;
                    current_state = STATE_SETTLE;
                    UART2_TxString("-> [LDR] Verified. Settling before evaluation...\r\n");
                }
                else if ((msTicks - wait_ldr_start_time) > LDR_TIMEOUT_MS) {
                    err_object_lost++;
                    UART2_TxString("-> [ERROR: OBJECT_LOST] LDR verification timeout. Resuming...\r\n");
                    current_state = STATE_RUNNING;
                }
                break;

            case STATE_SETTLE:
                if ((msTicks - settle_start_time) > IR_SETTLE_TIME_MS) {
                    current_state = STATE_EVALUATE;
                }
                break;

            case STATE_EVALUATE:
                Process_Evaluate(adc_buffer[1]);
                current_state = STATE_RUNNING;
                break;

            case STATE_PAUSED:
                break;

            case STATE_REPORT:
                sprintf(stringOut,
                        "\r\n=== TEST REPORT ===\r\nSUCCESS  -> S:%d/%d, M:%d/%d, L:%d/%d\r\nREJECTED -> S:%d, M:%d, L:%d\r\nERRORS   -> Lost:%d, Ghost:%d\r\nELAPSED TIME -> %lu s\r\n===================\r\n",
                        count_S, target_S, count_M, target_M, count_L, target_L,
                        reject_S, reject_M, reject_L,
                        err_object_lost, err_ghost_ir, elapsed_time);
                UART2_TxString(stringOut);
                Reset_Metrics();
                Clear_Size_LEDs();
                current_state = STATE_IDLE;
                UART2_TxString("\r\nWaiting for new config...\r\n");
                break;
        }
    }
    return 0;
}

/*==============================================================================
 * Helper Functions
 *============================================================================*/

static void Parse_Config(char* str) {
    target_S = target_M = target_L = target_time = 0;
    char* ptr = strchr(str, 'S');
    if (ptr != NULL) sscanf(ptr, "S%hu,M%hu,L%hu,T%lu", &target_S, &target_M, &target_L, &target_time);
}

static void Process_Evaluate(uint16_t pot_val) {
    Clear_Size_LEDs();

    if (pot_val < THRESHOLD_STAGE1) {
        if (count_S < target_S) {
            count_S++;
            GPIOA->BSRR = (1 << LED_RED_PIN);
            sprintf(stringOut, "-> [DETECTED] Size S (ACCEPTED %d/%d)\r\n", count_S, target_S);
        } else {
            reject_S++;
            GPIOA->BSRR = (1 << LED_RED_PIN);
            GPIOB->BSRR = (1 << LED_REJECT_PIN); // เปิดไฟ Reject
            sprintf(stringOut, "-> [DETECTED] Size S (REJECTED %d/%d)\r\n", count_S, target_S);
        }
    }
    else if (pot_val < THRESHOLD_STAGE2) {
        if (count_M < target_M) {
            count_M++;
            GPIOA->BSRR = (1 << LED_YELLOW_PIN);
            sprintf(stringOut, "-> [DETECTED] Size M (ACCEPTED %d/%d)\r\n", count_M, target_M);
        } else {
            reject_M++;
            GPIOA->BSRR = (1 << LED_YELLOW_PIN);
            GPIOB->BSRR = (1 << LED_REJECT_PIN); // เปิดไฟ Reject
            sprintf(stringOut, "-> [DETECTED] Size M (REJECTED %d/%d)\r\n", count_M, target_M);
        }
    }
    else {
        if (count_L < target_L) {
            count_L++;
            GPIOB->BSRR = (1 << LED_GREEN_PIN);
            sprintf(stringOut, "-> [DETECTED] Size L (ACCEPTED %d/%d)\r\n", count_L, target_L);
        } else {
            reject_L++;
            GPIOB->BSRR = (1 << LED_GREEN_PIN);
            GPIOB->BSRR = (1 << LED_REJECT_PIN); // เปิดไฟ Reject
            sprintf(stringOut, "-> [DETECTED] Size L (REJECTED %d/%d)\r\n", count_L, target_L);
        }
    }

    UART2_TxString(stringOut);
    size_led_off_time = msTicks + LED_HOLD_TIME_MS;
    size_led_active = 1;
}

static void Reset_Metrics(void) {
    count_S = count_M = count_L = 0;
    reject_S = reject_M = reject_L = 0;
    err_object_lost = err_ghost_ir = 0;
    elapsed_time = 0;
    last_printed_sec = 0xFFFFFFFF;
    Clear_Size_LEDs(); // ดับไฟทุกดวงทันทีเมื่อล้างค่า
}

static void Clear_Size_LEDs(void) {
    GPIOA->BSRR = (1 << (LED_RED_PIN + 16)) | (1 << (LED_YELLOW_PIN + 16));
    GPIOB->BSRR = (1 << (LED_GREEN_PIN + 16)) | (1 << (LED_REJECT_PIN + 16));
}

// ปรับแก้เป็น Non-blocking 100%: เปิดไฟแล้วฝากให้ลูป main ดับไฟเองตามเวลา
static void Flash_All_LEDs(void) {
    GPIOA->BSRR = (1 << LED_STATUS_PIN) | (1 << LED_RED_PIN) | (1 << LED_YELLOW_PIN);
    GPIOB->BSRR = (1 << LED_GREEN_PIN) | (1 << LED_REJECT_PIN);
    size_led_off_time = msTicks + 200; // กำหนดให้ติดค้างไว้ 200ms
    size_led_active = 1;               // แจ้งเตือน main() ให้มาคอยดับไฟ
}

static void UART2_TxString(char strOut[]){
    for (uint16_t idx = 0; strOut[idx] != '\0'; idx++) {
        uint16_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;
        if (next_head != tx_tail) {
            tx_buffer[tx_head] = strOut[idx];
            tx_head = next_head;
        } else {
            break;
        }
    }
    USART2->CR1 |= USART_CR1_TXEIE;
}

/*==============================================================================
 * Hardware Init
 *============================================================================*/
void System_Init(void) {
    SysTick_Config(16000000 / 1000);

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN);
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN);

    GPIOA->MODER &= ~((3 << (LED_STATUS_PIN * 2)) | (3 << (LED_RED_PIN * 2)) | (3 << (LED_YELLOW_PIN * 2)));
    GPIOA->MODER |= ((1 << (LED_STATUS_PIN * 2)) | (1 << (LED_RED_PIN * 2)) | (1 << (LED_YELLOW_PIN * 2)));

    GPIOB->MODER &= ~((3 << (LED_GREEN_PIN * 2)) | (3 << (LED_REJECT_PIN * 2)));
    GPIOB->MODER |= ((1 << (LED_GREEN_PIN * 2)) | (1 << (LED_REJECT_PIN * 2)));
    Clear_Size_LEDs();

    GPIOA->MODER |= (3 << (LIGHT_SENSOR_PIN * 2)) | (3 << (POT_PIN * 2));

    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD3_Pos) | (1 << GPIO_PUPDR_PUPD4_Pos) | (1 << GPIO_PUPDR_PUPD5_Pos);

    GPIOC->MODER &= ~GPIO_MODER_MODER10;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOC->PUPDR |= (1 << GPIO_PUPDR_PUPD10_Pos);

    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3); SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4); SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI5); SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;
    SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI10); SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PC;

    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR10);
    EXTI->FTSR |= (EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5 | EXTI_FTSR_TR10);
    EXTI->RTSR &= ~EXTI_RTSR_TR10;

    NVIC_EnableIRQ(EXTI3_IRQn); NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn); NVIC_EnableIRQ(EXTI15_10_IRQn);

    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= ((2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos));
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos));

    USART2->BRR = 0x8B;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE);
    NVIC_EnableIRQ(USART2_IRQn);

    DMA2_Stream0->CR = 0;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;
    DMA2_Stream0->NDTR = 2;
    DMA2_Stream0->CR |= (0 << 25) | (1 << 13) | (1 << 11) | (1 << 10) | (1 << 8);
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SQR1 |= (1 << 20);
    ADC1->SQR3 = (LIGHT_SENSOR_PIN << 0) | (POT_PIN << 5);
    ADC1->SMPR2 |= (7 << (LIGHT_SENSOR_PIN * 3)) | (7 << (POT_PIN * 3));

    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/*==============================================================================
 * Interrupt Handlers
 *============================================================================*/

void SysTick_Handler(void) {
    msTicks++;
    if ((current_state == STATE_RUNNING || current_state == STATE_WAIT_LDR || current_state == STATE_SETTLE) && (msTicks % 1000 == 0)) {
        elapsed_time++;
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char rx_data = (char)USART2->DR;
        if (rx_data == '\n' || rx_data == '\r') {
            if (rx_index > 0) { rx_buffer[rx_index] = '\0'; rx_index = 0; config_received = 1; }
        } else { if (rx_index < 49) rx_buffer[rx_index++] = rx_data; }
    }

    if ((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (tx_head != tx_tail) {
            USART2->DR = tx_buffer[tx_tail];
            tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

// EXTI3: Reset Button (ปลอดคำสั่งค้างลูป)
void EXTI3_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR3) {
        if ((msTicks - last_btn_reset) > 300) {
            last_btn_reset = msTicks;

            Flash_All_LEDs(); // สั่งกระพริบไฟแบบ Non-blocking

            if (current_state == STATE_IDLE || current_state == STATE_READY) {
                Reset_Metrics();
                target_S = target_M = target_L = target_time = 0;
                current_state = STATE_IDLE;
                UART2_TxString("\r\n[RESET] Cleared!\r\n");
            } else {
                Clear_Size_LEDs();
                UART2_TxString("\r\n[RESET] Jumped to Report!\r\n");
                current_state = STATE_REPORT; // กระโดดไปออก Report ได้ทันที ไม่ค้างแล้ว
            }
        }
        EXTI->PR |= EXTI_PR_PR3;
    }
}

void EXTI4_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR4) {
        if ((msTicks - last_btn_start) > 300) {
            last_btn_start = msTicks;
            if (current_state == STATE_READY) { current_state = STATE_RUNNING; last_printed_sec = 0xFFFFFFFF; UART2_TxString("\r\n[STARTED]\r\n"); }
        }
        EXTI->PR |= EXTI_PR_PR4;
    }
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR5) {
        if ((msTicks - last_btn_pause) > 300) {
            last_btn_pause = msTicks;
            if (current_state == STATE_RUNNING || current_state == STATE_WAIT_LDR || current_state == STATE_SETTLE) {
                current_state = STATE_PAUSED; UART2_TxString("\r\n[PAUSED]\r\n");
            }
            else if (current_state == STATE_PAUSED) {
                current_state = STATE_RUNNING; last_printed_sec = 0xFFFFFFFF; UART2_TxString("\r\n[RESUMED]\r\n");
            }
        }
        EXTI->PR |= EXTI_PR_PR5;
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR10) {
        EXTI->PR |= EXTI_PR_PR10;

        if ((msTicks - last_ir_trigger) > IR_COOLDOWN_MS) {
            last_ir_trigger = msTicks;

            if (current_state == STATE_RUNNING) {
                wait_ldr_start_time = msTicks;
                current_state = STATE_WAIT_LDR;
                UART2_TxString("-> [IR] Object detected (Input). Waiting for LDR...\r\n");
            }
        }
    }
}
