/*******************************************************************************
 * File Name    : main.c
 * Description  : FSM Conveyor System with Countdown & Quota Log
 * Board        : Training Shield 1 Rev 02.00
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* --- Hardware Pin Mapping --- */
#define LIGHT_SENSOR_PIN 1      // PA1  (LDR Light Sensor)
#define POT_PIN          4      // PA4  (Potentiometer)

/* --- LED Mapping --- */
#define LED_STATUS_PIN   5      // PA5  (Blue - System Status)
#define LED_RED_PIN      6      // PA6  (Red - Size S)
#define LED_YELLOW_PIN   7      // PA7  (Yellow - Size M)
#define LED_GREEN_PIN    6      // PB6  (Green - Size L)

/* --- Thresholds --- */
#define THRESHOLD_STAGE1 1365
#define THRESHOLD_STAGE2 2730
#define LDR_DARK_THRESHOLD  1200
#define LDR_LIGHT_THRESHOLD 2000

#define LED_HOLD_TIME_MS    500   // ระยะเวลาค้างไฟ LED (0.5 วินาที)

/* --- FSM States --- */
typedef enum {
    STATE_IDLE,
    STATE_READY,
    STATE_RUNNING,
    STATE_EVALUATE,
    STATE_PAUSED,
    STATE_REPORT
} SystemState;

/* --- Global Variables --- */
volatile SystemState current_state = STATE_IDLE;
volatile uint32_t msTicks = 0;
volatile uint16_t light_adc_val = 0;
volatile uint16_t pot_adc_val = 0;

// Config Target
volatile uint16_t target_S = 0, target_M = 0, target_L = 0;
volatile uint32_t target_time = 0;

// Accepted Count
volatile uint16_t count_S = 0, count_M = 0, count_L = 0;

// Rejected Count
volatile uint16_t reject_S = 0, reject_M = 0, reject_L = 0;
volatile uint32_t elapsed_time = 0;
volatile uint32_t last_printed_sec = 0xFFFFFFFF; // ตัวแปรเช็กการพิมพ์นับถอยหลัง

// LED Timer Control
volatile uint32_t size_led_off_time = 0;
volatile uint8_t size_led_active = 0;

// Button Debounce Timers
volatile uint32_t last_btn_start = 0;
volatile uint32_t last_btn_pause = 0;
volatile uint32_t last_btn_reset = 0;
static uint8_t is_covered = 0;

// Serial Buffer
char rx_buffer[50];
volatile uint8_t rx_index = 0;
volatile uint8_t config_received = 0;
char stringOut[200];

/* --- Prototypes --- */
void System_Init(void);
static void UART2_TxString(char strOut[]);
static uint16_t ADC_Read_Channel(uint8_t channel);
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
    UART2_TxString("\r\n=== System Booted ===\r\nWaiting for PC Config (Format: S10,M5,L2,T60)...\r\n");

    while (1)
    {
        light_adc_val = ADC_Read_Channel(LIGHT_SENSOR_PIN);
        pot_adc_val   = ADC_Read_Channel(POT_PIN);

        // 1. จัดการการดับไฟแสดงขนาดพัสดุเมื่อครบ 0.5 วินาที
        if (size_led_active && msTicks >= size_led_off_time) {
            Clear_Size_LEDs();
            size_led_active = 0;
        }

        // 2. ควบคุมไฟ PA5 (Status LED)
        if (current_state == STATE_RUNNING) {
            GPIOA->BSRR = (1 << LED_STATUS_PIN);
        } else if (current_state == STATE_PAUSED) {
            if ((msTicks / 250) % 2) {
                GPIOA->BSRR = (1 << LED_STATUS_PIN);
            } else {
                GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16));
            }
        } else {
            GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16));
        }

        // 3. Finite State Machine
        switch (current_state)
        {
            case STATE_IDLE:
                if (config_received) {
                    Parse_Config(rx_buffer);
                    config_received = 0;
                    current_state = STATE_READY;

                    sprintf(stringOut, "Config Loaded -> S:%d, M:%d, L:%d, Time:%lus\r\nPress START button (PB4)...\r\n",
                            target_S, target_M, target_L, target_time);
                    UART2_TxString(stringOut);
                }
                break;

            case STATE_READY:
                break;

            case STATE_RUNNING:
                // เช็กหมดเวลา
                if (target_time > 0 && elapsed_time >= target_time) {
                    UART2_TxString("\r\n[TIME EXPIRED]\r\n");
                    current_state = STATE_REPORT;
                    break;
                }

                // นับถอยหลังบน Serial Terminal ทุกๆ 1 วินาที
                if (target_time > 0 && elapsed_time != last_printed_sec) {
                    last_printed_sec = elapsed_time;
                    uint32_t remaining_time = target_time - elapsed_time;
                    sprintf(stringOut, "[TIME REMAINING: %lu s]\r\n", remaining_time);
                    UART2_TxString(stringOut);
                }

                // ตรวจจับพัสดุ
                if (light_adc_val < LDR_DARK_THRESHOLD && !is_covered) {
                    is_covered = 1;
                    current_state = STATE_EVALUATE;
                }

                if (light_adc_val > LDR_LIGHT_THRESHOLD) {
                    is_covered = 0;
                }
                break;

            case STATE_EVALUATE:
                Process_Evaluate(pot_adc_val);
                current_state = STATE_RUNNING;
                break;

            case STATE_PAUSED:
                break;

            case STATE_REPORT:
                sprintf(stringOut,
                        "\r\n=== TEST REPORT ===\r\n"
                        "SUCCESS  -> S:%d/%d, M:%d/%d, L:%d/%d\r\n"
                        "REJECTED -> S:%d, M:%d, L:%d\r\n"
                        "ELAPSED TIME -> %lu s\r\n"
                        "===================\r\n",
                        count_S, target_S, count_M, target_M, count_L, target_L,
                        reject_S, reject_M, reject_L,
                        elapsed_time);
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

static uint16_t ADC_Read_Channel(uint8_t channel) {
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

static void Parse_Config(char* str) {
    target_S = target_M = target_L = target_time = 0;
    char* ptr = strchr(str, 'S');
    if (ptr != NULL) {
        sscanf(ptr, "S%hu,M%hu,L%hu,T%lu", &target_S, &target_M, &target_L, &target_time);
    }
}

static void Process_Evaluate(uint16_t pot_val)
{
    Clear_Size_LEDs();

    if (pot_val < THRESHOLD_STAGE1) {
        if (count_S < target_S) {
            count_S++;
            GPIOA->BSRR = (1 << LED_RED_PIN);
            sprintf(stringOut, "-> [DETECTED] Size S (ACCEPTED %d/%d)\r\n", count_S, target_S);
        } else {
            reject_S++;
            GPIOA->BSRR = (1 << LED_RED_PIN);
            sprintf(stringOut, "-> [DETECTED] Size S (REJECTED - OVER QUOTA %d/%d)\r\n", count_S, target_S);
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
            sprintf(stringOut, "-> [DETECTED] Size M (REJECTED - OVER QUOTA %d/%d)\r\n", count_M, target_M);
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
            sprintf(stringOut, "-> [DETECTED] Size L (REJECTED - OVER QUOTA %d/%d)\r\n", count_L, target_L);
        }
    }

    UART2_TxString(stringOut);

    // กำหนดเวลาค้างไฟ LED (0.5 วินาที)
    size_led_off_time = msTicks + LED_HOLD_TIME_MS;
    size_led_active = 1;
}

static void Reset_Metrics(void) {
    count_S = count_M = count_L = 0;
    reject_S = reject_M = reject_L = 0;
    elapsed_time = 0;
    last_printed_sec = 0xFFFFFFFF;
    is_covered = 0;
}

static void Clear_Size_LEDs(void) {
    GPIOA->BSRR = (1 << (LED_RED_PIN + 16)) | (1 << (LED_YELLOW_PIN + 16));
    GPIOB->BSRR = (1 << (LED_GREEN_PIN + 16));
}

static void Flash_All_LEDs(void) {
    GPIOA->BSRR = (1 << LED_STATUS_PIN) | (1 << LED_RED_PIN) | (1 << LED_YELLOW_PIN);
    GPIOB->BSRR = (1 << LED_GREEN_PIN);

    for(volatile int i = 0; i < 600000; i++);

    GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16)) | (1 << (LED_RED_PIN + 16)) | (1 << (LED_YELLOW_PIN + 16));
    GPIOB->BSRR = (1 << (LED_GREEN_PIN + 16));
}

static void UART2_TxString(char strOut[]){
    for (uint8_t idx = 0; strOut[idx] != '\0'; idx++){
        while((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = strOut[idx];
    }
}

/*==============================================================================
 * Hardware Init
 *============================================================================*/
void System_Init(void)
{
    SysTick_Config(16000000 / 1000);

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN);

    // Output LEDs
    GPIOA->MODER &= ~((3 << (LED_STATUS_PIN * 2)) | (3 << (LED_RED_PIN * 2)) | (3 << (LED_YELLOW_PIN * 2)));
    GPIOA->MODER |= ((1 << (LED_STATUS_PIN * 2)) | (1 << (LED_RED_PIN * 2)) | (1 << (LED_YELLOW_PIN * 2)));
    GPIOB->MODER &= ~(3 << (LED_GREEN_PIN * 2));
    GPIOB->MODER |= (1 << (LED_GREEN_PIN * 2));
    Clear_Size_LEDs();

    // Analog Inputs
    GPIOA->MODER |= (3 << (LIGHT_SENSOR_PIN * 2)) | (3 << (POT_PIN * 2));

    // Digital Inputs
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOA->MODER &= ~GPIO_MODER_MODER10;

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD3_Pos) | (1 << GPIO_PUPDR_PUPD4_Pos) | (1 << GPIO_PUPDR_PUPD5_Pos);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |= (1 << GPIO_PUPDR_PUPD10_Pos);

    // UART2
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= ((2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos));
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos));

    USART2->BRR = 0x8B;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE);
    NVIC_EnableIRQ(USART2_IRQn);

    // EXTI Mapping
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;  // PB3 (Reset)

    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4);
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;  // PB4 (Start)

    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI5);
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;  // PB5 (Pause)

    SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI10);
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA; // PA10 (Manual)

    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR10);
    EXTI->FTSR |= (EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5 | EXTI_FTSR_TR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // ADC1
    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->SMPR2 |= (7 << (LIGHT_SENSOR_PIN * 3)) | (7 << (POT_PIN * 3));
    ADC1->CR2 |= ADC_CR2_ADON;
}

/*==============================================================================
 * Interrupt Handlers
 *============================================================================*/

void SysTick_Handler(void)
{
    msTicks++;
    if (current_state == STATE_RUNNING && (msTicks % 1000 == 0)) {
        elapsed_time++;
    }
}

void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        char rx_data = (char)USART2->DR;
        if (rx_data == '\n' || rx_data == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                rx_index = 0;
                config_received = 1;
            }
        } else {
            if (rx_index < 49) {
                rx_buffer[rx_index++] = rx_data;
            }
        }
    }
}

// PB3: RESET BUTTON
void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR3) {
        if ((msTicks - last_btn_reset) > 300) {
            last_btn_reset = msTicks;
            Flash_All_LEDs();

            if (current_state == STATE_IDLE || current_state == STATE_READY) {
                Reset_Metrics();
                target_S = target_M = target_L = target_time = 0;
                current_state = STATE_IDLE;
                UART2_TxString("\r\n[RESET] Config Cleared! Waiting for new config...\r\n");
            } else if (current_state == STATE_RUNNING || current_state == STATE_PAUSED) {
                UART2_TxString("\r\n[RESET] Test Jumped to Report by User!\r\n");
                current_state = STATE_REPORT;
            }
        }
        EXTI->PR |= EXTI_PR_PR3;
    }
}

// PB4: START BUTTON
void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR4) {
        if ((msTicks - last_btn_start) > 300) {
            last_btn_start = msTicks;

            if (current_state == STATE_READY) {
                // 1. เคลียร์ Pending Interrupt ของปุ่ม PA10 ที่ค้างอยู่ก่อนหน้า
                EXTI->PR |= EXTI_PR_PR10;

                // 2. ตั้งเป็น 1 เพื่อบังคับให้ระบบต้องเห็นแสงสว่างก่อน (LDR > LDR_LIGHT_THRESHOLD)
                // ถึงจะเริ่มรับพัสดุชิ้นแรก ตัดปัญหาพัสดุแรกเด้งเอง 100%
                is_covered = 1;

                current_state = STATE_RUNNING;
                last_printed_sec = 0xFFFFFFFF; // รีเซ็ตตัวนับเวลาบน Serial
                UART2_TxString("\r\n[TEST STARTED]\r\n");
            }
        }
        EXTI->PR |= EXTI_PR_PR4;
    }
}

// PB5: PAUSE / RESUME BUTTON
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR5) {
        if ((msTicks - last_btn_pause) > 300) {
            last_btn_pause = msTicks;

            if (current_state == STATE_RUNNING) {
                current_state = STATE_PAUSED;
                UART2_TxString("\r\n[PAUSED] Press PAUSE (PB5) again to Resume...\r\n");
            } else if (current_state == STATE_PAUSED) {
                current_state = STATE_RUNNING;
                last_printed_sec = 0xFFFFFFFF;
                UART2_TxString("\r\n[RESUMED] Test Continuing...\r\n");
            }
        }
        EXTI->PR |= EXTI_PR_PR5;
    }
}

// PA10: MANUAL TRIGGER BUTTON
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR10) {
        if (current_state == STATE_RUNNING) {
            current_state = STATE_EVALUATE;
        }
        EXTI->PR |= EXTI_PR_PR10;
    }
}
