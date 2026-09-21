/*******************************************************************************
 * File Name    : main.c
 * Description  : FSM Conveyor Belt System (Updated LED Mapping)
 * Board        : Training Shield 1 Rev 02.00
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* Pin Mapping (Physical Pins) */
#define LIGHT_SENSOR_PIN 1      // PA1 (Light Sensor - LDR)
#define POT_PIN          4      // PA4 (Potentiometer)

/* LED Mapping by Color */
#define LED_BLUE_PIN     5      // PA5 (Used for Reject)
#define LED_RED_PIN      6      // PA6 (Used for Size S)
#define LED_YELLOW_PIN   7      // PA7 (Used for Size M)
#define LED_GREEN_PIN    6      // PB6 (Used for Size L)

/* ADC Thresholds */
#define THRESHOLD_STAGE1 1365   // แบ่ง Size S กับ M
#define THRESHOLD_STAGE2 2730   // แบ่ง Size M กับ L

/* Hysteresis Thresholds สำหรับ LDR */
#define LDR_DARK_THRESHOLD  1200  // ต่ำกว่าค่านิคือโดนบัง (มืด)
#define LDR_LIGHT_THRESHOLD 2000  // สูงกว่าค่านิคือเปิดว่าง (สว่าง)

/*---- FSM States ----*/
typedef enum {
    STATE_IDLE,
    STATE_READY,
    STATE_RUNNING,
    STATE_EVALUATE,
    STATE_PAUSED,
    STATE_REPORT
} SystemState;

/*---- Global Variables ----*/
volatile SystemState current_state = STATE_IDLE;
volatile uint32_t msTicks = 0;
volatile uint16_t light_adc_val = 0;
volatile uint16_t pot_adc_val = 0;

// Target Variables
volatile uint16_t target_S = 0, target_M = 0, target_L = 0;
volatile uint32_t target_time = 0;

// Runtime Metrics
volatile uint16_t count_S = 0, count_M = 0, count_L = 0;
volatile uint16_t reject_count = 0;
volatile uint32_t elapsed_time = 0;

// Debounce & Trigger Tracking
volatile uint32_t last_btn_start = 0;
volatile uint32_t last_btn_pause = 0;
static uint8_t is_covered = 0;

// Serial Communication
char rx_buffer[50];
volatile uint8_t rx_index = 0;
volatile uint8_t config_received = 0;
char stringOut[150];

/*---- Function Prototypes ----*/
void System_Init(void);
static void UART2_TxString(char strOut[]);
static uint16_t ADC_Read_Channel(uint8_t channel);
static void Process_Evaluate(uint16_t pot_val);
static void Reset_Metrics(void);
static void Clear_LEDs(void);
static void Parse_Config(char* str);

/*---- Main function ----*/
int main(void)
{
    System_Init();

    UART2_TxString("\r\n=== System Booted ===\r\nWaiting for PC Config (Format: S10,M5,L2,T60)...\r\n");

    while (1)
    {
        // อ่านค่า ADC จาก LDR (PA1) และ POT (PA4)
        light_adc_val = ADC_Read_Channel(LIGHT_SENSOR_PIN);
        pot_adc_val   = ADC_Read_Channel(POT_PIN);

        switch (current_state)
        {
            case STATE_IDLE:
                if (config_received) {
                    Parse_Config(rx_buffer);
                    config_received = 0;
                    current_state = STATE_READY;
                    sprintf(stringOut, "Config Loaded -> S:%d, M:%d, L:%d, Time:%lus\r\nPress START button (PB3)...\r\n",
                            target_S, target_M, target_L, target_time);
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

                // Hysteresis Logic: บังแสง (มืด) ครั้งแรก -> ล็อคค้างไว้ยิงประมวลผลครั้งเดียว
                if (light_adc_val < LDR_DARK_THRESHOLD && !is_covered) {
                    is_covered = 1;
                    current_state = STATE_EVALUATE;
                }

                // แสงกลับมาสว่างแล้ว -> คลายล็อค พร้อมรับพัสดุชิ้นถัดไป
                if (light_adc_val > LDR_LIGHT_THRESHOLD) {
                    is_covered = 0;
                }
                break;

            case STATE_EVALUATE:
                Process_Evaluate(pot_adc_val);
                Clear_LEDs();
                current_state = STATE_RUNNING;
                break;

            case STATE_PAUSED:
                break;

            case STATE_REPORT:
                sprintf(stringOut, "\r\n=== TEST REPORT ===\r\nSUCCESS -> S:%d, M:%d, L:%d\r\nREJECTED -> %d\r\nELAPSED TIME -> %lu s\r\n===================\r\n",
                        count_S, count_M, count_L, reject_count, elapsed_time);
                UART2_TxString(stringOut);

                Reset_Metrics();
                Clear_LEDs();
                current_state = STATE_IDLE;
                UART2_TxString("\r\nWaiting for new config...\r\n");
                break;
        }
    }
    return 0;
}

/*---- Helper Functions ----*/
static uint16_t ADC_Read_Channel(uint8_t channel) {
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

static void Parse_Config(char* str) {
    target_S = target_M = target_L = 0;
    target_time = 0;

    char* ptr = strchr(str, 'S');
    if (ptr != NULL) {
        sscanf(ptr, "S%hu,M%hu,L%hu,T%lu", &target_S, &target_M, &target_L, &target_time);
    }
}

static void Process_Evaluate(uint16_t pot_val)
{
    if (pot_val < THRESHOLD_STAGE1) {
        if (count_S < target_S) {
            count_S++;
            GPIOA->BSRR = (1 << LED_RED_PIN); // Red = Size S
            UART2_TxString("-> [DETECTED] Size S (ACCEPTED)\r\n");
        } else {
            reject_count++;
            GPIOA->BSRR = (1 << LED_BLUE_PIN); // Blue = Reject
            UART2_TxString("-> [DETECTED] Size S (REJECTED - OVER QUOTA)\r\n");
        }
    } else if (pot_val < THRESHOLD_STAGE2) {
        if (count_M < target_M) {
            count_M++;
            GPIOA->BSRR = (1 << LED_YELLOW_PIN); // Yellow = Size M
            UART2_TxString("-> [DETECTED] Size M (ACCEPTED)\r\n");
        } else {
            reject_count++;
            GPIOA->BSRR = (1 << LED_BLUE_PIN); // Blue = Reject
            UART2_TxString("-> [DETECTED] Size M (REJECTED - OVER QUOTA)\r\n");
        }
    } else {
        if (count_L < target_L) {
            count_L++;
            GPIOB->BSRR = (1 << LED_GREEN_PIN); // Green = Size L
            UART2_TxString("-> [DETECTED] Size L (ACCEPTED)\r\n");
        } else {
            reject_count++;
            GPIOA->BSRR = (1 << LED_BLUE_PIN); // Blue = Reject
            UART2_TxString("-> [DETECTED] Size L (REJECTED - OVER QUOTA)\r\n");
        }
    }
}

static void Reset_Metrics(void) {
    count_S = count_M = count_L = reject_count = elapsed_time = 0;
}

static void Clear_LEDs(void) {
    GPIOA->BSRR = (1 << (LED_BLUE_PIN + 16)) | (1 << (LED_RED_PIN + 16)) | (1 << (LED_YELLOW_PIN + 16));
    GPIOB->BSRR = (1 << (LED_GREEN_PIN + 16));
}

static void UART2_TxString(char strOut[]){
    for (uint8_t idx = 0; strOut[idx] != '\0'; idx++){
        while((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = strOut[idx];
    }
}

/*---- Hardware Initialization ----*/
void System_Init(void)
{
    SysTick_Config(16000000 / 1000);

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN);

    // Outputs Setup (LEDs)
    GPIOA->MODER &= ~((3 << (LED_BLUE_PIN * 2)) | (3 << (LED_RED_PIN * 2)) | (3 << (LED_YELLOW_PIN * 2)));
    GPIOA->MODER |= ((1 << (LED_BLUE_PIN * 2)) | (1 << (LED_RED_PIN * 2)) | (1 << (LED_YELLOW_PIN * 2)));
    GPIOB->MODER &= ~(3 << (LED_GREEN_PIN * 2));
    GPIOB->MODER |= (1 << (LED_GREEN_PIN * 2));
    Clear_LEDs();

    // Analog Inputs: PA1 (Light Sensor) and PA4 (Potentiometer)
    GPIOA->MODER |= (3 << (LIGHT_SENSOR_PIN * 2)) | (3 << (POT_PIN * 2));

    // Digital Inputs: PB3 (Start), PB4 (Pause), PA10 (Manual Trigger D2)
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4);
    GPIOA->MODER &= ~GPIO_MODER_MODER10;

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4);
    GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD3_Pos) | (1 << GPIO_PUPDR_PUPD4_Pos);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |= (1 << GPIO_PUPDR_PUPD10_Pos);

    // UART2 Setup
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= ((2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos));
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos));

    USART2->BRR = 0x8B;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE);
    NVIC_EnableIRQ(USART2_IRQn);

    // EXTI Setup
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;

    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4);
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;

    SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI10);
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA;

    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR10);
    EXTI->FTSR |= (EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // ADC1 Setup
    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->SMPR2 |= (7 << (LIGHT_SENSOR_PIN * 3)) | (7 << (POT_PIN * 3));
    ADC1->CR2 |= ADC_CR2_ADON;
}

/*---- Interrupt Handlers ----*/

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

void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR3) {
        if ((msTicks - last_btn_start) > 250) {
            last_btn_start = msTicks;

            if (current_state == STATE_IDLE) {
                Reset_Metrics();
                UART2_TxString("Config Cleared!\r\n");
            } else if (current_state == STATE_READY) {
                current_state = STATE_RUNNING;
                UART2_TxString("\r\n[TEST STARTED]\r\n");
            } else if (current_state == STATE_RUNNING || current_state == STATE_PAUSED) {
                UART2_TxString("\r\n[TEST ABORTED BY USER]\r\n");
                current_state = STATE_REPORT;
            }
        }
        EXTI->PR |= EXTI_PR_PR3;
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR4) {
        if ((msTicks - last_btn_pause) > 250) {
            last_btn_pause = msTicks;

            if (current_state == STATE_RUNNING) {
                current_state = STATE_PAUSED;
                UART2_TxString("\r\n[PAUSED] Press PAUSE again to Resume...\r\n");
            } else if (current_state == STATE_PAUSED) {
                current_state = STATE_RUNNING;
                UART2_TxString("\r\n[RESUMED] Test Continuing...\r\n");
            }
        }
        EXTI->PR |= EXTI_PR_PR4;
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR10) {
        if (current_state == STATE_RUNNING) {
            current_state = STATE_EVALUATE;
        }
        EXTI->PR |= EXTI_PR_PR10;
    }
}
