/*******************************************************************************
 * File Name    : app_config.h
 * Description  : Common configuration, types and shared globals
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* --- Active Low/High Configuration for Reject LED --- */
#define REJECT_LED_ACTIVE_LOW 0u /* 0 = Active High (3.3V), 1 = Active Low (GND) */

/* --- Analog inputs --- */
#define LIGHT_SENSOR_PIN 1u      /* PA1 */
#define POT_PIN          4u      /* PA4 */

/* --- LEDs --- */
#define LED_STATUS_PIN   5u      /* PA5 */
#define LED_RED_PIN      6u      /* PA6 : S */
#define LED_YELLOW_PIN   7u      /* PA7 : M */
#define LED_GREEN_PIN    6u      /* PB6 : L */
#define LED_REJECT_PIN   10u     /* PB10: reject */
#define LED_WAIT_LDR_PIN 9u      /* PB9 : wait LDR */

/* --- Servo --- */
#define SERVO_PIN        8u      /* PB8 : Servo motor (TIM4_CH3) */
#define SERVO_POS_NORMAL 150u    /* PWM compare value: pass-through position */
#define SERVO_POS_REJECT 210u    /* PWM compare value: discard position */

/* --- Manual test input / future IR input --- */
#define IR_PIN               10u  /* PC10: IR sensor output (EXTI10) */
#define EMERGENCY_BUTTON_PIN 10u  /* PA10: external Emergency button OUT (TIM1_CH3) */

/* --- Thresholds --- */
#define THRESHOLD_STAGE1      1365u
#define THRESHOLD_STAGE2      2730u
#define LDR_THRESHOLD_DARK    2000u
#define LDR_DARK_WHEN_HIGH     1

#if LDR_DARK_WHEN_HIGH
#define IS_LDR_DARK()          (adc_buffer[0] > LDR_THRESHOLD_DARK)
#else
#define IS_LDR_DARK()          (adc_buffer[0] < LDR_THRESHOLD_DARK)
#endif

/* --- Timing --- */
#define BUTTON_DEBOUNCE_MS    300u
#define IR_COOLDOWN_MS        500u
#define LDR_TIMEOUT_MS        5000u      /* รอ LDR ตรวจพบวัตถุสูงสุด 5 วินาที */
#define LDR_CLEAR_TIMEOUT_MS  3000u      /* รอวัตถุออกจาก LDR สูงสุด 3 วินาที */
#define SETTLE_TIME_MS        300u
#define ROUTE_TIME_MS         400u       /* เวลาให้วัตถุเคลื่อนผ่านเซอโว */
#define PAUSE_TIMEOUT_MS      10000u     /* Pause ค้างไว้เกิน 10s จะตัดเข้า Report */
#define FAULT_TIMEOUT_MS      10000u     /* Fault ค้างไว้เกิน 10s จะตัดเข้า Report */
#define LED_HOLD_TIME_MS      500u
#define RESET_FLASH_DURATION_MS 250u
#define LED_RESULT_SHOW_MS      200u     /* เวลาแสดงผล accept หรือ phase แรกของ reject */
#define LED_RESULT_BLINK_OFF_MS 150u     /* reject: ช่วงดับไฟระหว่างกระพริบ */
#define LED_RESULT_BLINK_ON_MS  200u     /* reject: ช่วงติดไฟรอบที่สอง */

/* --- Buffer sizes --- */
#define RX_BUFFER_SIZE 50u
#define ADC_CHANNEL_COUNT 2u
#define PERCENT_SCALE_FACTOR 10000u
#define PERCENT_WHOLE_DIVISOR 100u
#define TIME_SENTINEL_UNSET 0xFFFFFFFFu

/* --- UART TX ring buffer --- */
#define TX_BUFFER_SIZE 1024u

/* --- FSM States --- */
typedef enum {
    STATE_IDLE,
    STATE_READY,
    STATE_RUNNING,
    STATE_WAIT_LDR,
    STATE_SETTLE,
    STATE_EVALUATE,
    STATE_ROUTE_ACCEPT,
    STATE_ROUTE_REJECT,
    STATE_WAIT_LDR_CLEAR,
    STATE_PAUSED,
    STATE_FAULT,
    STATE_EMERGENCY,
    STATE_COMPLETE,
    STATE_REPORT
} SystemState;

/* --- Package Info --- */
typedef enum {
    SIZE_S,
    SIZE_M,
    SIZE_L
} PackageSize;

typedef enum {
    DECISION_ACCEPT,
    DECISION_REJECT
} PackageDecision;
/* LED Control Patterns */
typedef enum {
    LED_PATTERN_NONE,
    LED_PATTERN_ACCEPT,
    LED_PATTERN_REJECT
} LedPatternType;

/* --- Global Variables (defined in app_globals.c) --- */
extern volatile SystemState current_state;
extern volatile SystemState resume_state;
extern volatile PackageSize current_size;
extern volatile PackageDecision current_decision;
extern volatile uint32_t msTicks;
extern volatile uint16_t adc_buffer[ADC_CHANNEL_COUNT];
extern volatile uint16_t target_S;
extern volatile uint16_t target_M;
extern volatile uint16_t target_L;
extern volatile uint32_t target_time;
extern volatile uint16_t count_S;
extern volatile uint16_t count_M;
extern volatile uint16_t count_L;
extern volatile uint16_t reject_S;
extern volatile uint16_t reject_M;
extern volatile uint16_t reject_L;
extern volatile uint16_t err_ir_fault;
extern volatile uint16_t err_ldr_fault;
extern volatile uint16_t err_stuck;
extern volatile uint8_t last_fault_reason;
extern volatile uint32_t elapsed_time;
extern volatile uint32_t last_printed_sec;
extern volatile uint32_t wait_ldr_start_time;
extern volatile uint32_t wait_led_hold_until;
extern volatile uint32_t settle_start_time;
extern volatile uint32_t route_start_time;
extern volatile uint32_t ldr_clear_start_time;
extern volatile uint32_t pause_start_time;
extern volatile uint32_t fault_start_time;
extern volatile uint32_t last_btn_start;
extern volatile uint32_t last_btn_pause;
extern volatile uint32_t last_btn_reset;
extern volatile uint32_t last_btn_emergency;
extern volatile uint32_t last_manual_trigger;
extern volatile uint8_t ir_event_pending;
extern volatile LedPatternType led_pattern;
extern volatile PackageSize led_pattern_size;
extern volatile uint8_t led_pattern_phase;
extern volatile uint32_t led_pattern_deadline;
extern volatile uint8_t reset_flash_active;
extern volatile uint32_t reset_flash_end_time;
extern volatile uint8_t report_printed;
extern volatile uint8_t package_active;
extern volatile uint8_t config_received;
extern volatile uint8_t rx_index;
extern char rx_buffer[RX_BUFFER_SIZE];
extern char tx_buffer[TX_BUFFER_SIZE];
extern volatile uint16_t tx_head;
extern volatile uint16_t tx_tail;
extern char stringOut[600];

#endif /* APP_CONFIG_H */
