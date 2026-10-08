/*******************************************************************************
 * File Name    : app_globals.c
 * Description  : Global variable definitions
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "app_config.h"

/* --- Global Variables --- */
volatile SystemState current_state = STATE_IDLE;
volatile SystemState resume_state = STATE_RUNNING;
volatile PackageSize current_size = SIZE_S;
volatile PackageDecision current_decision = DECISION_ACCEPT;

volatile uint32_t msTicks = 0u;
volatile uint16_t adc_buffer[ADC_CHANNEL_COUNT] = {0u, 0u};

volatile uint16_t target_S = 0u;
volatile uint16_t target_M = 0u;
volatile uint16_t target_L = 0u;
volatile uint32_t target_time = 0u;

volatile uint16_t count_S = 0u;
volatile uint16_t count_M = 0u;
volatile uint16_t count_L = 0u;
volatile uint16_t reject_S = 0u;
volatile uint16_t reject_M = 0u;
volatile uint16_t reject_L = 0u;
volatile uint16_t err_ir_fault = 0u;
volatile uint16_t err_ldr_fault = 0u;
volatile uint16_t err_stuck = 0u;
volatile uint8_t last_fault_reason = 0u;

volatile uint32_t elapsed_time = 0u;
volatile uint32_t last_printed_sec = TIME_SENTINEL_UNSET;

volatile uint32_t wait_ldr_start_time = 0u;
volatile uint32_t wait_led_hold_until = 0u;  /* ค้างไฟ Wait LDR ให้เห็นอย่างน้อย LED_HOLD_TIME_MS */
volatile uint32_t settle_start_time = 0u;
volatile uint32_t route_start_time = 0u;
volatile uint32_t ldr_clear_start_time = 0u;
volatile uint32_t pause_start_time = 0u;
volatile uint32_t fault_start_time = 0u;

volatile uint32_t last_btn_start = 0u;
volatile uint32_t last_btn_pause = 0u;
volatile uint32_t last_btn_reset = 0u;
volatile uint32_t last_btn_emergency = 0u;
volatile uint32_t last_manual_trigger = 0u;
volatile uint8_t ir_event_pending = 0u;

volatile LedPatternType led_pattern = LED_PATTERN_NONE;
volatile PackageSize led_pattern_size = SIZE_S;
volatile uint8_t led_pattern_phase = 0u;
volatile uint32_t led_pattern_deadline = 0u;

volatile uint8_t reset_flash_active = 0u;
volatile uint32_t reset_flash_end_time = 0u;
volatile uint8_t report_printed = 0u;

volatile uint8_t package_active = 0u;
volatile uint8_t config_received = 0u;
volatile uint8_t rx_index = 0u;
char rx_buffer[RX_BUFFER_SIZE];

char tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_head = 0u;
volatile uint16_t tx_tail = 0u;

char stringOut[600];
