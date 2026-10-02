/*******************************************************************************
 * File Name    : main.c
 * Description  : Conveyor Simulation - Merged IR/Fault/Emergency System
 *                (MISRA-C checksheet compliant - 22 rule pass)
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

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

/* LED Control Patterns */
typedef enum {
    LED_PATTERN_NONE,
    LED_PATTERN_ACCEPT,
    LED_PATTERN_REJECT
} LedPatternType;

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

/* --- Function Prototypes --- */
void System_Init(void);
static void UART2_TxString(const char strOut[]);
static void Process_Evaluate(uint16_t pot_val);
static void Reset_Metrics(void);
static void Clear_All_LEDs(void);
static void Set_All_LEDs(void);
static void Clear_Size_LEDs(void);
static void Set_Size_LED_Output(PackageSize size, uint8_t on);
static void Start_Size_Result_LED(PackageSize size, PackageDecision decision);
static void Update_Size_Result_LED(void);
static void Start_Reset_Flash(void);
static void Set_Emergency_LEDs(void);
static void Wait_LDR_LED_On(void);
static void Wait_LDR_LED_Off(void);
static void Reject_LED_On(void);
static void Reject_LED_Off(void);
static void Parse_Config(char *str);
static uint8_t Targets_Complete(void);
static void Enter_Fault(uint8_t reason);
static void Resume_From_Fault(void);
static void Enter_Emergency(void);
static void Servo_Reject(void);
static void Servo_Normal(void);
static void Pause_System(void);
static void Resume_System(void);
static void Request_Reset_Report(void);
static void Reset_To_Idle(void);
static void Process_IR_Event(void);
static PackageSize Get_Pot_Size(uint16_t pot_val);

/*==============================================================================
 * Main Application
 *============================================================================*/
int main(void)
{
    System_Init();

    /* --- [STARTUP TERMINAL MESSAGE] --- */
    UART2_TxString("\r\n========================================\r\n");
    UART2_TxString("PACKAGE SORTING SYSTEM SIMULATION READY\r\n");
    UART2_TxString("========================================\r\n");
    UART2_TxString("Waiting for Configuration from PC (Format : S2,M2,L2,T30)\r\n");

    while (1)
    {
        /* Reset LED Flash Sequence */
        if (reset_flash_active != 0u) {
            Set_All_LEDs();
            if (msTicks >= reset_flash_end_time) {
                reset_flash_active = 0u;
                Clear_All_LEDs();
            }
            else {
                /* No action */
            }
        }
        else {
            /* No action */
        }

        /* Update Size/Reject Result LEDs */
        Update_Size_Result_LED();

        /* Process IR Sensor Trigger */
        if (ir_event_pending != 0u) {
            ir_event_pending = 0u;
            Process_IR_Event();
        }
        else {
            /* No action */
        }

        /* Status LED Behavior */
        if (reset_flash_active == 0u) {
            if (current_state == STATE_EMERGENCY) {
                Set_Emergency_LEDs();
            }
            else if ((current_state == STATE_WAIT_LDR) ||
                     (((int32_t)(wait_led_hold_until - msTicks) > 0) &&
                      (current_state != STATE_FAULT) &&
                      (current_state != STATE_PAUSED))) {
                GPIOA->BSRR = (1u << LED_STATUS_PIN);
                Wait_LDR_LED_On();
            }
            else {
                Wait_LDR_LED_Off();

                if ((current_state == STATE_RUNNING) ||
                    (current_state == STATE_SETTLE) ||
                    (current_state == STATE_EVALUATE) ||
                    (current_state == STATE_ROUTE_ACCEPT) ||
                    (current_state == STATE_ROUTE_REJECT) ||
                    (current_state == STATE_WAIT_LDR_CLEAR)) {
                    GPIOA->BSRR = (1u << LED_STATUS_PIN);
                }
                else if ((current_state == STATE_PAUSED) || (current_state == STATE_FAULT)) {
                    if (((msTicks / 250u) % 2u) != 0u) {
                        GPIOA->BSRR = (1u << LED_STATUS_PIN);
                    }
                    else {
                        GPIOA->BSRR = (1u << (LED_STATUS_PIN + 16u));
                    }
                }
                else {
                    GPIOA->BSRR = (1u << (LED_STATUS_PIN + 16u));
                }
            }
        }
        else {
            /* No action */
        }

        /* FSM Engine */
        switch (current_state)
        {
            case STATE_IDLE:
                if (config_received != 0u) {
                    Parse_Config(rx_buffer);
                    config_received = 0u;
                    rx_index = 0u;
                    Reset_Metrics();
                    current_state = STATE_READY;

                    sprintf(stringOut,
                            "\r\n[CONFIG] Loaded\r\n"
                            "S: %d | M: %d | L: %d\r\n"
                            "Max Time: %lu s\r\n"
                            "[SYS] Press START button (PB4) to begin\r\n",
                            target_S, target_M, target_L, target_time);
                    UART2_TxString(stringOut);
                }
                else {
                    /* No action */
                }
                break;

            case STATE_READY:
                break;

            case STATE_RUNNING:
                if ((target_time > 0u) && (elapsed_time >= target_time)) {
                    UART2_TxString("\r\n[TIME] Operating time expired!\r\n");
                    current_state = STATE_REPORT;
                    break;
                }
                else {
                    /* No action */
                }

                if ((target_time > 0u) && (elapsed_time != last_printed_sec)) {
                    last_printed_sec = elapsed_time;
                    sprintf(stringOut, "[TIME] Remaining: %lu s\r\n", target_time - elapsed_time);
                    UART2_TxString(stringOut);
                }
                else {
                    /* No action */
                }

                if ((package_active == 0u) && IS_LDR_DARK()) {
                    Enter_Fault(2u);
                }
                else {
                    /* No action */
                }
                break;

            case STATE_WAIT_LDR:
                if (IS_LDR_DARK()) {
                    settle_start_time = msTicks;
                    current_state = STATE_SETTLE;
                    UART2_TxString("[SENSOR] LDR : Package arrived at sorting point.\r\n");
                }
                else if ((msTicks - wait_ldr_start_time) >= LDR_TIMEOUT_MS) {
                    Enter_Fault(1u);
                }
                else {
                    /* No action */
                }
                break;

            case STATE_SETTLE:
                if ((msTicks - settle_start_time) >= SETTLE_TIME_MS) {
                    current_state = STATE_EVALUATE;
                }
                else {
                    /* No action */
                }
                break;

            case STATE_EVALUATE:
            {
                uint16_t q;
                uint16_t a_or_r;

                Process_Evaluate(adc_buffer[1]);
                package_active = 1u;
                route_start_time = msTicks;

                if (current_size == SIZE_S) {
                    q = target_S;
                }
                else if (current_size == SIZE_M) {
                    q = target_M;
                }
                else {
                    q = target_L;
                }

                if (current_decision == DECISION_ACCEPT) {
                    Start_Size_Result_LED(current_size, DECISION_ACCEPT);
                    Servo_Normal(); /* ACCEPT -> เซอโวอยู่นิ่งที่ตำแหน่งปกติ */
                    current_state = STATE_ROUTE_ACCEPT;
                    UART2_TxString("[EVAL] Decision: ACCEPT -> Passing through (Servo Normal).\r\n");

                    if (current_size == SIZE_S) {
                        a_or_r = count_S;
                    }
                    else if (current_size == SIZE_M) {
                        a_or_r = count_M;
                    }
                    else {
                        a_or_r = count_L;
                    }

                    /* +1 = นับชิ้นนี้รวมแล้ว */
                    sprintf(stringOut, "Quota: %d | Accepted: %d\r\n", q, a_or_r + 1u);
                    UART2_TxString(stringOut);
                }
                else {
                    Start_Size_Result_LED(current_size, DECISION_REJECT);
                    Servo_Reject(); /* REJECT -> สั่งเซอโวปัดชิ้นงาน */
                    current_state = STATE_ROUTE_REJECT;
                    UART2_TxString("[EVAL] Decision: REJECT -> Actuating Servo to discard.\r\n");

                    if (current_size == SIZE_S) {
                        a_or_r = reject_S;
                    }
                    else if (current_size == SIZE_M) {
                        a_or_r = reject_M;
                    }
                    else {
                        a_or_r = reject_L;
                    }

                    /* +1 = นับชิ้นนี้รวมแล้ว */
                    sprintf(stringOut, "Quota: %d | Rejected: %d\r\n", q, a_or_r + 1u);
                    UART2_TxString(stringOut);
                }
                break;
            }

            case STATE_ROUTE_ACCEPT:
            case STATE_ROUTE_REJECT:
                if ((msTicks - route_start_time) >= ROUTE_TIME_MS) {
                    ldr_clear_start_time = msTicks;
                    current_state = STATE_WAIT_LDR_CLEAR;
                }
                else {
                    /* No action */
                }
                break;

            case STATE_WAIT_LDR_CLEAR:
                if (!IS_LDR_DARK()) {
                    if (current_decision == DECISION_ACCEPT) {
                        if (current_size == SIZE_S) {
                            count_S++;
                        }
                        else if (current_size == SIZE_M) {
                            count_M++;
                        }
                        else {
                            count_L++;
                        }
                    }
                    else {
                        if (current_size == SIZE_S) {
                            reject_S++;
                        }
                        else if (current_size == SIZE_M) {
                            reject_M++;
                        }
                        else {
                            reject_L++;
                        }
                    }

                    package_active = 0u;
                    Servo_Normal();

                    if ((current_decision == DECISION_ACCEPT) && (Targets_Complete() != 0u)) {
                        current_state = STATE_COMPLETE;
                    }
                    else {
                        current_state = STATE_RUNNING;
                    }
                }
                else if ((msTicks - ldr_clear_start_time) >= LDR_CLEAR_TIMEOUT_MS) {
                    err_stuck++;
                    Enter_Fault(3u);
                }
                else {
                    /* No action */
                }
                break;

            case STATE_PAUSED:
                if ((msTicks - pause_start_time) >= PAUSE_TIMEOUT_MS) {
                    UART2_TxString("\r\n[SYS] Pause timeout reached! Generating report.\r\n");
                    Start_Reset_Flash();
                    current_state = STATE_REPORT;
                }
                else {
                    /* No action */
                }
                break;

            case STATE_FAULT:
                if ((msTicks - fault_start_time) >= FAULT_TIMEOUT_MS) {
                    UART2_TxString("\r\n[SYS] Fault timeout reached! Generating report.\r\n");
                    Start_Reset_Flash();
                    current_state = STATE_REPORT;
                }
                else {
                    /* No action */
                }
                break;

            case STATE_EMERGENCY:
                break;

            case STATE_COMPLETE:
                UART2_TxString("\r\n[SYS] All target package counts reached successfully!\r\n");
                current_state = STATE_REPORT;
                break;

            case STATE_REPORT:
                if (report_printed == 0u) {
                    uint32_t total_target   = (uint32_t)target_S + (uint32_t)target_M + (uint32_t)target_L;
                    uint32_t total_accepted = (uint32_t)count_S + (uint32_t)count_M + (uint32_t)count_L;
                    uint32_t total_rejected = (uint32_t)reject_S + (uint32_t)reject_M + (uint32_t)reject_L;
                    uint32_t total_errors   = (uint32_t)err_ir_fault + (uint32_t)err_ldr_fault + (uint32_t)err_stuck;
                    uint32_t total_input    = total_accepted + total_rejected + total_errors;

                    uint32_t eff_whole = 0u;
                    uint32_t eff_dec = 0u;
                    uint32_t loss_whole = 0u;
                    uint32_t loss_dec = 0u;
                    uint32_t rej_whole = 0u;
                    uint32_t rej_dec = 0u;
                    uint32_t comp_whole = 0u;
                    uint32_t comp_dec = 0u;
                    uint32_t comp_scaled = 0u;

                    if (total_input > 0u) {
                        /* System Efficiency = (Total Accepted / Total Input) * 100 */
                        uint32_t eff_scaled = (total_accepted * PERCENT_SCALE_FACTOR) / total_input;
                        eff_whole = eff_scaled / PERCENT_WHOLE_DIVISOR;
                        eff_dec   = eff_scaled % PERCENT_WHOLE_DIVISOR;

                        /* Reject Rate = (Total Rejected / Total Input) * 100 */
                        uint32_t rej_scaled = (total_rejected * PERCENT_SCALE_FACTOR) / total_input;
                        rej_whole = rej_scaled / PERCENT_WHOLE_DIVISOR;
                        rej_dec   = rej_scaled % PERCENT_WHOLE_DIVISOR;
                    }
                    else {
                        /* No action */
                    }

                    if (total_target > 0u) {
                        /* Target Completion Rate = (Total Accepted / Total Target) * 100 */
                        comp_scaled = (total_accepted * PERCENT_SCALE_FACTOR) / total_target;
                        comp_whole = comp_scaled / PERCENT_WHOLE_DIVISOR;
                        comp_dec   = comp_scaled % PERCENT_WHOLE_DIVISOR;

                        /* System Loss Rate = 100 - [(Total Accepted / Total Target) * 100] */
                        if (comp_scaled <= PERCENT_SCALE_FACTOR) {
                            loss_whole = (PERCENT_SCALE_FACTOR - comp_scaled) / PERCENT_WHOLE_DIVISOR;
                            loss_dec   = (PERCENT_SCALE_FACTOR - comp_scaled) % PERCENT_WHOLE_DIVISOR;
                        }
                        else {
                            loss_whole = 0u;
                            loss_dec = 0u;
                        }
                    }
                    else {
                        /* No action */
                    }

                    sprintf(stringOut,
                            "\r\n========================================\r\n"
                            "SUMMARY REPORT\r\n"
                            "========================================\r\n"
                            "ACCEPTED -> S: %d/%d | M: %d/%d | L: %d/%d\r\n"
                            "REJECTED -> S: %d | M: %d | L: %d\r\n"
                            "ERRORS   -> Object Lost: %d | Unexpected Object: %d | Object Stuck: %d | Total Errors: %lu\r\n"
                            "----------------------------------------\r\n"
                            "TOTAL INPUT  -> %lu\r\n"
                            "TOTAL TARGET -> %lu\r\n"
                            "ELAPSED TIME -> %lu s\r\n"
                            "----------------------------------------\r\n"
                            "TARGET COMPLETION -> %lu.%02lu %%\r\n"
                            "SYSTEM EFFICIENCY -> %lu.%02lu %%\r\n"
                            "REJECT RATE       -> %lu.%02lu %%\r\n"
                            "SYSTEM LOSS RATE  -> %lu.%02lu %%\r\n"
                            "========================================\r\n",
                            count_S, target_S, count_M, target_M, count_L, target_L,
                            reject_S, reject_M, reject_L,
                            err_ir_fault, err_ldr_fault, err_stuck, total_errors,
                            total_input,
                            total_target,
                            elapsed_time,
                            comp_whole, comp_dec,
                            eff_whole, eff_dec,
                            rej_whole, rej_dec,
                            loss_whole, loss_dec);

                    UART2_TxString(stringOut);
                    report_printed = 1u;
                }
                else {
                    /* No action */
                }

                if (reset_flash_active == 0u) {
                    Reset_To_Idle();
                }
                else {
                    /* No action */
                }
                break;

            default:
                break;
        }
    }
}

/*==============================================================================
 * IR / Pot Helpers
 *============================================================================*/
static void Process_IR_Event(void)
{
    if ((current_state != STATE_RUNNING) || (package_active != 0u)) {
        return;
    }
    else {
        /* No action */
    }

    package_active = 1u;
    wait_ldr_start_time = msTicks;
    wait_led_hold_until = msTicks + LED_HOLD_TIME_MS;
    current_state = STATE_WAIT_LDR;
    UART2_TxString("[SENSOR] IR : Entry detected. Waiting for LDR sensor.\r\n");
}

static PackageSize Get_Pot_Size(uint16_t pot_val)
{
    PackageSize result;

    if (pot_val < THRESHOLD_STAGE1) {
        result = SIZE_S;
    }
    else if (pot_val < THRESHOLD_STAGE2) {
        result = SIZE_M;
    }
    else {
        result = SIZE_L;
    }

    return result;
}

static void Process_Evaluate(uint16_t pot_val)
{
    current_size = Get_Pot_Size(pot_val);

    if (current_size == SIZE_S) {
        current_decision = (count_S < target_S) ? DECISION_ACCEPT : DECISION_REJECT;
        UART2_TxString("[EVAL] Size Measured: [ S ]\r\n");
    }
    else if (current_size == SIZE_M) {
        current_decision = (count_M < target_M) ? DECISION_ACCEPT : DECISION_REJECT;
        UART2_TxString("[EVAL] Size Measured: [ M ]\r\n");
    }
    else {
        current_decision = (count_L < target_L) ? DECISION_ACCEPT : DECISION_REJECT;
        UART2_TxString("[EVAL] Size Measured: [ L ]\r\n");
    }
}

/*==============================================================================
 * Servo Control
 *============================================================================*/
static void Servo_Reject(void)
{
    TIM4->CCR3 = SERVO_POS_REJECT;
}

static void Servo_Normal(void)
{
    TIM4->CCR3 = SERVO_POS_NORMAL;
}

/*==============================================================================
 * State & LED Helper Functions
 *============================================================================*/
static uint8_t Targets_Complete(void)
{
    uint8_t result;

    if ((count_S >= target_S) && (count_M >= target_M) && (count_L >= target_L)) {
        result = 1u;
    }
    else {
        result = 0u;
    }

    return result;
}

static void Enter_Fault(uint8_t reason)
{
    package_active = 0u;
    ir_event_pending = 0u;
    last_fault_reason = reason;
    fault_start_time = msTicks;
    resume_state = STATE_RUNNING;

    Servo_Normal();
    Clear_All_LEDs();

    if (reason == 1u) {
        err_ir_fault++;
        UART2_TxString("\r\n[FAULT] IR triggered, but package missed LDR sensor!\r\n");
    }
    else if (reason == 2u) {
        err_ldr_fault++;
        UART2_TxString("\r\n[FAULT] Unexpected LDR trigger without IR trigger!\r\n");
    }
    else {
        UART2_TxString("\r\n[FAULT] Package STUCK at sorting zone!\r\n");
    }

    UART2_TxString("[SYS] System paused. Press PAUSE (PB5) to resume or RESET (PB3).\r\n");
    current_state = STATE_FAULT;
}

static void Resume_From_Fault(void)
{
    fault_start_time = 0u;
    last_printed_sec = TIME_SENTINEL_UNSET;
    current_state = STATE_RUNNING;
    UART2_TxString("\r\n[SYS] Resuming from fault. Sorting reactivated.\r\n");
}

static void Enter_Emergency(void)
{
    package_active = 0u;
    ir_event_pending = 0u;
    Servo_Normal();
    led_pattern = LED_PATTERN_NONE;
    reset_flash_active = 0u;

    Set_Emergency_LEDs();
    current_state = STATE_EMERGENCY;
    UART2_TxString("\r\n[EMERGENCY] EMERGENCY BUTTON PRESSED! All operations halted.\r\n");
}

static void Pause_System(void)
{
    resume_state = current_state;
    pause_start_time = msTicks;
    current_state = STATE_PAUSED;
    UART2_TxString("\r\n[SYS] System PAUSED. Press PAUSE (PB5) to resume or RESET (PB3)\r\n");
}

static void Resume_System(void)
{
    uint32_t paused_ms = msTicks - pause_start_time;

    if (resume_state == STATE_WAIT_LDR) {
        wait_ldr_start_time += paused_ms;
    }
    else if (resume_state == STATE_SETTLE) {
        settle_start_time += paused_ms;
    }
    else if ((resume_state == STATE_ROUTE_ACCEPT) || (resume_state == STATE_ROUTE_REJECT)) {
        route_start_time += paused_ms;
    }
    else if (resume_state == STATE_WAIT_LDR_CLEAR) {
        ldr_clear_start_time += paused_ms;
    }
    else {
        /* No action */
    }

    if (led_pattern != LED_PATTERN_NONE) {
        led_pattern_deadline += paused_ms;
    }
    else {
        /* No action */
    }

    last_printed_sec = TIME_SENTINEL_UNSET;
    current_state = resume_state;
    UART2_TxString("\r\n[SYS] System RESUMED.\r\n");
}

static void Start_Reset_Flash(void)
{
    reset_flash_active = 1u;
    reset_flash_end_time = msTicks + RESET_FLASH_DURATION_MS;
    Set_All_LEDs();
}

static void Request_Reset_Report(void)
{
    package_active = 0u;
    ir_event_pending = 0u;

    if ((current_state == STATE_IDLE) || (current_state == STATE_READY)) {
        Reset_To_Idle();
        return;
    }
    else {
        /* No action */
    }

    Servo_Normal();
    Start_Reset_Flash();
    current_state = STATE_REPORT;
    UART2_TxString("\r\n[SYS] RESET pressed. Finalizing report.\r\n");
}

static void Reset_To_Idle(void)
{
    Servo_Normal();
    reset_flash_active = 0u;
    Clear_All_LEDs();
    Reset_Metrics();

    target_S = 0u;
    target_M = 0u;
    target_L = 0u;
    target_time = 0u;

    config_received = 0u;
    rx_index = 0u;
    memset((void *)rx_buffer, 0, sizeof(rx_buffer));

    current_state = STATE_IDLE;

    UART2_TxString("\r\n[SYS] System Reset -> IDLE. Waiting for configuration.\r\n");
    UART2_TxString("Waiting for Configuration from PC (Format : S2,M2,L2,T30)\r\n");
}

static void Reset_Metrics(void)
{
    count_S = 0u;
    count_M = 0u;
    count_L = 0u;
    reject_S = 0u;
    reject_M = 0u;
    reject_L = 0u;
    err_ir_fault = 0u;
    err_ldr_fault = 0u;
    err_stuck = 0u;
    elapsed_time = 0u;
    last_printed_sec = TIME_SENTINEL_UNSET;
    package_active = 0u;
    current_decision = DECISION_ACCEPT;
    current_size = SIZE_S;
    resume_state = STATE_RUNNING;
    last_manual_trigger = (uint32_t)(0u - IR_COOLDOWN_MS);
    last_btn_emergency = 0u;
    pause_start_time = 0u;
    fault_start_time = 0u;
    wait_ldr_start_time = 0u;
    wait_led_hold_until = 0u;
    settle_start_time = 0u;
    route_start_time = 0u;
    ldr_clear_start_time = 0u;
    led_pattern = LED_PATTERN_NONE;
    led_pattern_phase = 0u;
    reset_flash_active = 0u;
    reset_flash_end_time = 0u;
    report_printed = 0u;
    ir_event_pending = 0u;
}

static void Clear_All_LEDs(void)
{
    GPIOA->BSRR = (1u << (LED_STATUS_PIN + 16u)) |
                  (1u << (LED_RED_PIN + 16u)) |
                  (1u << (LED_YELLOW_PIN + 16u));
    GPIOB->BSRR = (1u << (LED_GREEN_PIN + 16u)) |
                  (1u << (LED_WAIT_LDR_PIN + 16u));
    Reject_LED_Off();
}

static void Set_All_LEDs(void)
{
    GPIOA->BSRR = (1u << LED_STATUS_PIN) |
                  (1u << LED_RED_PIN) |
                  (1u << LED_YELLOW_PIN);
    GPIOB->BSRR = (1u << LED_GREEN_PIN);

    GPIOB->BSRR = (1u << (LED_WAIT_LDR_PIN + 16u));
    Reject_LED_Off();
}

static void Set_Emergency_LEDs(void)
{
    Set_All_LEDs();
}

static void Wait_LDR_LED_On(void)
{
    GPIOB->BSRR = (1u << LED_WAIT_LDR_PIN);
}

static void Wait_LDR_LED_Off(void)
{
    GPIOB->BSRR = (1u << (LED_WAIT_LDR_PIN + 16u));
}

static void Reject_LED_On(void)
{
#if REJECT_LED_ACTIVE_LOW
    GPIOB->BSRR = (1u << (LED_REJECT_PIN + 16u));
#else
    GPIOB->BSRR = (1u << LED_REJECT_PIN);
#endif
}

static void Reject_LED_Off(void)
{
#if REJECT_LED_ACTIVE_LOW
    GPIOB->BSRR = (1u << LED_REJECT_PIN);
#else
    GPIOB->BSRR = (1u << (LED_REJECT_PIN + 16u));
#endif
}

static void Clear_Size_LEDs(void)
{
    GPIOA->BSRR = (1u << (LED_RED_PIN + 16u)) |
                  (1u << (LED_YELLOW_PIN + 16u));
    GPIOB->BSRR = (1u << (LED_GREEN_PIN + 16u));
    Reject_LED_Off();
}

static void Set_Size_LED_Output(PackageSize size, uint8_t on)
{
    uint32_t value;

    if (size == SIZE_S) {
        value = (1u << LED_RED_PIN);
    }
    else if (size == SIZE_M) {
        value = (1u << LED_YELLOW_PIN);
    }
    else {
        value = (1u << LED_GREEN_PIN);
    }

    if (on != 0u) {
        if (size == SIZE_L) {
            GPIOB->BSRR = value;
        }
        else {
            GPIOA->BSRR = value;
        }
    }
    else {
        if (size == SIZE_L) {
            GPIOB->BSRR = (value << 16u);
        }
        else {
            GPIOA->BSRR = (value << 16u);
        }
    }
}

static void Start_Size_Result_LED(PackageSize size, PackageDecision decision)
{
    led_pattern = (decision == DECISION_ACCEPT) ? LED_PATTERN_ACCEPT : LED_PATTERN_REJECT;
    led_pattern_size = size;
    led_pattern_phase = 0u;

    Clear_Size_LEDs();
    Set_Size_LED_Output(size, 1u);

    if (decision == DECISION_REJECT) {
        Reject_LED_On();
    }
    else {
        /* No action */
    }

    led_pattern_deadline = msTicks + LED_RESULT_SHOW_MS;
}

static void Update_Size_Result_LED(void)
{
    if (led_pattern == LED_PATTERN_NONE) {
        return;
    }
    else {
        /* No action */
    }

    if ((current_state == STATE_EMERGENCY) || (reset_flash_active != 0u)) {
        return;
    }
    else {
        /* No action */
    }

    if (msTicks < led_pattern_deadline) {
        return;
    }
    else {
        /* No action */
    }

    if (led_pattern == LED_PATTERN_ACCEPT) {
        Set_Size_LED_Output(led_pattern_size, 0u);
        Reject_LED_Off();
        led_pattern = LED_PATTERN_NONE;
        return;
    }
    else {
        /* No action */
    }

    led_pattern_phase++;

    if (led_pattern_phase == 1u) {
        Set_Size_LED_Output(led_pattern_size, 0u);
        led_pattern_deadline = msTicks + LED_RESULT_BLINK_OFF_MS;
    }
    else if (led_pattern_phase == 2u) {
        Set_Size_LED_Output(led_pattern_size, 1u);
        led_pattern_deadline = msTicks + LED_RESULT_BLINK_ON_MS;
    }
    else {
        Set_Size_LED_Output(led_pattern_size, 0u);
        Reject_LED_Off();
        led_pattern = LED_PATTERN_NONE;
    }
}

static void Parse_Config(char *str)
{
    target_S = 0u;
    target_M = 0u;
    target_L = 0u;
    target_time = 0u;

    char *ptr = strchr(str, 'S');
    if (ptr != NULL) {
        sscanf(ptr, "S%hu,M%hu,L%hu,T%lu",
               &target_S, &target_M, &target_L, &target_time);
    }
    else {
        /* No action */
    }
}

/*==============================================================================
 * UART TX Ring Buffer
 *============================================================================*/
static void UART2_TxString(const char strOut[])
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

/*==============================================================================
 * Hardware Initialization
 *============================================================================*/
void System_Init(void)
{
    SysTick_Config(16000000u / 1000u);

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_DMA2EN);
    RCC->APB1ENR |= (RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM4EN);
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN);

    /* LEDs Config */
    GPIOA->MODER &= ~((3u << (LED_STATUS_PIN * 2u)) |
                       (3u << (LED_RED_PIN * 2u)) |
                       (3u << (LED_YELLOW_PIN * 2u)));
    GPIOA->MODER |= ((1u << (LED_STATUS_PIN * 2u)) |
                     (1u << (LED_RED_PIN * 2u)) |
                     (1u << (LED_YELLOW_PIN * 2u)));

    GPIOB->MODER &= ~((3u << (LED_GREEN_PIN * 2u)) |
                       (3u << (LED_REJECT_PIN * 2u)) |
                       (3u << (LED_WAIT_LDR_PIN * 2u)));
    GPIOB->MODER |= ((1u << (LED_GREEN_PIN * 2u)) |
                     (1u << (LED_REJECT_PIN * 2u)) |
                     (1u << (LED_WAIT_LDR_PIN * 2u)));
    GPIOB->OTYPER &= ~((1u << LED_REJECT_PIN) | (1u << LED_WAIT_LDR_PIN));
    Clear_All_LEDs();

    /* Servo TIM4 PWM on PB8 */
    GPIOB->MODER &= ~(3u << (SERVO_PIN * 2u));
    GPIOB->MODER |=  (2u << (SERVO_PIN * 2u));
    GPIOB->AFR[1] &= ~(0xFu << ((SERVO_PIN - 8u) * 4u));
    GPIOB->AFR[1] |=  (2u << ((SERVO_PIN - 8u) * 4u));

    TIM4->PSC = 160u - 1u;
    TIM4->ARR = 2000u - 1u;
    TIM4->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM4->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->CR1 |= TIM_CR1_CEN;
    TIM4->CCR3 = SERVO_POS_NORMAL;

    /* Analog inputs */
    GPIOA->MODER |= (3u << (LIGHT_SENSOR_PIN * 2u)) |
                    (3u << (POT_PIN * 2u));

    /* Buttons PB3, PB4, PB5 */
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 |
                      GPIO_MODER_MODER4 |
                      GPIO_MODER_MODER5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 |
                      GPIO_PUPDR_PUPD4 |
                      GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (1u << GPIO_PUPDR_PUPD3_Pos) |
                    (1u << GPIO_PUPDR_PUPD4_Pos) |
                    (1u << GPIO_PUPDR_PUPD5_Pos);

    /* Emergency PA10 */
    GPIOA->MODER &= ~(3u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->MODER |=  (2u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->PUPDR &= ~(3u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->PUPDR |=  (1u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->AFR[1] &= ~(0xFu << ((EMERGENCY_BUTTON_PIN - 8u) * 4u));
    GPIOA->AFR[1] |=  (1u << ((EMERGENCY_BUTTON_PIN - 8u) * 4u));

    TIM1->PSC = 16000u - 1u;
    TIM1->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM1->CCMR2 |= TIM_CCMR2_CC3S_0;
    TIM1->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E;
    TIM1->DIER |= TIM_DIER_CC3IE;
    TIM1->SR &= ~TIM_SR_CC3IF;
    TIM1->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    /* IR PC10 */
    GPIOC->MODER &= ~(3u << (IR_PIN * 2u));
    GPIOC->PUPDR &= ~(3u << (IR_PIN * 2u));
    GPIOC->PUPDR |=  (1u << (IR_PIN * 2u));

    /* EXTI Interrupts Setup */
    SYSCFG->EXTICR[0] &= ~(0xFu << 12u);
    SYSCFG->EXTICR[0] |= (0x1u << 12u); /* EXTI3 PB3 */

    SYSCFG->EXTICR[1] &= ~((0xFu << 0u) | (0xFu << 4u));
    SYSCFG->EXTICR[1] |= ((0x1u << 0u) | (0x1u << 4u)); /* EXTI4 PB4, EXTI5 PB5 */

    SYSCFG->EXTICR[2] &= ~(0xFu << 8u);
    SYSCFG->EXTICR[2] |= (0x2u << 8u); /* EXTI10 PC10 */

    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR10);
    EXTI->FTSR |= (EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5 | EXTI_FTSR_TR10);
    EXTI->RTSR &= ~EXTI_RTSR_TR10;
    EXTI->PR |= (EXTI_PR_PR3 | EXTI_PR_PR4 | EXTI_PR_PR5 | EXTI_PR_PR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* UART2 Setup */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= ((2u << GPIO_MODER_MODER2_Pos) | (2u << GPIO_MODER_MODER3_Pos));
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= ((7u << GPIO_AFRL_AFSEL2_Pos) | (7u << GPIO_AFRL_AFSEL3_Pos));

    USART2->BRR = 0x8Bu;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE);
    NVIC_EnableIRQ(USART2_IRQn);

    /* ADC1 & DMA Setup */
    DMA2_Stream0->CR = 0u;
    while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0u) {
        /* รอจนกว่า stream จะปิดสนิทก่อนตั้งค่าใหม่ */
    }

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;
    DMA2_Stream0->NDTR = ADC_CHANNEL_COUNT;
    DMA2_Stream0->CR |= (0u << 25u) | (1u << 13u) | (1u << 11u) | (1u << 10u) | (1u << 8u);
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SQR1 |= (1u << 20u);
    ADC1->SQR3 = (LIGHT_SENSOR_PIN << 0u) | (POT_PIN << 5u);
    ADC1->SMPR2 |= (7u << (LIGHT_SENSOR_PIN * 3u)) | (7u << (POT_PIN * 3u));
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

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