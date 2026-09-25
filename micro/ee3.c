/*******************************************************************************
 * File Name    : main2_manual_test.c
 * Description  : Conveyor Simulation - Merged IR/Fault/Emergency System
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *
 * Current hardware:
 *   PC10 = IR input (EXTI10)
 *   PA10 = External Emergency Stop button module (TIM1_CH3 interrupt)
 *   LDR  = PA1 / ADC channel 1
 *   Pot  = PA4 / ADC channel 4
 *   Servo is NOT connected yet. Reject action is simulated by LED_REJECT_PIN.
 *
 * Current behavior:
 *   FAULT acts like an automatic pause: PAUSE resumes, RESET reports then IDLE;
 *   if FAULT lasts too long, system goes to REPORT automatically.
 *   Emergency is triggered by the external Emergency button on PA10.
 *
 * IR/LDR detection:
 *   IR uses the same EXTI/falling-edge detection logic as the latest friend version.
 *   LDR uses the same threshold logic (ADC > 2000 = detected/dark).
 *   PA10 uses TIM1_CH3 input-capture interrupt so PC10 can keep EXTI10.
 *   External Emergency button module: VCC=3.3V, OUT=PA10, GND=GND.
 *   Replace Servo_SimReject() with real Servo PWM control.
 *   Timing: IR->LDR wait = 5 s; manual PAUSE/Fault timeout = 15 s.
 *******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* --- Analog inputs --- */
#define LIGHT_SENSOR_PIN 1      // PA1
#define POT_PIN          4      // PA4

/* --- LEDs --- */
#define LED_STATUS_PIN   5      // PA5
#define LED_RED_PIN      6      // PA6 : S
#define LED_YELLOW_PIN   7      // PA7 : M
#define LED_GREEN_PIN    6      // PB6 : L
#define LED_REJECT_PIN   8      // PB8 : reject / servo simulation
#define LED_WAIT_LDR_PIN 9      // PB9 : ON while IR detected and waiting for LDR

/* --- Manual test input / future IR input --- */
#define IR_PIN               10  // PC10: IR sensor output (EXTI10)
#define EMERGENCY_BUTTON_PIN 10  // PA10: external Emergency button OUT (TIM1_CH3)

/* --- Thresholds --- */
#define THRESHOLD_STAGE1      1365
#define THRESHOLD_STAGE2      2730
#define LDR_THRESHOLD_DARK    2000
#define LDR_DARK_WHEN_HIGH     1

#if LDR_DARK_WHEN_HIGH
#define IS_LDR_DARK()          (adc_buffer[0] > LDR_THRESHOLD_DARK)
#else
#define IS_LDR_DARK()          (adc_buffer[0] < LDR_THRESHOLD_DARK)
#endif

/* --- Timing --- */
#define BUTTON_DEBOUNCE_MS    300
#define IR_COOLDOWN_MS        500
#define LDR_TIMEOUT_MS        5000      // after IR, wait up to 5 s for LDR
#define LDR_CLEAR_TIMEOUT_MS  3000
#define SETTLE_TIME_MS        300
#define ROUTE_TIME_MS         1000
#define PAUSE_TIMEOUT_MS      10000      // manual pause timeout = 15 s
#define FAULT_TIMEOUT_MS      10000      // fault timeout = 15 s
#define LED_HOLD_TIME_MS      500

/* --- UART TX ring buffer --- */
#define TX_BUFFER_SIZE 512

/* --- FSM --- */
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

/* --- Package information --- */
typedef enum {
    SIZE_S,
    SIZE_M,
    SIZE_L
} PackageSize;

typedef enum {
    DECISION_ACCEPT,
    DECISION_REJECT
} PackageDecision;

/* --- Global variables --- */
volatile SystemState current_state = STATE_IDLE;
volatile SystemState resume_state = STATE_RUNNING;
volatile PackageSize current_size = SIZE_S;
volatile PackageDecision current_decision = DECISION_ACCEPT;

volatile uint32_t msTicks = 0;
volatile uint16_t adc_buffer[2] = {0, 0};

volatile uint16_t target_S = 0, target_M = 0, target_L = 0;
volatile uint32_t target_time = 0;

volatile uint16_t count_S = 0, count_M = 0, count_L = 0;
volatile uint16_t reject_S = 0, reject_M = 0, reject_L = 0;
volatile uint16_t err_ir_fault = 0;   /* IR saw object, but LDR did not confirm */
volatile uint16_t err_ldr_fault = 0;  /* LDR saw object without previous IR */
volatile uint16_t err_stuck = 0;
volatile uint8_t last_fault_reason = 0;

volatile uint32_t elapsed_time = 0;
volatile uint32_t last_printed_sec = 0xFFFFFFFF;

volatile uint32_t wait_ldr_start_time = 0;
volatile uint32_t settle_start_time = 0;
volatile uint32_t route_start_time = 0;
volatile uint32_t ldr_clear_start_time = 0;
volatile uint32_t pause_start_time = 0;
volatile uint32_t fault_start_time = 0;

volatile uint32_t last_btn_start = 0;
volatile uint32_t last_btn_pause = 0;
volatile uint32_t last_btn_reset = 0;
volatile uint32_t last_btn_emergency = 0;
volatile uint32_t last_manual_trigger = 0;
volatile uint8_t ir_event_pending = 0;

volatile uint32_t size_led_off_time = 0;
volatile uint8_t size_led_active = 0;

/* Non-blocking S/M/L result LED pattern */
typedef enum {
    LED_PATTERN_NONE,
    LED_PATTERN_ACCEPT,
    LED_PATTERN_REJECT
} LedPatternType;

volatile LedPatternType led_pattern = LED_PATTERN_NONE;
volatile PackageSize led_pattern_size = SIZE_S;
volatile uint8_t led_pattern_phase = 0;
volatile uint32_t led_pattern_deadline = 0;

/* Reset indication: all LEDs ON briefly, then OFF */
volatile uint8_t reset_flash_active = 0;
volatile uint32_t reset_flash_end_time = 0;
volatile uint8_t report_printed = 0;

volatile uint8_t package_active = 0;
volatile uint8_t config_received = 0;
volatile uint8_t rx_index = 0;
char rx_buffer[50];

char tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_head = 0;
volatile uint16_t tx_tail = 0;

char stringOut[300];

/* --- Prototypes --- */
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
static void Parse_Config(char* str);
static uint8_t Targets_Complete(void);
static void Enter_Fault(uint8_t reason);
static void Resume_From_Fault(void);
static void Enter_Emergency(void);
static void Servo_SimAccept(void);
static void Servo_SimReject(void);
static void Servo_SimNormal(void);
static void Pause_System(void);
static void Resume_System(void);
static void Reset_To_Idle(void);
static void Process_IR_Event(void);
static PackageSize Get_Pot_Size(uint16_t pot_val);

/*==============================================================================
 * Main
 *============================================================================*/
int main(void)
{
    System_Init();

    UART2_TxString("\r\n=== Conveyor Simulation ===\r\n");
    UART2_TxString("IR = PC10. External Emergency button = OUT -> PA10.\r\n");
    UART2_TxString("Waiting for PC config...\r\n");

    while (1)
    {
        /* Reset indication: all LEDs ON briefly, then OFF. */
        if (reset_flash_active) {
            Set_All_LEDs();
            if (msTicks >= reset_flash_end_time) {
                reset_flash_active = 0;
                Clear_All_LEDs();
            }
        }

        /* Result LEDs are independent from the status LED. */
        Update_Size_Result_LED();

        /* Main consumes the IR event raised by the PC10 EXTI interrupt. */
        if (ir_event_pending) {
            ir_event_pending = 0;
            Process_IR_Event();
        }

            /* Status LED */
        if (!reset_flash_active) {
            if (current_state == STATE_EMERGENCY) {
                /* Emergency: every LED stays ON while in EMERGENCY. */
                Set_All_LEDs();
            }
            else if (current_state == STATE_WAIT_LDR) {
                /* IR detected: PA5 remains ON and PB9 indicates waiting for LDR. */
                GPIOA->BSRR = (1 << LED_STATUS_PIN);
                Wait_LDR_LED_On();
            }
            else {
                /* PB9 is only for the IR -> LDR waiting state. */
                Wait_LDR_LED_Off();

                if (current_state == STATE_RUNNING ||
                    current_state == STATE_SETTLE ||
                    current_state == STATE_EVALUATE ||
                    current_state == STATE_ROUTE_ACCEPT ||
                    current_state == STATE_ROUTE_REJECT ||
                    current_state == STATE_WAIT_LDR_CLEAR) {
                    /* Running: PA5 stays ON. */
                    GPIOA->BSRR = (1 << LED_STATUS_PIN);
                }
                else if (current_state == STATE_PAUSED || current_state == STATE_FAULT) {
                    /* Pause/Fault: PA5 blinks. */
                    if ((msTicks / 250) % 2)
                        GPIOA->BSRR = (1 << LED_STATUS_PIN);
                    else
                        GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16));
                }
                else {
                    GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16));
                }
            }
        }

        switch (current_state)
        {
            case STATE_IDLE:
                if (config_received) {
                    Parse_Config(rx_buffer);
                    config_received = 0;
                    Reset_Metrics();
                    current_state = STATE_READY;

                    sprintf(stringOut,
                            "Config Loaded -> S:%d, M:%d, L:%d, Time:%lus\r\nPress START (PB4)...\r\n",
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

                if (target_time > 0 && elapsed_time != last_printed_sec) {
                    last_printed_sec = elapsed_time;
                    sprintf(stringOut,
                            "[TIME REMAINING: %lu s]\r\n",
                            target_time - elapsed_time);
                    UART2_TxString(stringOut);
                }

                /*
                 * No package should reach LDR before the manual/IR input.
                 * This is the "LDR detected without previous IR" fault.
                 */
                if (!package_active && IS_LDR_DARK()) {
                    Enter_Fault(2);
                }
                break;

            case STATE_WAIT_LDR:
                if (IS_LDR_DARK()) {
                    settle_start_time = msTicks;
                    current_state = STATE_SETTLE;
                    UART2_TxString("-> [LDR] Package confirmed at sorting point.\r\n");
                }
                else if ((msTicks - wait_ldr_start_time) >= LDR_TIMEOUT_MS) {
                    Enter_Fault(1);
                }
                break;

            case STATE_SETTLE:
                if ((msTicks - settle_start_time) >= SETTLE_TIME_MS) {
                    current_state = STATE_EVALUATE;
                }
                break;

            case STATE_EVALUATE:
                Process_Evaluate(adc_buffer[1]);
                package_active = 1;
                route_start_time = msTicks;

                if (current_decision == DECISION_ACCEPT) {
                    Start_Size_Result_LED(current_size, DECISION_ACCEPT);
                    Servo_SimAccept();
                    current_state = STATE_ROUTE_ACCEPT;
                    UART2_TxString("-> [ACCEPT] Package is moving to its size conveyor.\r\n");
                }
                else {
                    Start_Size_Result_LED(current_size, DECISION_REJECT);
                    Servo_SimReject();
                    current_state = STATE_ROUTE_REJECT;
                    UART2_TxString("-> [REJECT] Servo action simulated; size LED blinks twice.\r\n");
                }
                break;

            case STATE_ROUTE_ACCEPT:
                if ((msTicks - route_start_time) >= ROUTE_TIME_MS) {
                    ldr_clear_start_time = msTicks;
                    current_state = STATE_WAIT_LDR_CLEAR;
                }
                break;

            case STATE_ROUTE_REJECT:
                if ((msTicks - route_start_time) >= ROUTE_TIME_MS) {
                    ldr_clear_start_time = msTicks;
                    current_state = STATE_WAIT_LDR_CLEAR;
                }
                break;

            case STATE_WAIT_LDR_CLEAR:
                if (!IS_LDR_DARK()) {
                    if (current_decision == DECISION_ACCEPT) {
                        if (current_size == SIZE_S) count_S++;
                        else if (current_size == SIZE_M) count_M++;
                        else count_L++;
                    }
                    else {
                        if (current_size == SIZE_S) reject_S++;
                        else if (current_size == SIZE_M) reject_M++;
                        else reject_L++;
                    }

                    package_active = 0;
                    Servo_SimNormal();
                    Clear_All_LEDs();

                    if (current_decision == DECISION_ACCEPT && Targets_Complete()) {
                        current_state = STATE_COMPLETE;
                    }
                    else {
                        current_state = STATE_RUNNING;
                    }
                }
                else if ((msTicks - ldr_clear_start_time) >= LDR_CLEAR_TIMEOUT_MS) {
                    err_stuck++;
                    Enter_Fault(3);
                }
                break;

            case STATE_PAUSED:
                if ((msTicks - pause_start_time) >= PAUSE_TIMEOUT_MS) {
                    UART2_TxString("\r\n[PAUSE TIMEOUT] Going to report before reset.\r\n");
                    Start_Reset_Flash();
                    current_state = STATE_REPORT;
                }
                break;

            case STATE_FAULT:
                /* FAULT behaves like an automatic pause.
                 * Resume with PAUSE button, or let the timeout go to REPORT.
                 */
                if ((msTicks - fault_start_time) >= FAULT_TIMEOUT_MS) {
                    UART2_TxString("\r\n[FAULT TIMEOUT] Fault lasted too long. Going to report before reset.\r\n");
                    Start_Reset_Flash();
                    current_state = STATE_REPORT;
                }
                break;

            case STATE_EMERGENCY:
                /* Emergency stays stopped until RESET is pressed. */
                break;

            case STATE_COMPLETE:
                UART2_TxString("\r\n[TARGET COMPLETE]\r\n");
                current_state = STATE_REPORT;
                break;

            case STATE_REPORT:
                if (!report_printed) {
                    sprintf(stringOut,
                            "\r\n=== TEST REPORT ===\r\n"
                            "ACCEPTED -> S:%d/%d, M:%d/%d, L:%d/%d\r\n"
                            "REJECTED -> S:%d, M:%d, L:%d\r\n"
                            "ERRORS   -> IR Fault:%d, LDR Fault:%d, Stuck:%d\r\n"
                            "ELAPSED TIME -> %lu s\r\n"
                            "===================\r\n",
                            count_S, target_S, count_M, target_M, count_L, target_L,
                            reject_S, reject_M, reject_L,
                            err_ir_fault, err_ldr_fault, err_stuck,
                            elapsed_time);
                    UART2_TxString(stringOut);
                    report_printed = 1;
                }

                /* Keep the reset flash visible briefly before clearing the run. */
                if (!reset_flash_active) {
                    Reset_To_Idle();
                }
                break;
        }
    }
}

/*==============================================================================
 * IR / Pot helpers
 *============================================================================*/
static void Process_IR_Event(void)
{
    if (current_state != STATE_RUNNING || package_active) return;

    package_active = 1;
    wait_ldr_start_time = msTicks;
    current_state = STATE_WAIT_LDR;
    UART2_TxString("-> [IR] Package input detected at PC10. Waiting for LDR confirmation...\r\n");
}

static PackageSize Get_Pot_Size(uint16_t pot_val)
{
    if (pot_val < THRESHOLD_STAGE1) return SIZE_S;
    if (pot_val < THRESHOLD_STAGE2) return SIZE_M;
    return SIZE_L;
}

/*==============================================================================
 * Package evaluation
 *============================================================================*/
static void Process_Evaluate(uint16_t pot_val)
{
    current_size = Get_Pot_Size(pot_val);

    if (current_size == SIZE_S) {
        current_decision = (count_S < target_S) ? DECISION_ACCEPT : DECISION_REJECT;
    }
    else if (current_size == SIZE_M) {
        current_decision = (count_M < target_M) ? DECISION_ACCEPT : DECISION_REJECT;
    }
    else {
        current_decision = (count_L < target_L) ? DECISION_ACCEPT : DECISION_REJECT;
    }

    if (current_size == SIZE_S) {
        UART2_TxString("-> Size S detected.\r\n");
    }
    else if (current_size == SIZE_M) {
        UART2_TxString("-> Size M detected.\r\n");
    }
    else {
        UART2_TxString("-> Size L detected.\r\n");
    }
}

/*==============================================================================
 * Servo placeholder
 *============================================================================*/
static void Servo_SimAccept(void)
{
    /* No real servo yet. Accept = servo stays in normal position. */
}

static void Servo_SimReject(void)
{
    /* No real servo yet. The S/M/L LED pattern represents the reject action. */
}

static void Servo_SimNormal(void)
{
    /* Future real servo will return to its normal position here. */
}

/*==============================================================================
 * State / utility helpers
 *============================================================================*/
static uint8_t Targets_Complete(void)
{
    return (count_S >= target_S &&
            count_M >= target_M &&
            count_L >= target_L);
}

static void Enter_Fault(uint8_t reason)
{
    package_active = 0;
    last_fault_reason = reason;
    fault_start_time = msTicks;
    resume_state = STATE_RUNNING;

    Servo_SimNormal();
    Clear_All_LEDs();

    if (reason == 1) {
        err_ir_fault++;
        UART2_TxString("\r\n[FAULT][IR] Object detected by IR, but LDR did not confirm within timeout.\r\n");
    }
    else if (reason == 2) {
        err_ldr_fault++;
        UART2_TxString("\r\n[FAULT][LDR] LDR detected an object without a preceding IR detection.\r\n");
    }
    else {
        UART2_TxString("\r\n[FAULT] OBJECT STUCK: package did not leave LDR point.\r\n");
    }

    UART2_TxString("[FAULT] System paused automatically. Press PAUSE to resume or RESET for report.\r\n");
    current_state = STATE_FAULT;
}

static void Resume_From_Fault(void)
{
    fault_start_time = 0;
    last_printed_sec = 0xFFFFFFFF;
    current_state = STATE_RUNNING;
    UART2_TxString("\r\n[FAULT RESUMED] System back to RUNNING. Waiting for next package...\r\n");
}

static void Enter_Emergency(void)
{
    package_active = 0;
    Servo_SimNormal();
    led_pattern = LED_PATTERN_NONE;
    reset_flash_active = 0;
    Set_All_LEDs();
    current_state = STATE_EMERGENCY;
    UART2_TxString("\r\n[EMERGENCY] Emergency button pressed. System stopped. Press RESET after fixing the problem.\r\n");
}

static void Pause_System(void)
{
    resume_state = current_state;
    pause_start_time = msTicks;
    current_state = STATE_PAUSED;
    UART2_TxString("\r\n[PAUSED]\r\n");
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
    else if (resume_state == STATE_ROUTE_ACCEPT || resume_state == STATE_ROUTE_REJECT) {
        route_start_time += paused_ms;
    }
    else if (resume_state == STATE_WAIT_LDR_CLEAR) {
        ldr_clear_start_time += paused_ms;
    }

    if (led_pattern != LED_PATTERN_NONE) {
        led_pattern_deadline += paused_ms;
    }

    last_printed_sec = 0xFFFFFFFF;
    current_state = resume_state;
    UART2_TxString("\r\n[RESUMED]\r\n");
}

static void Start_Reset_Flash(void)
{
    reset_flash_active = 1;
    reset_flash_end_time = msTicks + 250;
    Set_All_LEDs();
}

static void Request_Reset_Report(void)
{
    /* During an active run/fault/emergency, show the current report first. */
    if (current_state == STATE_IDLE || current_state == STATE_READY) {
        Reset_To_Idle();
        return;
    }

    Servo_SimNormal();
    Start_Reset_Flash();
    current_state = STATE_REPORT;
    UART2_TxString("\r\n[RESET] Current run will be reported, then system returns to IDLE.\r\n");
}

static void Reset_To_Idle(void)
{
    Servo_SimNormal();
    reset_flash_active = 0;
    Clear_All_LEDs();
    Reset_Metrics();
    target_S = target_M = target_L = 0;
    target_time = 0;
    current_state = STATE_IDLE;
    UART2_TxString("\r\n[RESET] System -> IDLE\r\nWaiting for new config...\r\n");
}

static void Reset_Metrics(void)
{
    count_S = count_M = count_L = 0;
    reject_S = reject_M = reject_L = 0;
    err_ir_fault = 0;
    err_ldr_fault = 0;
    err_stuck = 0;
    elapsed_time = 0;
    last_printed_sec = 0xFFFFFFFF;
    package_active = 0;
    current_decision = DECISION_ACCEPT;
    current_size = SIZE_S;
    resume_state = STATE_RUNNING;
    last_manual_trigger = (uint32_t)(0 - IR_COOLDOWN_MS);
    last_btn_emergency = 0;
    pause_start_time = 0;
    fault_start_time = 0;
    wait_ldr_start_time = 0;
    settle_start_time = 0;
    route_start_time = 0;
    ldr_clear_start_time = 0;
    led_pattern = LED_PATTERN_NONE;
    led_pattern_phase = 0;
    reset_flash_active = 0;
    reset_flash_end_time = 0;
    report_printed = 0;
    ir_event_pending = 0;
}

static void Clear_All_LEDs(void)
{
    GPIOA->BSRR = (1 << (LED_STATUS_PIN + 16)) |
                  (1 << (LED_RED_PIN + 16)) |
                  (1 << (LED_YELLOW_PIN + 16));
    GPIOB->BSRR = (1 << (LED_GREEN_PIN + 16)) |
                  (1 << (LED_REJECT_PIN + 16)) |
                  (1 << (LED_WAIT_LDR_PIN + 16));
}

static void Set_All_LEDs(void)
{
    GPIOA->BSRR = (1 << LED_STATUS_PIN) |
                  (1 << LED_RED_PIN) |
                  (1 << LED_YELLOW_PIN);
    GPIOB->BSRR = (1 << LED_GREEN_PIN) |
                  (1 << LED_REJECT_PIN) |
                  (1 << LED_WAIT_LDR_PIN);
}

static void Wait_LDR_LED_On(void)
{
    GPIOB->BSRR = (1 << LED_WAIT_LDR_PIN);
}

static void Wait_LDR_LED_Off(void)
{
    GPIOB->BSRR = (1 << (LED_WAIT_LDR_PIN + 16));
}

static void Reject_LED_On(void)
{
#if REJECT_LED_ACTIVE_LOW
    GPIOB->BSRR = (1 << (LED_REJECT_PIN + 16));
#else
    GPIOB->BSRR = (1 << LED_REJECT_PIN);
#endif
}

static void Reject_LED_Off(void)
{
#if REJECT_LED_ACTIVE_LOW
    GPIOB->BSRR = (1 << LED_REJECT_PIN);
#else
    GPIOB->BSRR = (1 << (LED_REJECT_PIN + 16));
#endif
}

static void Clear_Size_LEDs(void)
{
    GPIOA->BSRR = (1 << (LED_RED_PIN + 16)) |
                  (1 << (LED_YELLOW_PIN + 16));
    GPIOB->BSRR = (1 << (LED_GREEN_PIN + 16));
    Reject_LED_Off();
}

static void Set_Size_LED_Output(PackageSize size, uint8_t on)
{
    uint32_t value = 0;

    if (size == SIZE_S) value = (1u << LED_RED_PIN);
    else if (size == SIZE_M) value = (1u << LED_YELLOW_PIN);
    else value = (1u << LED_GREEN_PIN);

    if (on) {
        if (size == SIZE_L) GPIOB->BSRR = value;
        else GPIOA->BSRR = value;
    }
    else {
        if (size == SIZE_L) GPIOB->BSRR = (value << 16);
        else GPIOA->BSRR = (value << 16);
    }
}

static void Start_Size_Result_LED(PackageSize size, PackageDecision decision)
{
    led_pattern = (decision == DECISION_ACCEPT) ? LED_PATTERN_ACCEPT : LED_PATTERN_REJECT;
    led_pattern_size = size;
    led_pattern_phase = 0;

    Clear_Size_LEDs();
    Set_Size_LED_Output(size, 1);

    /* Reject uses PB8 while the two-blink reject indication is active. */
    if (decision == DECISION_REJECT) {
        Reject_LED_On();
    }

    /* Accept: 1 flash (200 ms ON, then OFF).
       Reject: 2 blinks (200 ms ON/OFF, 200 ms ON/OFF). */
    led_pattern_deadline = msTicks + 200;
}

static void Update_Size_Result_LED(void)
{
    if (led_pattern == LED_PATTERN_NONE) return;
    if (current_state == STATE_EMERGENCY || reset_flash_active) return;
    if (msTicks < led_pattern_deadline) return;

    if (led_pattern == LED_PATTERN_ACCEPT) {
        Set_Size_LED_Output(led_pattern_size, 0);
        Reject_LED_Off();
        led_pattern = LED_PATTERN_NONE;
        return;
    }

    /* Reject pattern phases: ON -> OFF -> ON -> OFF */
    led_pattern_phase++;

    if (led_pattern_phase == 1) {
        Set_Size_LED_Output(led_pattern_size, 0);
        led_pattern_deadline = msTicks + 150;
    }
    else if (led_pattern_phase == 2) {
        Set_Size_LED_Output(led_pattern_size, 1);
        led_pattern_deadline = msTicks + 200;
    }
    else {
        Set_Size_LED_Output(led_pattern_size, 0);
        Reject_LED_Off();
        led_pattern = LED_PATTERN_NONE;
    }
}

static void Set_Emergency_LEDs(void)
{
    Set_All_LEDs();
}

static void Parse_Config(char* str)
{
    target_S = target_M = target_L = target_time = 0;

    char* ptr = strchr(str, 'S');
    if (ptr != NULL) {
        sscanf(ptr, "S%hu,M%hu,L%hu,T%lu",
               &target_S, &target_M, &target_L, &target_time);
    }
}

/*==============================================================================
 * UART TX ring buffer
 *============================================================================*/
static void UART2_TxString(const char strOut[])
{
    for (uint16_t idx = 0; strOut[idx] != '\0'; idx++) {
        uint16_t next_head = (uint16_t)((tx_head + 1) % TX_BUFFER_SIZE);

        if (next_head == tx_tail) {
            break;
        }

        tx_buffer[tx_head] = strOut[idx];
        tx_head = next_head;
    }

    USART2->CR1 |= USART_CR1_TXEIE;
}

/*==============================================================================
 * Hardware init
 *============================================================================*/
void System_Init(void)
{
    SysTick_Config(16000000 / 1000);

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_DMA2EN);
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN);

    /* LEDs */
    GPIOA->MODER &= ~((3 << (LED_STATUS_PIN * 2)) |
                       (3 << (LED_RED_PIN * 2)) |
                       (3 << (LED_YELLOW_PIN * 2)));
    GPIOA->MODER |= ((1 << (LED_STATUS_PIN * 2)) |
                     (1 << (LED_RED_PIN * 2)) |
                     (1 << (LED_YELLOW_PIN * 2)));

    GPIOB->MODER &= ~((3 << (LED_GREEN_PIN * 2)) |
                       (3 << (LED_REJECT_PIN * 2)) |
                       (3 << (LED_WAIT_LDR_PIN * 2)));
    GPIOB->MODER |= ((1 << (LED_GREEN_PIN * 2)) |
                     (1 << (LED_REJECT_PIN * 2)) |
                     (1 << (LED_WAIT_LDR_PIN * 2)));
    GPIOB->OTYPER &= ~((1 << LED_REJECT_PIN) | (1 << LED_WAIT_LDR_PIN));
    Clear_All_LEDs();

    /* Analog inputs */
    GPIOA->MODER |= (3 << (LIGHT_SENSOR_PIN * 2)) |
                    (3 << (POT_PIN * 2));

    /* Existing buttons: PB3 Reset, PB4 Start, PB5 Pause */
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 |
                      GPIO_MODER_MODER4 |
                      GPIO_MODER_MODER5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 |
                      GPIO_PUPDR_PUPD4 |
                      GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD3_Pos) |
                    (1 << GPIO_PUPDR_PUPD4_Pos) |
                    (1 << GPIO_PUPDR_PUPD5_Pos);

    /* External Emergency button module on PA10; use TIM1_CH3 input-capture interrupt. */
    GPIOA->MODER &= ~(3 << (EMERGENCY_BUTTON_PIN * 2));
    GPIOA->MODER |=  (2 << (EMERGENCY_BUTTON_PIN * 2));
    GPIOA->PUPDR &= ~(3 << (EMERGENCY_BUTTON_PIN * 2));
    GPIOA->PUPDR |=  (1 << (EMERGENCY_BUTTON_PIN * 2));
    GPIOA->AFR[1] &= ~(0xF << ((EMERGENCY_BUTTON_PIN - 8) * 4));
    GPIOA->AFR[1] |=  (1 << ((EMERGENCY_BUTTON_PIN - 8) * 4)); // AF1 = TIM1_CH3

    TIM1->PSC = 16000 - 1;       // 1 ms timer tick at 16 MHz
    TIM1->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM1->CCMR2 |= TIM_CCMR2_CC3S_0; // CC3 as input, TI3 mapped
    TIM1->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E; // falling edge
    TIM1->DIER |= TIM_DIER_CC3IE;
    TIM1->SR &= ~TIM_SR_CC3IF;
    TIM1->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    /* Real IR input on PC10; use EXTI10 interrupt (same method as friend version). */
    GPIOC->MODER &= ~(3 << (IR_PIN * 2));
    GPIOC->PUPDR &= ~(3 << (IR_PIN * 2));
    GPIOC->PUPDR |=  (1 << (IR_PIN * 2));

    /* EXTI3 -> PB3 */
    SYSCFG->EXTICR[0] &= ~(0xF << 12);
    SYSCFG->EXTICR[0] |=  (0x1 << 12);

    /* EXTI4 -> PB4, EXTI5 -> PB5 */
    SYSCFG->EXTICR[1] &= ~((0xF << 0) | (0xF << 4));
    SYSCFG->EXTICR[1] |=  ((0x1 << 0) | (0x1 << 4));

    /* EXTI10 -> PC10 (IR). PA10 uses TIM1_CH3 separately. */
    SYSCFG->EXTICR[2] &= ~(0xF << 8);
    SYSCFG->EXTICR[2] |=  (0x2 << 8); // Port C

    EXTI->IMR |= (EXTI_IMR_MR3 |
                  EXTI_IMR_MR4 |
                  EXTI_IMR_MR5 |
                  EXTI_IMR_MR10);

    EXTI->FTSR |= (EXTI_FTSR_TR3 |
                   EXTI_FTSR_TR4 |
                   EXTI_FTSR_TR5 |
                   EXTI_FTSR_TR10);

    /* Only falling edge is used for IR (active-low sensor) and all buttons. */
    EXTI->RTSR &= ~EXTI_RTSR_TR10;

    /* Clear any stale pending flags before enabling the IRQs. */
    EXTI->PR |= (EXTI_PR_PR3 | EXTI_PR_PR4 | EXTI_PR_PR5 | EXTI_PR_PR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* UART2: PA2 TX, PA3 RX */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= ((2 << GPIO_MODER_MODER2_Pos) |
                     (2 << GPIO_MODER_MODER3_Pos));
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos) |
                      (7 << GPIO_AFRL_AFSEL3_Pos));

    USART2->BRR = 0x8B;
    USART2->CR1 |= (USART_CR1_TE |
                    USART_CR1_RE |
                    USART_CR1_RXNEIE |
                    USART_CR1_UE);
    NVIC_EnableIRQ(USART2_IRQn);

    /* ADC1 + DMA: continuously updates LDR and Pot without polling */
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;
    DMA2_Stream0->NDTR = 2;
    DMA2_Stream0->CR |= (0 << 25) |
                        (1 << 13) |
                        (1 << 11) |
                        (1 << 10) |
                        (1 << 8);
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_CONT |
                 ADC_CR2_DMA |
                 ADC_CR2_DDS;
    ADC1->SQR1 |= (1 << 20);
    ADC1->SQR3 = (LIGHT_SENSOR_PIN << 0) |
                 (POT_PIN << 5);
    ADC1->SMPR2 |= (7 << (LIGHT_SENSOR_PIN * 3)) |
                   (7 << (POT_PIN * 3));
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;

}

/*==============================================================================
 * Interrupt handlers
 *============================================================================*/
void SysTick_Handler(void)
{
    msTicks++;

    if (current_state == STATE_RUNNING ||
        current_state == STATE_WAIT_LDR ||
        current_state == STATE_SETTLE ||
        current_state == STATE_EVALUATE ||
        current_state == STATE_ROUTE_ACCEPT ||
        current_state == STATE_ROUTE_REJECT ||
        current_state == STATE_WAIT_LDR_CLEAR) {
        if (msTicks % 1000 == 0) {
            elapsed_time++;
        }
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
        }
        else if (rx_index < 49) {
            rx_buffer[rx_index++] = rx_data;
        }
    }

    if ((USART2->SR & USART_SR_TXE) &&
        (USART2->CR1 & USART_CR1_TXEIE)) {
        if (tx_head != tx_tail) {
            USART2->DR = tx_buffer[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1) % TX_BUFFER_SIZE);
        }
        else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* PB3 = Reset */
void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR3) {
        if ((msTicks - last_btn_reset) >= BUTTON_DEBOUNCE_MS) {
            last_btn_reset = msTicks;

            /* Active states go to REPORT first; IDLE/READY reset directly. */
            Request_Reset_Report();
        }
        EXTI->PR |= EXTI_PR_PR3;
    }
}

/* PB4 = Start */
void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR4) {
        if ((msTicks - last_btn_start) >= BUTTON_DEBOUNCE_MS) {
            last_btn_start = msTicks;

            if (current_state == STATE_READY) {
                elapsed_time = 0;
                last_printed_sec = 0xFFFFFFFF;
                current_state = STATE_RUNNING;
                UART2_TxString("\r\n[STARTED]\r\n");
            }
        }
        EXTI->PR |= EXTI_PR_PR4;
    }
}

/* PB5 = Pause / Resume */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR5) {
        if ((msTicks - last_btn_pause) >= BUTTON_DEBOUNCE_MS) {
            last_btn_pause = msTicks;

            if (current_state == STATE_RUNNING ||
                current_state == STATE_WAIT_LDR ||
                current_state == STATE_SETTLE ||
                current_state == STATE_EVALUATE ||
                current_state == STATE_ROUTE_ACCEPT ||
                current_state == STATE_ROUTE_REJECT ||
                current_state == STATE_WAIT_LDR_CLEAR) {
                Pause_System();
            }
            else if (current_state == STATE_PAUSED) {
                Resume_System();
            }
            else if (current_state == STATE_FAULT) {
                Resume_From_Fault();
            }
        }
        EXTI->PR |= EXTI_PR_PR5;
    }
}

/* PA10 = External Emergency via TIM1_CH3 input-capture interrupt */
void TIM1_CC_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_CC3IF) {
        TIM1->SR &= ~TIM_SR_CC3IF;

        if ((msTicks - last_btn_emergency) >= BUTTON_DEBOUNCE_MS) {
            last_btn_emergency = msTicks;
            /* One interrupt corresponds to the falling edge of the Emergency button. */
            Enter_Emergency();
        }
    }
}

/* PC10 = IR Sensor via EXTI10 interrupt */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR10) {
        EXTI->PR |= EXTI_PR_PR10;

        if ((msTicks - last_manual_trigger) >= IR_COOLDOWN_MS) {
            last_manual_trigger = msTicks;

            if (current_state == STATE_RUNNING && !package_active) {
                ir_event_pending = 1;
            }
        }
    }
}
