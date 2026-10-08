/*******************************************************************************
 * File Name    : sorter.c
 * Description  : Sorting logic / state helper functions
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "sorter.h"
#include "uart.h"
#include "servo.h"
#include "led.h"

/* --- File-local prototypes --- */
static PackageSize Get_Pot_Size(uint16_t pot_val);

/*==============================================================================
 * IR / Pot Helpers
 *============================================================================*/
void Process_IR_Event(void)
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

void Process_Evaluate(uint16_t pot_val)
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
 * State & LED Helper Functions
 *============================================================================*/
uint8_t Targets_Complete(void)
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

void Enter_Fault(uint8_t reason)
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

void Resume_From_Fault(void)
{
    fault_start_time = 0u;
    last_printed_sec = TIME_SENTINEL_UNSET;
    current_state = STATE_RUNNING;
    UART2_TxString("\r\n[SYS] Resuming from fault. Sorting reactivated.\r\n");
}

void Enter_Emergency(void)
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

void Pause_System(void)
{
    resume_state = current_state;
    pause_start_time = msTicks;
    current_state = STATE_PAUSED;
    UART2_TxString("\r\n[SYS] System PAUSED. Press PAUSE (PB5) to resume or RESET (PB3)\r\n");
}

void Resume_System(void)
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

void Request_Reset_Report(void)
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

void Reset_To_Idle(void)
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

void Reset_Metrics(void)
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

void Parse_Config(char *str)
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
