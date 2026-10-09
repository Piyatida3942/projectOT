/*******************************************************************************
 * File Name    : main.c
 * Description  : Conveyor Simulation - Merged IR/Fault/Emergency System
 *                (MISRA-C checksheet compliant - 22 rule pass)
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "app_config.h"
#include "system_init.h"
#include "uart.h"
#include "led.h"
#include "servo.h"
#include "sorter.h"

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
                    /* พัสดุถึง LDR แล้ว แต่ยังไม่วัดขนาด: ต้องรอดูก่อนว่าพัสดุผ่าน LDR ไปได้ (ไม่ติด) */
                    ldr_clear_start_time = msTicks;
                    current_state = STATE_WAIT_LDR_CLEAR;
                    UART2_TxString("[SENSOR] LDR : Package detected at LDR sensor. Checking for jam...\r\n");
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
                    /* พัสดุผ่านการตรวจ stuck และถูกคัดแยกลงกล่องแล้ว -> นับผลตอนนี้ */
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
                else {
                    /* No action */
                }
                break;

            case STATE_WAIT_LDR_CLEAR:
                /* ตรวจ stuck ก่อนวัดขนาด: พัสดุต้องผ่าน LDR ไป (LDR สว่างอีกครั้ง) ภายใน LDR_CLEAR_TIMEOUT_MS */
                if (!IS_LDR_DARK()) {
                    settle_start_time = msTicks;
                    current_state = STATE_SETTLE;
                    UART2_TxString("[SENSOR] LDR : Package passed LDR sensor. No jam detected.\r\n");
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
