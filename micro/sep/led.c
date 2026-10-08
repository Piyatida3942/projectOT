/*******************************************************************************
 * File Name    : led.c
 * Description  : LED control
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "led.h"

/* --- File-local prototypes --- */
static void Clear_Size_LEDs(void);
static void Set_Size_LED_Output(PackageSize size, uint8_t on);
static void Reject_LED_On(void);
static void Reject_LED_Off(void);

void Start_Reset_Flash(void)
{
    reset_flash_active = 1u;
    reset_flash_end_time = msTicks + RESET_FLASH_DURATION_MS;
    Set_All_LEDs();
}

void Clear_All_LEDs(void)
{
    GPIOA->BSRR = (1u << (LED_STATUS_PIN + 16u)) |
                  (1u << (LED_RED_PIN + 16u)) |
                  (1u << (LED_YELLOW_PIN + 16u));
    GPIOB->BSRR = (1u << (LED_GREEN_PIN + 16u)) |
                  (1u << (LED_WAIT_LDR_PIN + 16u));
    Reject_LED_Off();
}

void Set_All_LEDs(void)
{
    GPIOA->BSRR = (1u << LED_STATUS_PIN) |
                  (1u << LED_RED_PIN) |
                  (1u << LED_YELLOW_PIN);
    GPIOB->BSRR = (1u << LED_GREEN_PIN);

    GPIOB->BSRR = (1u << (LED_WAIT_LDR_PIN + 16u));
    Reject_LED_Off();
}

void Set_Emergency_LEDs(void)
{
    Set_All_LEDs();
}

void Wait_LDR_LED_On(void)
{
    GPIOB->BSRR = (1u << LED_WAIT_LDR_PIN);
}

void Wait_LDR_LED_Off(void)
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

void Start_Size_Result_LED(PackageSize size, PackageDecision decision)
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

void Update_Size_Result_LED(void)
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
