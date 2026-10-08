/*******************************************************************************
 * File Name    : servo.c
 * Description  : Servo control
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "servo.h"

/*==============================================================================
 * Servo Control
 *============================================================================*/
void Servo_Reject(void)
{
    TIM4->CCR3 = SERVO_POS_REJECT;
}

void Servo_Normal(void)
{
    TIM4->CCR3 = SERVO_POS_NORMAL;
}
