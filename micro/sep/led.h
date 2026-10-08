/*******************************************************************************
 * File Name    : led.h
 * Description  : LED control API
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#ifndef LED_H
#define LED_H

#include "app_config.h"

void Clear_All_LEDs(void);
void Set_All_LEDs(void);
void Set_Emergency_LEDs(void);
void Wait_LDR_LED_On(void);
void Wait_LDR_LED_Off(void);
void Start_Size_Result_LED(PackageSize size, PackageDecision decision);
void Update_Size_Result_LED(void);
void Start_Reset_Flash(void);

#endif /* LED_H */
