/*******************************************************************************
 * File Name    : sorter.h
 * Description  : Sorting logic / state helper API
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#ifndef SORTER_H
#define SORTER_H

#include "app_config.h"

void Process_IR_Event(void);
void Process_Evaluate(uint16_t pot_val);
uint8_t Targets_Complete(void);
void Enter_Fault(uint8_t reason);
void Resume_From_Fault(void);
void Enter_Emergency(void);
void Pause_System(void);
void Resume_System(void);
void Request_Reset_Report(void);
void Reset_To_Idle(void);
void Reset_Metrics(void);
void Parse_Config(char *str);

#endif /* SORTER_H */
