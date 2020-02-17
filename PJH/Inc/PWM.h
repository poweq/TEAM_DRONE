#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx_hal.h"

#define MOTOR_V1 TIM2->CCR1
#define MOTOR_V2 TIM2->CCR2
#define MOTOR_V3 TIM3->CCR1
#define MOTOR_V4 TIM3->CCR2

void ESC_Calibration(int Min_Pulse, int Max_Pulse);
void Motor_Init(int Min_Pulse);
void Motor_Test(int Min_Pulse, int Max_Pulse);
void Motor_Test_2(int Min_Pulse, int Max_Pulse);
void Motor_Start(void);




#endif