#include "PWM.h"



void Motor_Test(int Min_Pulse, int Max_Pulse)
{

	int i = 0;
	for (i = Min_Pulse; i < Max_Pulse;)
	{
		//  TIM2->CCR1 = i;
		MOTOR_V1 = i;
		MOTOR_V2 = i;
		MOTOR_V3 = i;
		MOTOR_V4 = i;
		HAL_Delay(100);
		i += 50;
	}
}

void Motor_Test_2(int Min_Pulse, int Max_Pulse)
{
	int i = 0;
	int a = 0;
	int b = 9700;
	for (i = Min_Pulse; i < 9700;)
	{
		//  TIM2->CCR1 = i;
		MOTOR_V1 = i;
		MOTOR_V2 = i;
		MOTOR_V3 = i;
		MOTOR_V4 = i;
		HAL_Delay(100);
		i += 10;
	}
	for (i = 9700; i < 10500;)
	{
		//  TIM2->CCR1 = i;
		MOTOR_V1 = i;
		MOTOR_V2 = i;
		MOTOR_V3 = b - a;
		MOTOR_V4 = i;
		HAL_Delay(100);
		i += 10;
		a += 10;
	}
}

void ESC_Calibration(int Min_Pulse, int Max_Pulse)
{
	//  TIM2->CCR1 = 15500;
	TIM2->CCR1 = Max_Pulse;
	TIM2->CCR2 = Max_Pulse;
	TIM3->CCR1 = Max_Pulse;
	TIM3->CCR2 = Max_Pulse;
	HAL_Delay(5000);
	//  TIM2->CCR1 = 8500;
	TIM2->CCR1 = Min_Pulse;
	TIM2->CCR2 = Min_Pulse;
	TIM3->CCR1 = Min_Pulse;
	TIM3->CCR2 = Min_Pulse;
	HAL_Delay(4000);
}

void Motor_Init(int Min_Pulse)
{
	MOTOR_V1 = Min_Pulse;
	MOTOR_V2 = Min_Pulse;
	MOTOR_V3 = Min_Pulse;
	MOTOR_V4 = Min_Pulse;
	HAL_Delay(5000);
}

void Motor_Start(void)
{
	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);

	MOTOR_V1 += 100;
	MOTOR_V2 += 100;
	MOTOR_V3 += 100;
	MOTOR_V4 += 100;
	HAL_Delay(100);
}
