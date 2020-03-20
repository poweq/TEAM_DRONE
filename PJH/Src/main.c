/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//#include "fuzzy.h"
#include "stm32f4xx_hal.h"
#include "tm_stm32_mpu9250.h"
#include "Quaternion.h"
#include "pid.h"
#include "PWM.h"
#include "LPF.h"
#include "fuzzy.h"
#include "STM32F4_FLASH_MEMORY.h"
#include "nRF24_Receive.h"
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI                      (3.141592f)             //the ratio of the circumference of a circle to its diameter.
#define dt                      (2.0f)                  //Least dt milliseconds (>1/dt mHz)Update term (milliseconds).
#define init_angle_average      (40)                    //Initiate_Setting_angle.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//========================GLOBAL VARIABLES=========================
//=================================================================
//======================UART variables=============================
uint8_t uart1_tx_to_MFC[100];                             //Trandmit variable.
uint8_t uart1_tx_to_MFC2[100];                           //Trandmit variable.
uint8_t uart2_tx_data2[255];
uint8_t uart2_tx_data[255];

uint8_t pid_buffer[71];
int num = 0;
uint8_t data;
//=======================INIT Variables=============================
//=======================MPU9250 variables==========================
//==================================================================
//========================nRF24L01 GLOBAL VARIABLES==============================
//uint8_t TxAddress[] = {                                 // Controller
//  0xE7,
//  0xE7,
//  0xE7,
//  0xE7,
//  0xE7
//};
uint8_t MyAddress[] = {                                 // Controller 
  0x7E,
  0x7E,
  0x7E,
  0x7E,
  0x7E
};

//int value=0;                                            // 

//========================================================================

//==================printf FUNCTION====================
int fputc(int ch ,FILE *f)
{
 HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
 return ch;
}
//=====================================================

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //=======================INIT Variables=============================
  TM_MPU9250_t    MPU9250;                                //MPU9250 Sensor structure.
  __PID           pid;                                    //PID Controll structure.
  //=====================Flag and Time flag=================================    
  uint32_t Now = 0;                                       //Used to calculate integration interval.
  uint32_t lastUpdate = 0;                                //Used to calculate integration interval.
  uint32_t before_while = 0;                              //Time of Before entering while loop.
  uint16_t wait = 0;                                      //getting initiate setting_angle waiting time(3secs) 
  uint8_t wait_flag = 0;                                  //Time waiting flag.
 
  uint32_t UART_Now = 0;     
  uint32_t UART_Pre = 0;
  uint32_t UART_deltat = 0;
  
  uint8_t UART_flag=0;
  uint8_t UART_sytic_flag=0;
  //=========================UART Variables=================================
  //====================Quaternion VARIABLES================================
  float deltat = 0.0f;                                    //integration interval for filter schemes.
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};                  // vector to hold quaternion.
  //====================Fuzzy Variables=====================================
  //float prev_err[3];                                    //Prev_Setting_point - Euler_angle.
  //========================================================================
  int Controller_1 = 20;                                  //Moter Throttle.
  int Controller_2 = 0;                                   //Moter Yaw Throttle. 
  //================hanging Variables from external controll================
  float setting_angle[3] = {0.0f, 0.0f, 0.0f};            //roll pitch yaw.
  float init_setting_angle[3] = {0.0f, 0.0f, 0.0f};
   //float pid_val[3][3] = {{2.5f, 0.2f, 0.0f}, {3.0f, 0.005f, 0.0f}, {2.0f, 0.005f, 0.0f}};            //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
  float pid_val[3][3] = {{2.63f, 0.2f, 0.0f}, {2.63f, 0.2f, 0.0f}, {2.0f, 0.005f, 0.0f}};            //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
  //float inpid_val[3][3] = {{8.0f, 0.3f, 2.0f}, {8.0f, 0.3f, 2.0f}, {1.0f, 0.5f, 0.3f}};              //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
  float inpid_val[3][3] = {{11.5f, 0.28f, 2.848f}, {8.0f, 0.3f, 2.0f}, {1.0f, 0.5f, 0.3f}};
  float angular_velocity[3];                              //For double loop PID.
  //====================Quaternion VARIABLES================================
  float preGyro[3] = {0.0f, 0.0f, 0.0f};
  float LPF_Gyro[3] = {0.0f, 0.0f, 0.0f};
  float Euler_angle[3] = {0.0f, 0.0f, 0.0f};              //roll pitch yaw.
  float preEuler_angle[3] = {0.0f, 0.0f, 0.0f};           //Used in LPF.
  float LPF_Euler_angle[3] = {0.0f, 0.0f, 0.0f};          //Used in LPF.
  //===========================MPU9250 Variables============================
  float Self_Test[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; //MPU9250 Accell and Gyro Self_Test.
  float Self_Test_Mag[3] = {0.0f, 0.0f, 0.0f};            //MPU9250 Magnetometer Self_Test.
  //=====================Drone Calibration Mode Variables===================
  uint8_t CaliFlag = 0;
  //========================nRF24L01 VARIABLES==============================
  int temp_int;                                           // uint8_t
  float temp;                                             // uint8_t
  
  /* USER CODE END 1 */   

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __INIT__MPU9250(&MPU9250);
  MPU9250SelfTest(&MPU9250, &Self_Test[0],TM_MPU9250_Device_0);
  if (0){
    calibrateMPU9250(&MPU9250);
  }
  TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);
  TM_MPU9250_ReadMagASA(&MPU9250);      //Get MPU9250 Magnetic ASA data.
  pid_init(&pid, pid_val, inpid_val);
  //fuzzy_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */  
  //===================Calibration Part=========================
  if(CaliFlag == 1)    
  {    
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    
    MPU9250SelfTest(&MPU9250, &Self_Test[0],TM_MPU9250_Device_0);  
    calibrateMPU9250(&MPU9250);   
    Delayms(1000);     
    TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);
    TM_MPU9250_ReadMagASA(&MPU9250);            //Get MPU9250 Magnetic ASA data.
    AK8963SelfTest(&MPU9250, &Self_Test_Mag[0]);
    MagCalibration(&MPU9250);
    //while(1){}                                  //Unlimited loop.
  }
  //=================Calibration Part END=======================  
  //================PWM START===================================
  if (1)
  {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }

  //  ESC_Calibration();  
    Motor_Init();  
  //  Motor_Start();
    
  //*********************************************
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);          
    memset(pid_buffer,'\0',sizeof(pid_buffer));           
  //********************************************
  
  TM_NRF24L01_Init(120,8);  
  TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_250k, TM_NRF24L01_OutputPower_0dBm);
  TM_NRF24L01_SetMyAddress(MyAddress);
  //====================Get Biases From Flash Memory==========================
  if(1){
    Get_biases(&MPU9250);
  }
  //===================Get Biases From Flash Memory END=======================  
  before_while = HAL_GetTick(); //Get time of before while loop.
  lastUpdate = before_while;    //First time of lastUpdate using for gain the deltat.
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {         
    //int aa = HAL_GetTick();     //Get time.    
    //HAL_UART_Receive_IT(&huart1, &data, 1);             //PID PPID
  //========================Get MPU9250 data===========================  
    TM_MPU9250_ReadAcce(&MPU9250);      //get Accel data.
    TM_MPU9250_ReadGyro(&MPU9250);      //get Gyro data.
    TM_MPU9250_ReadMag(&MPU9250);       //get Magnetic data.
  //======================Get MPU9250 data END===========================  
  //===Subtract Automatic Accelometer Gyroscope and Magnetic filed bias===

     MPU9250.Ax -= MPU9250.Accbiasx;             //callibrate Accel values.
     MPU9250.Ay -= MPU9250.Accbiasy;
     MPU9250.Az -= MPU9250.Accbiasz;

     MPU9250.Gx -= MPU9250.Gybiasx;              //callibrate Gyro values.
     MPU9250.Gy -= MPU9250.Gybiasy;
     MPU9250.Gz -= MPU9250.Gybiasz;  
     
     MPU9250.Mx -= MPU9250.Magbiasx;             //callibrate Magnetic values.
     MPU9250.My -= MPU9250.Magbiasy;
     MPU9250.Mz -= MPU9250.Magbiasz;     
     
     MPU9250.Mx *= MPU9250.Magscalex;            //callibrate Magnetic values.
     MPU9250.My *= MPU9250.Magscaley;
     MPU9250.Mz *= MPU9250.Magscalez;

  //=======Subtract Automatic Accelometer Gyroscope and Magnetic filed bias END=======
  //==================Init settiing angle=====================
    if (wait_flag < init_angle_average)
    {
      wait = HAL_GetTick() - before_while;
      if (wait >= 3500)
      {            
        init_setting_angle[0] += Euler_angle[0];         //roll
        init_setting_angle[1] += Euler_angle[1];         //pitch
        init_setting_angle[2] += Euler_angle[2];         //yaw
        wait_flag ++;
      }
    }    
    if (wait_flag == init_angle_average)
    {
//      setting_angle[0] = init_setting_angle[0] / init_angle_average;    //init roll
//      setting_angle[1] = init_setting_angle[1] / init_angle_average;    //init pitch
//      setting_angle[2] = init_setting_angle[2] / init_angle_average;    //init yaw
      wait_flag ++;
    }
  //================Init settiing angle END====================
  //====================Get delta T ===========================
    Now = HAL_GetTick();                //Get current time.
    deltat += (Now - lastUpdate);       //Set integration time by time elapsed since last filter update (milliseconds).
    lastUpdate = Now;                   //Update lastupdate time to current time.
  //==================Get delta T END==========================
    //sprintf((char*)uart2_tx_data2,"%10.5f\r\n",deltat);    
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);
  //===========================================================
    angular_velocity[0] = MPU9250.Gx / 500.0f * deltat;                //angular velocity (degree/2ms(*2))
    angular_velocity[1] = MPU9250.Gy / 500.0f * deltat;
    angular_velocity[2] = MPU9250.Gz / 500.0f * deltat;    
    if (deltat >= dt)                                                   //Update term.(500Hz.dt=2)
    {
      deltat /= 1000.0f;                                                             //Make millisecond to second.
      __LPFGyro(LPF_Gyro, &MPU9250, preGyro, deltat);
      MahonyQuaternionUpdate(MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Gx*PI/180.0f, MPU9250.Gy*PI/180.0f, MPU9250.Gz*PI/180.0f, MPU9250.My, MPU9250.Mx, -MPU9250.Mz, q, deltat);
      Quternion2Euler(q, Euler_angle);                                                  //Get Euler angles (roll, pitch, yaw) from Quaternions.
      __LPF(LPF_Euler_angle, Euler_angle, preEuler_angle, deltat);
  //=========================Fuzzy part============================
//      Fuzzification(setting_angle[0], Euler_angle[0], &prev_err[0]);                  //roll
//      Create_Fuzzy_Matrix();
//      Defuzzification(&inpid_val[0][0],&inpid_val[0][1],&inpid_val[0][2]);                  //Fuzzy roll end.
//      
//      Fuzzification(setting_angle[1], Euler_angle[1], &prev_err[1]);                  //pitch
//      Create_Fuzzy_Matrix();
//      Defuzzification(&inpid_val[1][0],&inpid_val[1][1],&inpid_val[1][2]);                  //Fuzzy pitch end.
//      
//      Fuzzification(setting_angle[2], Euler_angle[2], &prev_err[2]);                  //yaw
//      Create_Fuzzy_Matrix();
//      Defuzzification(&inpid_val[2][0],&inpid_val[2][1],&inpid_val[2][2]);                  //Fuzzy yaw end.     
//      pid_gain_update(&pid, pid_val, inpid_val);                                      //From Fuzzy the PID gain value is changed.
  //=======================Fuzzy part END============================
      if (HAL_GetTick() - before_while >= 5500)
      {
        //__pid_update(&pid, setting_angle, Euler_angle, angular_velocity, deltat);         //PID value update.
        __pid_update(&pid, setting_angle, LPF_Euler_angle, angular_velocity, deltat);         //PID value update.
      }    
      //if(Euler_angle[2] < 0) Euler_angle[2] += 360.0f;         // Ensure yaw stays between 0 and 360     
      deltat = 0.0f;                                             //reset deltat.
    }
//=====================Data print transmit UART part===============        
    //sprintf((char*)uart2_tx_data2,"%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f \r\n",  \
      MPU9250.Ax, MPU9250.Ay ,MPU9250.Az, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Mx, MPU9250.My, MPU9250.Mz);
    //sprintf((char*)uart2_tx_data2,"%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f \r\n",  \
      MPU9250.Ax, MPU9250.Ay ,MPU9250.Az, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, LPF_Gyro[0],LPF_Gyro[1], LPF_Gyro[2]);
    //sprintf((char*)uart2_tx_data2,"%f  %f  %f  %f  %f  %f\r\n", Self_Test[0], Self_Test[1], Self_Test[2], Self_Test[3], Self_Test[4], Self_Test[5]);
    //sprintf((char*)uart2_tx_data2,"%f  %f  %f\r\n", Self_Test_Mag[0], Self_Test_Mag[1], Self_Test_Mag[2]);
    //sprintf((char*)uart2_tx_data2,"%f  %f  %f  %f  %f  %f  %f  %f  %f\r\n", MPU9250.Accbiasx, MPU9250.Accbiasy, MPU9250.Accbiasz, MPU9250.Gybiasx, MPU9250.Gybiasy, MPU9250.Gybiasz, MPU9250.Magbiasx, MPU9250.Magbiasy, MPU9250.Magbiasz);
    //sprintf((char*)uart2_tx_data2,"%f  %f  %f  %f  %f  %f\r\n", MPU9250.Magbiasx, MPU9250.Magbiasy, MPU9250.Magbiasz, MPU9250.Magscalex, MPU9250.Magscaley, MPU9250.Magscalez);
    //sprintf((char*)uart2_tx_data2,"%10.4f %10.4f %10.4f %10.4f\r\n", q[0], q[1], q[2], q[3]);   

    //sprintf((char*)uart2_tx_data2,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[0],pid.output[1], pid.output[2]);
    //sprintf((char*)uart2_tx_data2,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  LPF_Euler_angle[0], LPF_Euler_angle[1], LPF_Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[0],pid.output[1], pid.output[2]);   
    //sprintf((char*)uart2_tx_data2,"%4d %4d %4d\r\n", (int)LPF_Euler_angle[0], (int)LPF_Euler_angle[1], (int)LPF_Euler_angle[2]);
    
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);
 //====================Data print transmit UART part END===================
    
 //======================BLDC Motor Part===================================
   
    if(HAL_GetTick() - before_while >= 5000 && HAL_GetTick() - before_while < 6000)
    {
       Motor_Start();
    }
    
    else if (HAL_GetTick() - before_while >= 6000)// && HAL_GetTick() - before_while <= 400000)
    { 
      //Motor_Drive(Controller_1, pid.output);

      if (Controller_1 <= 5)
      {
        MOTOR_V1 = MIN_PULSE;
        MOTOR_V2 = MIN_PULSE;
        MOTOR_V3 = MIN_PULSE;
        MOTOR_V4 = MIN_PULSE;
      }
      
      else if (Controller_1 > 5)    //Controller_1
      {     
        if (fabs(LPF_Euler_angle[0]) > 15.0f || fabs(LPF_Euler_angle[1]) > 15.0f)    //Restrict yaw acting Euler angle.
        {           
          pid.output[2] = 0.0f;
        }         
        
        //MOTOR_V1 = MIN_PULSE + (Controller_1 * 65) + (int)(MoterGain_roll * pid.output[0]) - (int)(MoterGain_pitch * pid.output[1]) - (int)(MoterGain_yaw * pid.output[2]) + (Controller_2 * 5);
        //MOTOR_V1 = MIN_PULSE + (Controller_1 * 65) - (int)(MoterGain_pitch * pid.output[1]) + (Controller_2 * 5);
        MOTOR_V1 = MIN_PULSE + (Controller_1 * 65) + (int)(MoterGain_roll * pid.output[0]);// + (Controller_2 * 5);
        if (MOTOR_V1 >= MAX_PULSE - MOTER_SAFTY)
          MOTOR_V1 = MAX_PULSE -  MOTER_SAFTY;
        else if (MOTOR_V1 <= MIN_PULSE + 700)
          MOTOR_V1 = MIN_PULSE + 700;
      
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 65) - (int)((MoterGain_roll) * pid.output[0]) - (int)((MoterGain_pitch) * pid.output[1]) + (int)(MoterGain_yaw * pid.output[2]) - (Controller_2 * 5);
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 65) - (int)((MoterGain_pitch) * pid.output[1]) - (Controller_2 * 5);
        MOTOR_V2 = MIN_PULSE + (Controller_1 * 65) - (int)(MoterGain_roll * pid.output[0]);// + (Controller_2 * 5);

        if (MOTOR_V2 >= MAX_PULSE - MOTER_SAFTY)
          MOTOR_V2 = MAX_PULSE - MOTER_SAFTY;
        else if (MOTOR_V2 <= MIN_PULSE + 700)
          MOTOR_V2 = MIN_PULSE + 700;
      
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 65) + (int)(MoterGain_roll * pid.output[0]) + (int)(MoterGain_pitch * pid.output[1]) + (int)(MoterGain_yaw * pid.output[2]) - (Controller_2 * 5);
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 65) + (int)(MoterGain_pitch * pid.output[1]) - (Controller_2 * 5);
        MOTOR_V3 = MIN_PULSE + (Controller_1 * 65) + (int)(MoterGain_roll * pid.output[0]);// + (Controller_2 * 5);

        if (MOTOR_V3 >= MAX_PULSE - MOTER_SAFTY)
          MOTOR_V3 = MAX_PULSE - MOTER_SAFTY;
        else if (MOTOR_V3 <= MIN_PULSE + 700)
          MOTOR_V3 = MIN_PULSE + 700;
      
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 65) - (int)((MoterGain_roll) * pid.output[0]) + (int)((MoterGain_pitch) * pid.output[1]) - (int)(MoterGain_yaw * pid.output[2]) + (Controller_2 * 5); 
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 65) + (int)((MoterGain_pitch) * pid.output[1]) + (Controller_2 * 5); 
        MOTOR_V4 = MIN_PULSE + (Controller_1 * 65) - (int)(MoterGain_roll * pid.output[0]);// + (Controller_2 * 5);

        if (MOTOR_V4 >= MAX_PULSE - MOTER_SAFTY)
          MOTOR_V4 = MAX_PULSE - MOTER_SAFTY;
        else if (MOTOR_V4 <= MIN_PULSE + 700)
          MOTOR_V4 = MIN_PULSE + 700;
       }
    }  
    //Motor_Stop(400000, before_while);    
    //sprintf((char*)uart2_tx_data2,"%10d  %10d  %10d  %10d\r\n",  MOTOR_V1, MOTOR_V2, MOTOR_V3, MOTOR_V4);
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);
//======================BLDC Motor Part END===================================
    
//======================NRF24L01 Receive Part=================================    
  NRF24_Receive(&Controller_1,temp,temp_int,&pid,setting_angle,&Controller_2);          
//=====================NRF24L01 Receive Part END==============================  
    
//========================Data transmit part==================================
    if(UART_flag ==0)
    {
      //sprintf((char*)uart1_tx_to_MFC,"%.2f,%.2f,%.2f", LPF_Euler_angle[0], LPF_Euler_angle[1], LPF_Euler_angle[2]);
      sprintf((char*)uart1_tx_to_MFC,"%d,%d,%d", (int)LPF_Euler_angle[0], (int)LPF_Euler_angle[1], (int)LPF_Euler_angle[2]);
      //HAL_UART_Transmit(&huart1,uart1_tx_to_MFC ,sizeof(uart1_tx_to_MFC),5);
      //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 5);
    }
//*********************************************************************************
  
    if(num>=70)
    {
      UART_flag=1;
      if(UART_sytic_flag ==0)
      {
        UART_Pre = HAL_GetTick();
        UART_sytic_flag=1;
        
      }
      UART_Now = HAL_GetTick();               //Get current time.
      UART_deltat += (UART_Now - UART_Pre);       //Set integration time by time elapsed since last filter update (milliseconds).
      UART_Pre = UART_Now;
      //UART_lastUpdate = UART_Now;                   //Update lastupdate time to current time.
      if (UART_deltat >= 10)
      {
          //HAL_UART_Transmit(&huart2,pid_buffer,sizeof(pid_buffer), 10);
          if (strstr((char*)pid_buffer,"B") != NULL)               //Outer PID.
          {
            Parsing_PID_val(pid_buffer, pid_val);
            sprintf((char*)uart1_tx_to_MFC2,"PPP%6.3fPPI%6.3fPPD%6.3fPRRP%6.3fRRI%6.3fRRD%6.3fRYYP%6.3fYYI%6.3fYYD%6.3fY\r\n", pid_val[1][0], pid_val[1][1], pid_val[1][2], pid_val[0][0], pid_val[0][1], pid_val[0][2], pid_val[2][0], pid_val[2][1], pid_val[2][2]);
            pid_gain_update(&pid, pid_val, inpid_val);
            //HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            memset(pid_buffer,'\0',sizeof(pid_buffer));
            memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
            num = 0;
          }
          else if (strstr((char*)pid_buffer,"A") != NULL)          //Inner PID.
          {
            Parsing_inPID_val(pid_buffer, inpid_val);
            sprintf((char*)uart1_tx_to_MFC2,"PP%6.3fPI%6.3fPD%6.3fPRP%6.3fRI%6.3fRD%6.3fRYP%6.3fYI%6.3fYD%6.3fY\r\n", inpid_val[1][0], inpid_val[1][1], inpid_val[1][2], inpid_val[0][0], inpid_val[0][1], inpid_val[0][2], inpid_val[2][0], inpid_val[2][1], inpid_val[2][2]);
            pid_gain_update(&pid, pid_val, inpid_val);
            //HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            memset(pid_buffer,'\0',sizeof(pid_buffer));
            memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
            num = 0;
          }
          else if (strstr((char*)pid_buffer,"C") != NULL)          //Throttle.
          {
            Parsing_Throttle_val(pid_buffer, &Controller_1);
            sprintf((char*)uart1_tx_to_MFC2,"T%6.3f\r\n", (float)Controller_1);
            //HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            memset(pid_buffer,'\0',sizeof(pid_buffer));
            memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
            num = 0;
          }
          else if (strstr((char*)pid_buffer,"D") != NULL)          //Setting Point.
          {
            Parsing_SettingPoint_val(pid_buffer, setting_angle);
            sprintf((char*)uart1_tx_to_MFC2,"S%6.3fP%6.3fR%6.3fY\r\n", setting_angle[0], setting_angle[1], setting_angle[2]);
            //HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
            memset(pid_buffer,'\0',sizeof(pid_buffer));
            memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
            num = 0;
           }
      UART_flag=0;
      UART_sytic_flag=0;
      UART_deltat = 0;
      }
    }
//========================Data transmit part END=============================
    
//=====================TIme Check==========================
    //int bb = HAL_GetTick();
    //sprintf((char*)uart2_tx_data,"%d  %d\r\n",  aa, bb);    
    //HAL_UART_Transmit(&huart2,uart2_tx_data ,sizeof(uart2_tx_data), 3);
//=====================TIme Check END======================
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 16000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CEpin_Pin|CSNpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CEpin_Pin CSNpin_Pin */
  GPIO_InitStruct.Pin = CEpin_Pin|CSNpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
