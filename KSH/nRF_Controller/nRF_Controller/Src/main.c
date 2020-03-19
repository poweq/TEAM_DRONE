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
#include "stm32f4xx.h"
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_delay.h"
#include "nRF_Transmit.h"
#include "key_Debug.h"
#include "JOYSTICK.h"
//#include "tm_stm32_usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Roll_Set_Point                       'r'
#define THROTTLE                            't'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* My address */
uint8_t MyAddress[] = {
	0xE7,
	0xE7,
	0xE7,
	0xE7,
	0x75
};

/* Receiver address */
uint8_t TxAddress[] = {
	0x7E,
	0x7E,
	0x7E,
	0x7E,
	0x57
};

// nRF 통신 변수//
uint8_t  dataIn[8];            //  TRANSMIT DATA BUFFER

//디버깅 모드 변수//
uint8_t key_input=0;
uint8_t key_tr=0;

// ADC  변수
int8_t ADC_DATA[4];   // ADC DATA BUFFER

// Interrupt variable
uint8_t button_flag=0;

// in pid값
float In_Roll_P=0, In_Roll_I=0, In_Roll_D=0;
float In_Pitch_P=0, In_Pitch_I=0, In_Pitch_D=0;
float In_Yaw_P=0, In_Yaw_I=0, In_Yaw_D=0;

// out pid값
float Out_Roll_P=0, Out_Roll_I=0, Out_Roll_D=0;
float Out_Pitch_P=0, Out_Pitch_I=0, Out_Pitch_D=0;
float Out_Yaw_P=0, Out_Yaw_I=0, Out_Yaw_D=0;

// throttle 변수
float Throttle=0;

//SettingPoint
int Set_Point[3];       // Roll_SetPoint, Pitch_SetPoint, Yaw_SetPoint


int count=1;


int fputc(int ch ,FILE *f)
{
 HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
 return ch;
}

void Display_UI()
{
   printf("\r\n  --------------Enter Debugging Mode---------------\r\n");
   printf("  ------------------------------------------------------\r\n");
   printf("     Roll             Pitch                Yaw\r\n");
   printf("  ------------------------------------------------------\r\n");
   printf(" [1].IN_R_P : %.3f     [2].IN_P_P : %.3f      [3].IN_Y_P : %.3f\r\n",In_Roll_P,In_Pitch_P,In_Yaw_P);
   printf(" [4].IN_R_I : %.3f     [5].IN_P_I : %.3f      [6].IN_Y_I : %.3f\r\n",In_Roll_I,In_Pitch_I,In_Yaw_I);
   printf(" [7].IN_R_D : %.3f     [8].IN_P_D : %.3f      [9].IN_Y_D : %.3f\r\n",In_Roll_D,In_Pitch_D,In_Yaw_D);
   printf("  ------------------------------------------------------\r\n");
   printf(" [A].OUT_R_P : %.3f     [B].OUT_P_P : %.3f      [C].OUT_Y_P : %.3f\r\n",Out_Roll_P, Out_Pitch_P, Out_Yaw_P);
   printf(" [D].OUT_R_I : %.3f     [E].OUT_P_I : %.3f      [F].OUT_Y_I : %.3f\r\n",Out_Roll_I, Out_Pitch_I, Out_Yaw_I);
   printf(" [G].OUT_R_D : %.3f     [H].OUT_P_D : %.3f      [I].OUT_Y_D : %.3f\r\n",Out_Roll_D, Out_Pitch_D, Out_Yaw_D);
   printf("  ------------------------------------------------------\r\n");
   
   printf(" [T].Throttle : %.0f                         [x].ESCAPE\r\n",Throttle);
   printf(" [R].Set_Roll : %d    [P].Set_Pitch : %d    [Y].Set_Yaw : %d\r\n",Set_Point[0],Set_Point[1],Set_Point[2]);

   
}

void PID_THROTTLE_Transmit(uint8_t key_input, uint8_t key_tr, float* data)
{
  key_tr = key_input;
  key_input='\0';
  printf("[%c]: ",key_tr);
  
  Keyboard_Debug(data,key_tr);
  
  nRF24_Transmit(data,key_tr);
  nRF24_Transmit_Status();
  //Display_UI();
  key_tr = '\0';
}

//void Set_Point_Transmit(uint8_t key_input, uint8_t key_tr, int Set_Point)
//{
//  key_tr = key_input;
//  key_input = '\0';
//  printf("Roll_SetPoint : ");
//  
//  Keyboard_Debug_Set_Point(&Set_Point[0]);
//  nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
//  nRF24_Transmit_Status();
//  Display_UI();
//  key_tr ='\0';
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t Mode_Flag = Debug_Mode_Flag;// START DEBUG MODE
  uint8_t test_buf;
  uint8_t exit_flag=0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  
  TM_NRF24L01_Init(15,8);
  TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_250k, TM_NRF24L01_OutputPower_0dBm);

  /* Set my address, 5 bytes */
  TM_NRF24L01_SetMyAddress(MyAddress);

  /* Set TX address, 5 bytes */
  TM_NRF24L01_SetTxAddress(TxAddress);

  /* Reset counter */
  TM_DELAY_SetTime(2001);
  
  HAL_ADC_Start_IT(&hadc1);
  
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /*========================DRIVE MODE===================*/
    
   if(button_flag == Drive_Mode_Flag)
   {
     if(Mode_Flag==Debug_Mode_Flag){
       Print_Mode(&Mode_Flag);
     }
     
     Print_ADC_DEBUG();
    
//     printf("Roll : %d",ADC_DATA[1]);
//     nRF24_Transmit_ADC(&ADC_DATA[1],Roll_Set_Point);
//     
//     //HAL_Delay(10);
//     
//     printf("   Th :          %d\r\n",ADC_DATA[2]);
//     nRF24_Transmit_ADC(&ADC_DATA[2],THROTTLE);
//     nRF24_Transmit_Status();
     
     HAL_Delay(10);
    }
   
   /*========================DEBUG MODE===================*/
    
  else if(button_flag == Debug_Mode_Flag)
  {
     if(Mode_Flag==Drive_Mode_Flag){
         Print_Mode(&Mode_Flag);    
     }
     INPUT_CONTORLLER();
     INPUT_THROTTLE();
     HAL_Delay(100);
  }
   
   //==============KEYBOARD INPUT MODE==============//
   
   else if(button_flag == Key_Mode_Flag)
   {
          if(Mode_Flag==Key_Mode_Flag){
             Print_Mode(&Mode_Flag);    
          }
          HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10);
          if(key_input == 'd')      //디버깅 모드로 진입
          {
            exit_flag=0;
            while(exit_flag ==0)
            {
              nRF24_Transmit_Mode_Change(key_input);
             //nRF24_Transmit_Status();
//             TM_NRF24L01_PowerUpRx();
//             //while (!TM_NRF24L01_DataReady() && TM_DELAY_Time() < 100);
//
//              printf("%d\r\n",TM_NRF24L01_DataReady());
//              TM_NRF24L01_GetData(dataIn);
//
//               //*test_buf = atof((char*) dataIn);
//               //printf("%.3f\r\n",test_buf[0]);
//               printf("%s\r\n",dataIn);
//               *dataIn='\0';
             

             Display_UI();
             
            while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK));
            
            switch(key_input)
            {
//              case 'w':
//                  key_tr = key_input;
//                  key_input = '\0';
//                  printf("Roll_SetPoint : ");

//                  Keyboard_Debug_Set_Point(&Set_Point[0]);
//                  nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
//                  nRF24_Transmit_Status();
          
//         // ====================== Roll Set_Point Increase ====================== //
//                    
////                  while(Set_Point[0]<=30)
////                  {
////                    nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
////                    nRF24_Transmit_Status();
////                    printf("set_point : %d\r\n", Set_Point[0]);
////                    Set_Point[0] += 5;
////                    if(Set_Point[0] >30)
////                    {
////                      Set_Point[0] =30;
////                      break;
////                    }
////                 }
//                 
////                Display_UI();
////                key_tr ='\0';
////                break;
//          // ============================================================== //      
//              case 'W':
//                  key_tr = key_input;
//                  key_input = '\0';
//                  printf("Roll_SetPoint : ");
//                  
//                  Keyboard_Debug_Set_Point(&Set_Point[0]);
//                  while(Set_Point[0]<=30)
//                  {
//                    nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
//                    nRF24_Transmit_Status();
//                    printf("set_point : %d\r\n", Set_Point[0]);
//                    Set_Point[0] -= 5;
////                    if(Set_Point[0] <-30)
////                    {
////                      Set_Point[0] = -30;
////                      break;
////                    }
//                 }
//                  
//                  Display_UI();
//                  key_tr ='\0';
//                  break;
                    
              case 't':
                 PID_THROTTLE_Transmit(key_input, key_tr, &Throttle);            // Throttle Input and Transmit
                 break;
                 
             // ============================== IN PID DATA ============================== //          
               case '1':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Roll_P);            //  IN_Roll_P Input and Transmit
                  //key_input='d';
                  break;                 
               
               case '2':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Pitch_P);           // IN_Pitch_P Input and Transmit
                  break;
                 
               case '3':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Yaw_P);             //  IN_Yaw_P Input and Transmit
                  break;
                 
               case '4':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Roll_I);              //  IN_Roll_I Input and Transmit
                  break;
                 
               case '5':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Pitch_I);           //  IN_Pitch_I Input and Transmit
                  break;
                 
               case '6':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Yaw_I);             //  IN_Yaw_I Input and Transmit
                  break;
                 
               case '7':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Roll_D);            //  IN_Roll_D Input and Transmit
                  break;
                 
               case '8':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Pitch_D);           //  IN_Pitch_D Input and Transmit
                  break;
                 
               case '9':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Yaw_D);             //  IN_Yaw_D Input and Transmit
                  break;
                  
              // ========================================================================= // 
                  
                  
              // ============================== OUT PID DATA ============================== // 
                  
               case 'A':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_P);            //  OUT_Roll_P Input and Transmit
                  break;                 
               
               case 'B':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_P);           // OUT_Pitch_P Input and Transmit
                  break;
                 
               case 'C':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_P);             //  OUT_Yaw_P Input and Transmit
                  break;
                 
               case 'D':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_I);              //  OUT_Roll_I Input and Transmit
                  break;
                 
               case 'E':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_I);           //  OUT_Pitch_I Input and Transmit
                  break;
                 
               case 'F':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_I);             //  OUT_Yaw_I Input and Transmit
                  break;
                 
               case 'G':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_D);            //  OUT_Roll_D Input and Transmit
                  break;
                 
               case 'H':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_D);           //  OUT_Pitch_D Input and Transmit
                  break;
                 
               case 'I':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_D);             //  OUT_Yaw_D Input and Transmit
                  break;
                  
               // ========================================================================= //
               
                case 'q':
                  key_tr = key_input;
                  key_input='\0';
                    
                   while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
                   {
                      if(count <= 1000)
                      {
                        nRF24_Transmit_ASCII(key_tr);
                        nRF24_Transmit_Status();
                         //key_tr ='\0';
                        count++;
                      }
                    }
                    count=1;
                   key_tr='\0';
                   break;
                 
                // ============================== SET POINT DATA ============================== // 
                  
                case 'r':
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Roll_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[0]);
                  nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
                  nRF24_Transmit_Status();
                  //Display_UI();
                  key_tr ='\0';
                  break;
                  
                case 'p':
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Pitch_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[1]);
                  nRF24_Transmit_Set_Point(&Set_Point[1],key_tr);
                  nRF24_Transmit_Status();
                  //Display_UI();
                  key_tr ='\0';
                  break;
                  
                case 'y':
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Yaw_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[2]);
                  nRF24_Transmit_Set_Point(&Set_Point[2],key_tr);
                  nRF24_Transmit_Status();
                  //Display_UI();
                  key_tr ='\0';
                  break;
                  
                // ========================================================================= //
                  
                  
                case 'x':
                  key_tr = key_input;
                  key_input = '\0';
                  
                  printf("Throttle : ");
                  
                  nRF24_Transmit_Mode_Change(key_tr);
                  nRF24_Transmit_Status();
                  key_tr='\0';
                  Throttle=0;
                  button_flag=0;
                  exit_flag = 1;
                  
                  break;
         
                 default:
                  printf("\r\nError Number\r\n");
                  break;
              }
            key_input = '\0';
            }
  
          }
        } 
     else 
     {
       button_flag = Debug_Mode_Flag;
     }
  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSNpin_Pin|CEpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSNpin_Pin CEpin_Pin */
  GPIO_InitStruct.Pin = CSNpin_Pin|CEpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*==================ADC CALLBACK===================*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   static uint8_t adc_count;
  
   if(adc_count ==2) //   
   {
      ADC_DATA[adc_count] =((( HAL_ADC_GetValue(hadc)/41)-100)*(-1));
   }
   else
   {
      ADC_DATA[adc_count] =(((HAL_ADC_GetValue(hadc)/41)-50) *(-1));  
   }
   adc_count++;

   if(adc_count >3)
   {
     adc_count=0;
   }

    HAL_ADC_Start_IT(&hadc1);
}
///*=================GPIO CALLBACK===================*/
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
    static uint32_t temp;// debounce time
   if(GPIO_Pin==button_Pin)
   {    
     
     if((HAL_GetTick()-temp)>500)
     {
      // button_flag = button_flag;
       button_flag++;
       if(button_flag >= 3)
         button_flag = 0;
     }
     
 
   }
      while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET);
      temp = HAL_GetTick();
 }

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
