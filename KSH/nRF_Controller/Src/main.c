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
	0xE7
};
/* Receiver address */
uint8_t TxAddress[] = {
	0x7E,
	0x7E,
	0x7E,
	0x7E,
	0x7E
};

// nRF ��� ����//
TM_NRF24L01_Transmit_Status_t transmissionStatus;
uint8_t buffer[7];
uint8_t dataOut[32], dataIn[32];

//����� ��� ����//
uint8_t key_input=0;
uint8_t key_tr=0;
uint8_t debug_buf[32];
uint8_t C_buff[7];
uint8_t tempbuf[7];
uint16_t C_count;
float temp_1;
float Qick_q;
uint8_t r_temp[32];
uint8_t d_transmit[1];

// ���� ���� ī��Ʈ ����
int fail_count;


//pid��, throttle ����
float Roll_P=0, Roll_I=0, Roll_D=0;
float Pitch_P=0, Pitch_I=0, Pitch_D=0;
float Yaw_P=0, Yaw_I=0, Yaw_D=0;
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
   printf("  ------------------------------------------------------\r\n");
   printf("     Roll             Pitch                Yaw\r\n");
   printf("  ------------------------------------------------------\r\n");
   printf(" [1].R_P : %.3f     [2].P_P : %.3f      [3].Y_P : %.3f\r\n",Roll_P,Pitch_P,Yaw_P);
   printf(" [4].R_I : %.3f     [5].P_I : %.3f      [6].Y_I : %.3f\r\n",Roll_I,Pitch_I,Yaw_I);
   printf(" [7].R_D : %.3f     [8].P_D : %.3f      [9].Y_D : %.3f\r\n",Roll_D,Pitch_D,Yaw_D);
   printf("  ------------------------------------------------------\r\n");
   printf(" [0].Throttle : %.0f\r\n",Throttle);
   printf(" [R].Set_Roll : %d    [P].Set_Pitch : %d    [Y].Set_Yaw : %d\r\n",Set_Point[0],Set_Point[1],Set_Point[2]);
}


      // ����ʿ��� DATA�� ��� ����Ǵ��� Ȯ��//

void nRF24_Transmit(float* temp, uint8_t adr_value)
{
  sprintf((char*)buffer,"%.3f",*temp);
    /* Reset time, start counting microseconds */
      TM_DELAY_SetTime(0);
      buffer[6] = (uint8_t)adr_value;
      /* Transmit data, goes automatically to TX mode */
      
       TM_NRF24L01_Transmit((uint8_t*)buffer);
       //TM_NRF24L01_Transmit((uint8_t*)adr_value);
      
      /* Wait for data to be sent */
      do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
            //printf("\r\nTransmition Ok\r\n");
      } 
      while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
      printf("\r\nbuffer : %c",buffer[6]);
       memset(buffer,'\0',7); // C_buff �޸� �ʱ�ȭ  
}

void nRF24_Transmit_Set_Point(int* temp, uint8_t adr_value)
{
  sprintf((char*)buffer,"%d",*temp);
    /* Reset time, start counting microseconds */
      TM_DELAY_SetTime(0);
      buffer[6] = (uint8_t)adr_value;
      /* Transmit data, goes automatically to TX mode */
      
       TM_NRF24L01_Transmit((uint8_t*)buffer);
       //TM_NRF24L01_Transmit((uint8_t*)adr_value);
      
      /* Wait for data to be sent */
      do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
            //printf("\r\nTransmition Ok\r\n");
      } 
      while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
      //printf("\r\nbuffer : %c",buffer[6]);
       memset(buffer,'\0',7); // C_buff �޸� �ʱ�ȭ
}

// ===============Qick Test�� ���� q ����===============//
void nRF24_Transmit_ASCII(uint8_t adr_value)
{
     TM_DELAY_SetTime(0);
     buffer[6] = (uint8_t)adr_value;
     TM_NRF24L01_Transmit((uint8_t*)buffer);
     
     do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
      } 
      while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
      printf("\r\nbuffer : %c",buffer[6]);
       memset(tempbuf,'\0',7); // C_buff �޸� �ʱ�ȭ
}

void nRF24_Transmit_Debug_Mode(uint8_t adr_value)
{
  TM_DELAY_SetTime(0);
  buffer[6]=adr_value;
  TM_NRF24L01_Transmit((uint8_t*)buffer);
  
  do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
      } 
      while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
      memset(buffer,'\0',7); // C_buff �޸� �ʱ�ȭ
  
}

void nRF24_Receive()
{
  TM_NRF24L01_GetData(r_temp);
   
  printf("receive_echo : %s\r\n",dataIn);
  HAL_Delay(300);
}

void nRF24_Transmit_Echo()
{
  sprintf((char*)dataOut,"%s",dataIn);
      TM_NRF24L01_Transmit(dataOut);

      while (!TM_NRF24L01_DataReady() && TM_DELAY_Time() < 100);
      do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
      } 
      while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
}

void nRF24_Transmit_Status()
{
   
  /* Check transmit status */
      if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) 
      {
        /* Transmit went OK */
        printf("\r\nOK   %d \r\n",count);
        //return 1;        
      }
       else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) 
       {
         /* Message was LOST */
           printf("\r\nLOST \r\n");
          // return 0; 
       }
      else {
	/* This should never happen */
         printf("\r\nSEND \r\n");
      } 
}



void Keyboard_Debug(float* temp,uint8_t key_flag)
{
  char save_flag = 0;
  
  //char set_flag;
 // static char i;
  int j=0;

  //while(!(key_input==0x0D))
  while(!save_flag)
  {
    while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
    {
      if(key_input==0x0D)// ���� �Է� ������ ����
      {
        key_input='\0';                 //Ű ���� �ʱ�ȭ
        C_buff[6]='\0';
        sprintf((char*)tempbuf,"%s", C_buff);
        printf("\r\n save : %s",tempbuf);
        
        memset(C_buff,'\0',7); // C_buff �޸� �ʱ�ȭ

        C_count=0;//���� ī���� �ʱ�ȭ
        save_flag=1;
        break;
      }
      else if(key_flag<='9' || key_flag>='1')           // key_input���� 1~9�� �޾��� ��
      {
        if(key_input <= '9' && key_input >= '0' ||key_input == '.' )
        {
          
          if(C_count<6)// 6���� ���ϸ� �Է� ����
          {
            C_buff[C_count]=key_input;
            ++C_count;
            printf("%c",C_buff[j++]);
            if(C_count==6)
            {
              printf("\r\n");
              printf("Modify data : %s",C_buff);
            }
          }
         
          *temp = atof((char*)C_buff);
            key_input ='\0';
            key_flag = '\0';
            
         }
      }
      else if(key_flag=='0')                            // key_input���� 0�� �޾�����
      {
        if(key_input == '0')
       {
         if(C_count<2)   // 2���� ���ϸ� �Է� ����
         {
           C_buff[C_count]=key_input;
           ++C_count;
           printf("%c",C_buff[j++]);
           if(C_count==2)
           {
             printf("\r\nModify data : %s",C_buff);
           }
         }
         *temp = atoi((char*)C_buff);
           key_input ='\0';
           key_flag = '\0';
       }
      }
    }
  }
 }

void Keyboard_Debug_Set_Point(int* temp)
{
  char save_flag = 0;
  
  //char set_flag;
 // static char i;
  int j=0;

  //while(!(key_input==0x0D))
  while(!save_flag)
  {
    while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
    {
      if(key_input==0x0D)// ���� �Է� ������ ����
      {
        key_input='\0';                 //Ű ���� �ʱ�ȭ
        C_buff[6]='\0';
        sprintf((char*)tempbuf,"%s", C_buff);
        printf("\r\n save : %s",tempbuf);
        
        memset(C_buff,'\0',7); // C_buff �޸� �ʱ�ȭ

        C_count=0;//���� ī���� �ʱ�ȭ
        save_flag=1;
        break;
      }
      
        else if(key_input <= '9' && key_input >= '0')
        {
          
          if(C_count<5)// 5���� ���ϸ� �Է� ����
          {
            C_buff[C_count]=key_input;
            ++C_count;
            printf("%c",C_buff[j++]);
            if(C_count==5)
            {
              printf("\r\n");
              printf("Modify data : %s",C_buff);
            }
          }
         
          *temp = atoi((char*)C_buff);
            key_input ='\0';
         }
    }
  }
 }



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

//char str[40];
//uint32_t ADC_DATA[2];
//uint8_t Exit_flag=0;
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
  TM_NRF24L01_Init(15,32);
        TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_2M, TM_NRF24L01_OutputPower_M18dBm);
	
	/* Set my address, 5 bytes */
	TM_NRF24L01_SetMyAddress(MyAddress);
	
	/* Set TX address, 5 bytes */
	TM_NRF24L01_SetTxAddress(TxAddress);
	
	/* Reset counter */
	TM_DELAY_SetTime(2001);
        
        /* Reset counter */
	//TM_DELAY_SetTime(2001);
   
     
    //HAL_UART_Receive_IT(&huart2,(uint8_t*)key_input, 1);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   

//    printf("%d",count);
//    HAL_Delay(300);
    //count++;
    
 //----------------------ADC-----------------------//
//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, 100);
//    ADC_DATA[0] = HAL_ADC_GetValue(&hadc1);
//    sprintf((char*)buffer,"adc0 : %d",ADC_DATA[0]);
//    HAL_UART_Transmit(&huart2,buffer, sizeof(buffer),10);
//    
//    TM_NRF24L01_Transmit(buffer);
//    
//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, 100);
//    ADC_DATA[1] = HAL_ADC_GetValue(&hadc1);
//    sprintf((char*)buffer,"adc1 : %d \r\n",ADC_DATA[1]);
//    HAL_UART_Transmit(&huart2,buffer, sizeof(buffer),10);
//    
//    TM_NRF24L01_Transmit(buffer);
//    HAL_Delay(300);
 //----------------------ADC-----------------------//
    
//      nRF24_Transmit();
//    
//      
//      /* Go back to RX mode */
//      TM_NRF24L01_PowerUpRx();
//      
//      /* Wait received data, wait max 100ms, if time is larger, then data were probably lost */
//      while (!TM_NRF24L01_DataReady() && TM_DELAY_Time() < 100);
//      
//      /* Get data from NRF2L01+ */
//      nRF24_Receive();
//      
// //-----------------------------ECHO TEST------------------------------//
//      nRF24_Transmit_Echo();
//      
//      HAL_Delay(1000);
// //------------------------------------------------------------------//
//      nRF24_Transmit_Status();

      while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))              // Ű���� �ԷµǱ� �������� while���� �����ִ�. 
      {
          if(key_input == 's' || key_input == 'S')              // ���� ���� ����
          {
            for(int i =0; i<3;i++)
            {
              nRF24_Transmit_Set_Point(&Set_Point[i],key_tr);
            }
            printf("Set_Roll : %d\r\nSet_Pitch : %d\r\nSet_Yaw : %d\r\n",Set_Point[0],Set_Point[1],Set_Point[2]);
            
          }
          else if(key_input == 'd' || key_input == 'D')      //����� ���� ����
          {
          // ������κ��� �����͸� ���� ������ ���� ui �Լ��� ���
            printf("\r\n  --------------Enter Debugging Mode---------------\r\n");
            Display_UI();
            nRF24_Transmit_Debug_Mode(key_input);
            key_input='\0';
            while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK));
            switch(key_input)
            {
              case '0':
                  key_tr = key_input;
                  key_input='\0';
                  printf("Throttle: ");
                  
                  Keyboard_Debug(&Throttle,key_tr);
                  nRF24_Transmit(&Throttle,key_tr);
                  nRF24_Transmit_Status();
                   
                  key_tr = '\0';
                  break;
                  
                case '1':
                  key_tr = key_input;
                  key_input='\0';
                  printf("R_P Data: ");
                  
                  Keyboard_Debug(&Roll_P,key_tr);
                  nRF24_Transmit(&Roll_P,key_tr);
                  nRF24_Transmit_Status();
                   //Qick_NRF24L01_Transmit(&Roll_P);
                    //TM_NRF24L01_PowerUpRx();
    //                TM_NRF24L01_GetData(r_temp);
    //                 temp_1 = atof((char*)r_temp);
    //                 printf("r_temp : %s\r\n", r_temp);
    //                 printf("receive : %.3f\r\n", temp_1);
    //                  r_temp[32] = '\0';
                    
                    
//                   //���� ���н� ������ �� ���� ����
//                    while(transmissionStatus==TM_NRF24L01_Transmit_Status_Lost)
//                    {
//                        fail_count++;
//                        if(fail_count >= 1000)
//                        {
//                          printf("\r\n Transmiton Fail\r\n");
//                        }
//                        else
//                        {
//                          nRF24_Transmit(&Roll_P,key_input);
//                          printf("\r\n Transmiton Successfully\r\n");
//                        }
//                        fail_count = 1;
//                        break;  
//                    }
                    
                  key_tr = '\0';
                  break;
                  
                case '2':
                 key_tr = key_input;
                 key_input='\0';
                 printf("P_P Data: ");
            
                 Keyboard_Debug(&Pitch_P,key_tr);
                 nRF24_Transmit(&Pitch_P,key_tr);
                 nRF24_Transmit_Status();
                  
                 key_tr = '\0';
                 break;
                 
                case '3':
                 key_tr = key_input;
                 key_input='\0';
                 printf("Y_P Data: ");
                
                 Keyboard_Debug(&Yaw_P,key_tr);
                 nRF24_Transmit(&Yaw_P,key_tr);
                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                 
                case '4':
                 key_tr = key_input;
                 key_input='\0';
                 printf("R_I Data: ");
                 
                 Keyboard_Debug(&Roll_I,key_tr);
                 nRF24_Transmit(&Roll_I,key_tr);
                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                 
                case '5':
                 key_tr = key_input;
                 key_input='\0';
                 printf("P_I Data: ");
                 
                 Keyboard_Debug(&Pitch_I,key_tr);
                 nRF24_Transmit(&Pitch_I,key_tr);
                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                 
                case '6':
                 key_tr = key_input;
                 key_input='\0';
                 printf("Y_I Data: ");
                 
                 Keyboard_Debug(&Yaw_I,key_tr);
                 nRF24_Transmit(&Yaw_I,key_tr);

                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                 
                case '7':
                 key_tr = key_input;
                 key_input='\0';
                 printf("R_D Data: ");
                 
                 Keyboard_Debug(&Roll_D,key_tr);
                 nRF24_Transmit(&Roll_D,key_tr);
                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                 
                case '8':
                 key_tr = key_input;
                 key_input='\0';
                 printf("P_D Data: ");
                 
                 Keyboard_Debug(&Pitch_D,key_tr);
                 nRF24_Transmit(&Pitch_D,key_tr);
                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                 
                case '9':
                 key_tr = key_input;
                 key_input='\0';
                 printf("Y_D Data: ");
                 
                 Keyboard_Debug(&Yaw_D,key_tr);
                 nRF24_Transmit(&Yaw_D,key_tr);
                 nRF24_Transmit_Status();
                 key_tr = '\0';
                 break;
                
              case 'q':
                key_tr = key_input;
                key_input='\0';
                printf("Qick_Test : ");
                  
                 while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
                  {
                    //Keyboard_Debug(&Qick_q);
                    
                    if(count <= 1000)
                    {
                      nRF24_Transmit_ASCII(key_tr);
                      HAL_Delay(1);
                      nRF24_Transmit_Status();
                       //key_tr ='\0';
                      count++;
                    }
                  }
                  count=1;
                 key_tr='\0';
                 break;
                 
            case 'r':
              key_tr = key_input;
              key_input = '\0';
              printf("Roll_SetPoint : ");
              
              Keyboard_Debug_Set_Point(&Set_Point[0]);
              nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
              nRF24_Transmit_Status();
              key_tr ='\0';
              break;
              
            case 'p':
              key_tr = key_input;
              key_input = '\0';
              printf("Pitch_SetPoint : ");
              
              Keyboard_Debug_Set_Point(&Set_Point[1]);
              nRF24_Transmit_Set_Point(&Set_Point[1],key_tr);
              nRF24_Transmit_Status();
              key_tr ='\0';
              break;
              
            case 'y':
              key_tr = key_input;
              key_input = '\0';
              printf("Yaw_SetPoint : ");
              
              Keyboard_Debug_Set_Point(&Set_Point[2]);
              nRF24_Transmit_Set_Point(&Set_Point[2],key_tr);
              nRF24_Transmit_Status();
              key_tr ='\0';
              break;
                
              default:
                printf("\r\nError Number\r\n");
                break;
          }
          key_input = 0;
        
        }
    }
    
    
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

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
  hadc1.Init.NbrOfConversion = 1;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSNpin_Pin|CEpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CSNpin_Pin CEpin_Pin */
  GPIO_InitStruct.Pin = CSNpin_Pin|CEpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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