#include "main.h"
#include "nRF_Transmit.h"
#include <stdint.h>

TM_NRF24L01_Transmit_Status_t transmissionStatus;
uint8_t buffer[8];
extern int count;

//  =============PID DATA TRANSMIT============= //
void nRF24_Transmit(float* temp, uint8_t adr_value)                     
{
  int No_Connection=0;

  sprintf((char*)buffer,"%.3f",*temp);
    /* Reset time, start counting microseconds */
  TM_DELAY_SetTime(0); 
  buffer[6] = (uint8_t)adr_value;
  
 /* Transmit data, goes automatically to TX mode */
  TM_NRF24L01_Transmit((uint8_t*)buffer);
      
 /* Wait for data to be sent */

  do {
  /* Get transmission status */
      transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
      if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
      {
        No_Connection++;
        TM_NRF24L01_Transmit((uint8_t*)buffer);
        nRF24_Transmit_Status();
        if(No_Connection >= 1000)
        {
          //printf("No Connection!\r\n");
          break;
        }
      }
  }
  
  while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
  printf("\r\nbuffer : %c",buffer[6]);
  memset(buffer,'\0',7); // C_buff 메모리 초기화  
}

// =============ADC DATA TRANSMIT============= //

void nRF24_Transmit_ADC(int8_t* temp, uint8_t adr_value)                
{
  int No_Connection=0;
  memset(buffer,'\0',7); // C_buff 메모리 초기화  
  sprintf((char*)buffer,"%d",*temp);
  
    /* Reset time, start counting microseconds */
  TM_DELAY_SetTime(0);
  buffer[6] = (uint8_t)adr_value; 
 /* Transmit data, goes automatically to TX mode */
  TM_NRF24L01_Transmit((uint8_t*)buffer);
    
 /* Wait for data to be sent */

  do {
  /* Get transmission status */
      transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
      if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
      {
        No_Connection++;
        TM_NRF24L01_Transmit((uint8_t*)buffer);
        //nRF24_Transmit_Status();
        if(No_Connection >= 300)
        {
          //printf("No Connection!\r\n");
          break;
        }
      }
  }while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
  
}

//  =============SETPOINT DATA TRANSMIT============= //
void nRF24_Transmit_Set_Point(int* temp, uint8_t adr_value)
{
  int No_Connection=0;
  sprintf((char*)buffer,"%d",*temp);
    /* Reset time, start counting microseconds */
  TM_DELAY_SetTime(0);
  buffer[6] = (uint8_t)adr_value;
  /* Transmit data, goes automatically to TX mode */
  
  TM_NRF24L01_Transmit((uint8_t*)buffer);
  
  /* Wait for data to be sent */
  do {
  /* Get transmission status */
      transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
      if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
      {
        No_Connection++;
        TM_NRF24L01_Transmit((uint8_t*)buffer);
        //nRF24_Transmit_Status();
        if(No_Connection >= 300)
        {
          //printf("No Connection!\r\n");
          break;
        }
      }
  }
  
  while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
  memset(buffer,'\0',7); // C_buff 메모리 초기화
}

// ===============Qick Test를 위한 q 전송===============//
void nRF24_Transmit_ASCII(uint8_t adr_value)
{
  int No_Connection=0;
  memset(buffer,'\0',7); // C_buff 메모리 초기화    
  TM_DELAY_SetTime(0);
  buffer[6] = (uint8_t)adr_value;
  TM_NRF24L01_Transmit((uint8_t*)buffer);

  do {
     /* Get transmission status */
         transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
         No_Connection++;
          if(No_Connection >= 1000)
          {
            printf("No Connection!\r\n");
            break;
          }
   } 
  while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
  printf("\r\nbuffer : %c",buffer[6]);
  HAL_Delay(10);
}

// =============MODE CHANGE KEY DATA TRANSMIT============= //
void nRF24_Transmit_Mode_Change(uint8_t adr_value)
{
  int No_Connection=0;
  TM_DELAY_SetTime(0);
  buffer[6]=adr_value;
  TM_NRF24L01_Transmit((uint8_t*)buffer);
  
  do {
  /* Get transmission status */
      transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
//      if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
//      {
//        No_Connection++;
//        TM_NRF24L01_Transmit((uint8_t*)buffer);
//        //nRF24_Transmit_Status();
//        if(No_Connection >= 1000)
//        {
//          //printf("No Connection!\r\n");
//          break;
//        }
//      }
  } while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
  memset(buffer,'\0',7); // C_buff 메모리 초기화
  
}

// ==================TRANSMIT STATUS CHECK================= //
void nRF24_Transmit_Status()
{
  /* Check transmit status */
      if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) 
      {
        /* Transmit went OK */
        printf("\r\nOK   %d\r\n",count);      
      }
      else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) 
      {
         /* Message was LOST */
           printf("\r\nLOST \r\n");
      }
      else {
	/* This should never happen */
         printf("\r\nSEND \r\n");
      } 
}