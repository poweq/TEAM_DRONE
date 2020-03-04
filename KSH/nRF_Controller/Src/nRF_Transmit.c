#include "main.h"
#include "nRF_Transmit.h"

TM_NRF24L01_Transmit_Status_t transmissionStatus;
uint8_t buffer[7];
extern int count;

void nRF24_Transmit(float* temp, uint8_t adr_value)
{
  //char Re_transmit=0;
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
      //printf("transmission : %d\r\n",transmissionStatus);
  } 
  while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
 
  printf("\r\nbuffer : %c",buffer[6]);
  memset(buffer,'\0',7); // C_buff 메모리 초기화  
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
  memset(buffer,'\0',7); // C_buff 메모리 초기화
}

// ===============Qick Test를 위한 q 전송===============//
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
  
  adr_value='\0';
  printf("\r\nbuffer : %c",buffer[6]);
  HAL_Delay(1);
  memset(buffer,'\0',7); // C_buff 메모리 초기화    
}

void nRF24_Transmit_Mode_Change(uint8_t adr_value)
{
  TM_DELAY_SetTime(0);
  buffer[6]=adr_value;
  TM_NRF24L01_Transmit((uint8_t*)buffer);
  
  do {
        /* Get transmission status */
        transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
  } 
  while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
  memset(buffer,'\0',7); // C_buff 메모리 초기화
}

void nRF24_Transmit_Status()
{
  /* Check transmit status */
      if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) 
      {
        /* Transmit went OK */
        printf("\r\nOK   %d\r\n",count);
        printf("transmission : %d\r\n",transmissionStatus);
        //return 1;        
      }
      else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) 
      {
         /* Message was LOST */
           printf("\r\nLOST \r\n");
           printf("transmission : %d\r\n",transmissionStatus);
          // return 0; 
      }
      else {
	/* This should never happen */
         printf("\r\nSEND \r\n");
         printf("transmission : %d\r\n",transmissionStatus);
      } 
}