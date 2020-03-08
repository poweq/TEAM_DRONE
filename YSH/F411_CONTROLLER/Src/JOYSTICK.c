#include "JOYSTICK.h"


 /*=======Print Mode==================*/
void Print_Mode(uint8_t* Mode_Flag)
{
    uint8_t buffer_str[50];
    memset(buffer_str,'\0',50);
      if(*Mode_Flag==Debug_Mode_Flag)
      {
        *Mode_Flag= Drive_Mode_Flag;
        sprintf((char*)buffer_str,"\r\n =======DEBUG MODE======\r\n"); 
        HAL_UART_Transmit(&huart2,buffer_str, sizeof(buffer_str),10);
        memset(buffer_str,'\0',50);
      }
     
    
   else if(*Mode_Flag==Drive_Mode_Flag)
      {
        *Mode_Flag= Debug_Mode_Flag;
       sprintf((char*)buffer_str,"\r\n =======DRIVER MODE======\r\n"); 
       HAL_UART_Transmit(&huart2,buffer_str, sizeof(buffer_str),10);
        memset(buffer_str,'\0',50);
      }
}

 /*=======Print ADC DEBUG==================*/
void Print_ADC_DEBUG()
{
    uint8_t buffer_ADC[50];
    memset(buffer_ADC,'\0',50);
    sprintf((char*)buffer_ADC,"%4d / %4d  / %4d / %4d\r\n",ADC_DATA[0], ADC_DATA[1],ADC_DATA[2], ADC_DATA[3]); 
    HAL_UART_Transmit(&huart2,buffer_ADC, sizeof(buffer_ADC),10);
}



 /*=======INPUT THROTTLE DATA==================*/
char INPUT_THROTTLE()
{
  static char count = 0; 
  uint8_t TH_Str_Buff[50];//THROTTLE BUFF
  memset(TH_Str_Buff,'\0',sizeof(TH_Str_Buff));
    if(ADC_DATA[3]>3000)//UP
      {
      sprintf((char*)TH_Str_Buff,"\r\nTH : %d",count++);
      HAL_UART_Transmit(&huart2,(uint8_t*)TH_Str_Buff, sizeof(TH_Str_Buff),10);
      }
      else if(ADC_DATA[3]<500)//DOWN
    {
      sprintf((char*)TH_Str_Buff,"\r\nTH : %d ",count--);
      HAL_UART_Transmit(&huart2,(uint8_t*)TH_Str_Buff, sizeof(TH_Str_Buff),10);
    }
 

      if(ADC_DATA[2]>3000)//LEFT
    {
      sprintf((char*)TH_Str_Buff," \r\nTH : %d",count = count+10);
      HAL_UART_Transmit(&huart2,(uint8_t*)TH_Str_Buff, sizeof(TH_Str_Buff),10);
    }
        else if(ADC_DATA[2]<500)//RIGHT
    {
      sprintf((char*)TH_Str_Buff," \r\nTH : %d ",count = count-10);
      HAL_UART_Transmit(&huart2,(uint8_t*)TH_Str_Buff, sizeof(TH_Str_Buff),10);
    }
    memset(TH_Str_Buff,'\0',sizeof(TH_Str_Buff));
    
    return 0;
}


 /*=======INPUT JOYSTICK DATA==================*/
char INPUT_CONTORLLER()
{
  static char count= 0; 
    uint8_t JS_Str_Buff[50]; //JOYSTICK BUFF    
    memset(JS_Str_Buff,'\0',sizeof(JS_Str_Buff));
    
   if(ADC_DATA[0]>3000)//UP
      {
      sprintf((char*)JS_Str_Buff,"\r\n%d",count++);
      HAL_UART_Transmit(&huart2,(uint8_t*)JS_Str_Buff, sizeof(JS_Str_Buff),10);
      }
      else if(ADC_DATA[0]<500)//DOWN
    {
      sprintf((char*)JS_Str_Buff,"\r\n%d ",count--);
      HAL_UART_Transmit(&huart2,(uint8_t*)JS_Str_Buff, sizeof(JS_Str_Buff),10);
    }

      if(ADC_DATA[1]>3000)//LEFT
    {
      sprintf((char*)JS_Str_Buff," \r\n%d",count = count+10);
      HAL_UART_Transmit(&huart2,(uint8_t*)JS_Str_Buff, sizeof(JS_Str_Buff),10);
    }
       else if(ADC_DATA[1]<500)//RIGHT
    {
      sprintf((char*)JS_Str_Buff," \r\n%d ",count = count-10);
      HAL_UART_Transmit(&huart2,(uint8_t*)JS_Str_Buff, sizeof(JS_Str_Buff),10);
    }
    memset(JS_Str_Buff,'\0',50);
    return 0;
}

