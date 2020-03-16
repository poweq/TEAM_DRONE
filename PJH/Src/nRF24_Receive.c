#include "main.h"
#include "nRF24_Receive.h"


//========================nRF24L01 FUNCTION==============================
void NRF24_Data_save(int* Throttle,float temp, int temp_int, int value, __PID*  pid, float* setting_angle, int* Throttle2)
{
  switch(value)
  {
    case 't':
      *Throttle = temp_int;
      printf("Throttle : %d\r\n",temp_int);                                      // ����Ʋ �� ����
      value = '\0';
      break;
      
    case 1:
      pid->iKp[0] = temp;
      //inpid_val[0][0] = temp;

      printf("Roll_P[0][0] : %.3f\r\n",temp);                                 // Roll_P �� ����
      value = '\0';
      break;
      
     case 2:
       pid->iKp[1] = temp;
      //inpid_val[1][0] = temp;
      printf("Pitch_P[1][0] : %.3f\r\n",temp);                               // Pitch_P �� ����
      value = '\0';
      break;
      
     case 3:
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
      printf("Yaw_P[2][0] : %.3f\r\n",temp);                                // Yaw_P �� ����
      value = '\0';
      break;
      
     case 4:
       pid->iKi[0] = temp;
      //inpid_val[0][1] = temp;
      printf("Roll_I[0][1] : %.3f\r\n",temp);                                 // Roll_I �� ����
      value = '\0';
      break;
      
     case 5:
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      printf("Pitch_I[1][1] : %.3f\r\n",temp);                               // Pitch_I �� ����
      value = '\0';
      break;
      
     case 6:
       pid->iKi[2] = temp;
      //inpid_val[2][1] = temp;
      printf("Yaw_I[2][1] : %.3f\r\n",temp);                                 // Yaw_I �� ����
      value = '\0';
      break;
      
     case 7:
       pid->iKd[0] = temp;
      //inpid_val[0][2] = temp;
      printf("Roll_D[0][2] : %.3f\r\n",temp);                                // Roll_D �� ����
      value = '\0';
      break;
      
     case 8:
       pid->iKd[1] = temp;
      //inpid_val[1][2] = temp;
      printf("Pitch_D[1][2] : %.3f\r\n",temp);                              // Pitch_D �� ����
      value = '\0';
      break;
      
     case 9:
       pid->iKd[2] = temp;
      //inpid_val[2][2] = temp;
      printf("Yaw_D[2][2] : %.3f\r\n",temp);                               // Yaw_D �� ����
      value = '\0';
      break;
      
      // ============================== OUT PID DATA ============================== //    
    case 'A':
      pid->Kp[0] = temp;
      //inpid_val[0][0] = temp;
      printf("OUT_Roll_P : %.3f\r\n",temp);                                 // Roll_P �� ����
      value = '\0';
      break;
      
     case 'B':
       pid->Kp[1] = temp;
      //inpid_val[1][0] = temp;
      printf("OUT_Pitch_P : %.3f\r\n",temp);                               // Pitch_P �� ����
      value = '\0';
      break;
      
     case 'C':
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
      printf("OUT_Yaw_P : %.3f\r\n",temp);                                // Yaw_P �� ����
      value = '\0';
      break;
      
     case 'D':
       pid->Ki[0] = temp;
      //inpid_val[0][1] = temp;
      printf("OUT_Roll_I : %.3f\r\n",temp);                                 // Roll_I �� ����
      value = '\0';
      break;
      
     case 'E':
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      printf("OUT_Pitch_I : %.3f\r\n",temp);                               // Pitch_I �� ����
      value = '\0';
      break;
      
     case 'F':
       pid->Ki[2] = temp;
      //inpid_val[2][1] = temp;
      printf("OUT_Yaw_I : %.3f\r\n",temp);                                 // Yaw_I �� ����
      value = '\0';
      break;
      
     case 'G':
       pid->Kd[0] = temp;
      //inpid_val[0][2] = temp;
      printf("OUT_Roll_D : %.3f\r\n",temp);                                // Roll_D �� ����
      value = '\0';
      break;
      
     case 'H':
       pid->Kd[1] = temp;
      //inpid_val[1][2] = temp;
      printf("OUT_Pitch_D : %.3f\r\n",temp);                              // Pitch_D �� ����
      value = '\0';
      break;
      
     case 'I':
       pid->Kd[2] = temp;
      //inpid_val[2][2] = temp;
      printf("OUT_Yaw_D : %.3f\r\n",temp);                               // Yaw_D �� ����
      value = '\0';
      break;
      
   // ========================================================================= //  
      
     case 'r':
      setting_angle[0] = (float)((int8_t)temp);
      printf("Roll_Set_Point : %.0f\r\n",setting_angle[0]);            // Roll_SetPoint �� ����
      value = '\0';
      break;
       
     case 'p':
      setting_angle[1] = (float)((int8_t)temp);
      printf("Pitch_Set_Point : %.0f\r\n",setting_angle[1]);           // Pitch_SetPoint �� ����
      value = '\0';
      break;
      
     case 'y':
      //setting_angle[2] = (float)((int8_t)temp);
      *Throttle2 = (int)((int8_t)temp);

      printf("Yaw_Set_Point : %d\r\n",*Throttle2);           // Yaw_SetPoint �� ����
      value = '\0';
      break;
      
    case 'x':
      *Throttle = temp_int;
      printf("Throttle_Reset : %d\r\n",temp_int);                                     // ����Ʋ �� ����(Throttle = MIN_PULSE)
      value = '\0';
      break;
      
     case 's':
       value = '\0';
       break;
  }
}

void NRF24_Receive(int* Throttle,float temp, int temp_int,__PID*  pid,float* setting_angle, int* Throttle2)    // Controller���� PID�� ����
{
  int value='\0';     // ��Ʈ�ѷ����� ���� key_input �� ���� ����
  uint8_t dataIn[8]={0};                                     // Controller Data Receive Buffer

  //printf("Receive_Data_Ready : %d\r\n",TM_NRF24L01_DataReady());
  
  if(TM_NRF24L01_DataReady())
  {
   // printf("Receive_Data_Ok : %d\r\n",TM_NRF24L01_DataReady());
       
      TM_NRF24L01_GetData(dataIn);
      
        if(dataIn[6]=='q'||dataIn[6]=='Q')      // q ������ �� �� �׽�Ʈ
        {
          value = dataIn[6];
          dataIn[6]='\0';
          printf("\r\n[6] : %c\r\n",value);
        }
        
        else if(dataIn[6] == 'd'||dataIn[6]=='D')       // d ���� �Ǿ��� �� ����� ���
        {
          value = dataIn[6];
          dataIn[6] = '\0'; 
          temp_int=atoi((char*)dataIn);
          //temp_int = 0;
          //printf("\r\n dataIn_test : %d\r\n",temp_int);
        }
        
        else if(dataIn[6] == 's')                     // s ���� �Ǿ��� �� ���� ��� (��Ʈ�ѷ����� s ������ ����� ���� Roll, Pitch, Yaw ���� ��Ʈ�ѷ� �� �۽��ϰ� ������ ����)
        {
          value = dataIn[6];
          dataIn[6] = '\0';
          //printf("key_input : %c", value);
        }
        
        else if(dataIn[6] == 'x')                                             // x ���� �Ǿ��� ��  Throttle�� �ʱ�ȭ
        {
          value = dataIn[6];
          dataIn[6] = '\0'; 
          temp_int=atoi((char*)dataIn);
          temp_int = 0;
        }
        
        else if(dataIn[6] == 'r'||dataIn[6]=='R')                       // r, R ���� �Ǿ��� �� Roll SetPoint �� ����
        {
          value = dataIn[6];
          dataIn[6] = '\0';
          temp = atof((char*)dataIn);                                       // uint8_t ������ ���ŵ� Roll SetPoint ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
        
        else if(dataIn[6] == 'p'||dataIn[6]=='P')                       // p, P ���� �Ǿ��� �� Pitch SetPoint �� ����
        {
          value = dataIn[6];
          dataIn[6] = '\0';
          temp = atof((char*)dataIn);                                        // uint8_t ������ ���ŵ� Pitch SetPoint ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
        
        else if(dataIn[6] == 'y'||dataIn[6]=='Y')                        // y, Y ���� �Ǿ��� �� Yaw SetPoint �� ����
        {
          value = dataIn[6];
          dataIn[6] = '\0';
          temp = atoi((char*)dataIn);                                       // uint8_t ������ ���ŵ� Yaw SetPoint ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
 
        else if(dataIn[6]<='9' && dataIn[6]>='1')                     // 1~9�� ���� �Ǿ��� �� pid�� ����
        {
          value = dataIn[6]-48;
          dataIn[6]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t ������ ���ŵ� PID ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.3f\r\n",temp);
        }
        
        else if(dataIn[6]<='I' && dataIn[6]>='A')
        {
          value = dataIn[6];
          dataIn[6]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t ������ ���ŵ� PID ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.3f\r\n",temp);
        }
        
        
        else if(dataIn[6]=='t')                                               // 0�� ������ �� Throttle�� ����
        {
          value = dataIn[6];  
          dataIn[6]='\0';
          temp_int=atoi((char*)dataIn);                                     //  uint8_t ������ ���ŵ� Throttle ���� int������ ��ȯ�Ͽ� temp_init�� ����
          //printf("\r\n dataIn_test : %d\r\n",temp_int);
//          if(temp_int>5)
//          {
//            MOTOR_V1 = 8000+(Controller_1*70);
//            MOTOR_V2 = 8000+(Controller_1*70);
//            MOTOR_V3 = 8000+(Controller_1*70);
//            MOTOR_V4 = 8000+(Controller_1*70);
//          }
        }
//        else
//        {
//          
//          
//        }
        
      NRF24_Data_save(Throttle,temp,temp_int,value,pid,setting_angle,Throttle2);        //  ���ŵ� ������ ���� �Լ�
      TM_NRF24L01_PowerUpRx();
  }
}
//===================================================