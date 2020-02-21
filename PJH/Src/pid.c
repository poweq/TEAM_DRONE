

#include "pid.h"

#define PID_IMAX             (40.0f)
#define PID_IMIN              (-40.0f)

extern float deltat;

void pid_init(__PID * pid, float pid_val[][3], float inpid_val[][3])
{	
	pid->err[0]		= 0.0f;	
        pid->err[1]		= 0.0f;	
	pid->err[2]		= 0.0f;	

        pid->rateError[0]     = 0.0f;
        pid->rateError[1]     = 0.0f;
        pid->rateError[2]     = 0.0f;

	pid->integral[0]	        = 0.0f;
        pid->integral[1]	        = 0.0f;
	pid->integral[2]	        = 0.0f;

	pid->err_last[0]	        = 0.0f;
        pid->err_last[1]	        = 0.0f;
	pid->err_last[2]	        = 0.0f;
        
        pid->preRateError[0]    = 0.0f;
        pid->preRateError[1]    = 0.0f;
        pid->preRateError[2]    = 0.0f;
        
	pid->output[0]		= 0.0f;
        pid->output[1]		= 0.0f;
	pid->output[2]		= 0.0f;
	
	pid->Kp[0]		= pid_val[0][0];//outer Roll PID
	pid->Ki[0]		        = pid_val[0][1];
	pid->Kd[0]		= pid_val[0][2];
        
        pid->Kp[1]		= pid_val[1][0];//outer Pitch PID
	pid->Ki[1]		        = pid_val[1][1];
	pid->Kd[1]		= pid_val[1][2];
        
        pid->Kp[2]		= pid_val[2][0];//outer Yaw PID
	pid->Ki[2]		        = pid_val[2][1];
	pid->Kd[2]		= pid_val[2][2];
        
        pid->iKp[0]		= inpid_val[0][0];//inner Roll PID
	pid->iKi[0]		= inpid_val[0][1];
	pid->iKd[0]		= inpid_val[0][2];
        
        pid->iKp[1]		= inpid_val[1][0];//inner Pitch PID
	pid->iKi[1]		= inpid_val[1][1];
	pid->iKd[1]		= inpid_val[1][2];
        
        pid->iKp[2]		= inpid_val[2][0];//inner Yaw PID
	pid->iKi[2]		= inpid_val[2][1];
	pid->iKd[2]		= inpid_val[2][2];
}
void pid_gain_update(__PID * pid, float pid_val[][3], float inpid_val[][3])
{
        pid->Kp[0]		= pid_val[0][0];//outer Roll PID
	pid->Ki[0]		        = pid_val[0][1];
	pid->Kd[0]		= pid_val[0][2];
        
        pid->Kp[1]		= pid_val[1][0];//outer Pitch PID
	pid->Ki[1]		        = pid_val[1][1];
	pid->Kd[1]		= pid_val[1][2];
        
        pid->Kp[2]		= pid_val[2][0];//outer Yaw PID
	pid->Ki[2]		        = pid_val[2][1];
	pid->Kd[2]		= pid_val[2][2];
}

void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float * angular_velocity)
{
  pid_update(pid, setting_angle[0], Euler_angle[0], angular_velocity[0], 0);
  pid_update(pid, setting_angle[1], Euler_angle[1], angular_velocity[1], 1);
  pid_update(pid, setting_angle[2], Euler_angle[2], angular_velocity[2], 2);
}
#if 1 //double loop PID 
void pid_update(__PID * pid, float set, float actual, float angular_velocity, int axis)
{  
  //set 목표각도
  //actual 현재각도
  //angular_velocity 현재 각속도
  float Kp_term, Ki_term, Kd_term;
  float D_err = 0.0f;
  float stabilPgain = 0.0f;
  float stabilIgain = 0.0f;
   
    
  //angular_velocity = angular_velocity * deltat;                   //Degree/2ms(500Hz).
  switch (axis)
  {
  case 0: //roll
    {      
        pid->err[0] = set - actual;                                             //오차 = 목표치 - 현재값	

        if (pid->err[0] >  PID_IMAX)
        pid->err[0] = PID_IMAX;
        if (pid->err[0] < PID_IMIN)
        pid->err[0] = PID_IMIN;
        
        stabilPgain = pid->Kp[0] * pid->err[0];                                          //p항 = Kp * 오차
        stabilIgain += pid->Ki[0] *pid->err[0] * deltat;
          
        pid->rateError[0] = stabilPgain - angular_velocity;

        Kp_term = pid->iKp[0] * pid->rateError[0];                                     //p항 = KpGain * rate오차.        

        pid->integral[0] += (pid->rateError[0] * deltat);                              //오차적분 = 오차적분 + (오차rate * dt).        
        Ki_term = pid->iKi[0] * pid->integral[0];                                         //i항 = Ki * 오차적분.        

        D_err = (pid->rateError[0] - pid->preRateError[0]) / deltat;             //오차미분 = (현재rate오차-이전rate오차)/dt.        
        Kd_term = pid->iKd[0] * D_err;                                                       //d항 = Kd * rate오차미분.	
        
        pid->output[0] = Kp_term + Ki_term + Kd_term + stabilIgain;                               //제어량 = Kp항 + Ki항 + Kd항

        pid->preRateError[0] = pid->rateError[0];                                      //현재rate오차를 이전rate오차로.
        
      break;
    }
  case 1: //pitch
    {
        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	

        if (pid->err[1] >  PID_IMAX)
        pid->err[1] = PID_IMAX;
        if (pid->err[1] < PID_IMIN)
        pid->err[1] = PID_IMIN;
        
        stabilPgain = pid->Kp[1] * pid->err[1];                                          //p항 = Kp * 오차
        stabilIgain += pid->Ki[0] *pid->err[0] * deltat;

        
        pid->rateError[1] =  stabilPgain - angular_velocity;

        Kp_term = pid->iKp[1] * pid->rateError[1];                                     //p항 = KpGain * rate오차.        

        pid->integral[1] += (pid->rateError[1] * deltat);                              //오차적분 = 오차적분 + 오차rate * dt.        
        Ki_term = pid->iKi[1] * pid->integral[1];                                         //i항 = Ki * 오차적분.        

        D_err = (pid->rateError[1] - pid->preRateError[1]) / deltat;             //오차미분 = (현재rate오차-이전rate오차)/dt.        
        Kd_term = pid->iKd[1] * D_err;                                                       //d항 = Kd * rate오차미분.	
        
        pid->output[1] = Kp_term + Ki_term + Kd_term + stabilIgain;                               //제어량 = Kp항 + Ki항 + Kd항

        pid->preRateError[1] = pid->rateError[1];                                      //현재rate오차를 이전rate오차로.

        
      break;
    }
  case 2:  //yaw
    {
        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	

        if (pid->err[2] >  PID_IMAX)
        pid->err[2] = PID_IMAX;
        if (pid->err[2] < PID_IMIN)
        pid->err[2] = PID_IMIN;
        
        stabilPgain = pid->Kp[2] * pid->err[2];                                           //p항 = Kp * 오차
        stabilIgain += pid->Ki[0] *pid->err[0] * deltat;

        pid->rateError[2] = stabilPgain - angular_velocity;

        Kp_term = pid->iKp[2] * pid->rateError[2];                                     //p항 = KpGain * rate오차.        

        pid->integral[2] += (pid->rateError[2] * deltat);                              //오차적분 = 오차적분 + 오차rate * dt.        
        Ki_term = pid->iKi[2] * pid->integral[2];                                         //i항 = Ki * 오차적분.        

        D_err = (pid->rateError[2] - pid->preRateError[2]) / deltat;             //오차미분 = (현재rate오차-이전rate오차)/dt.        
        Kd_term = pid->iKd[2] * D_err;                                                       //d항 = Kd * rate오차미분.	
        
        pid->output[2] = Kp_term + Ki_term + Kd_term + stabilIgain;                               //제어량 = Kp항 + Ki항 + Kd항

        pid->preRateError[2] = pid->rateError[2];                                      //현재rate오차를 이전rate오차로.

      break;
    }
  }
	
}
#else

void pid_update(__PID * pid, float set, float actual,float angular_velocity, int axis)
{  
  float Kp_term, Ki_term, Kd_term;
  float D_err = 0.0f;
  switch (axis)
  {
  case 0: //roll
    {
        pid->err[0] = set - actual; //오차 = 목표치 - 현재값	
        if (pid->err[0] >  PID_IMAX)
          pid->err[0] = PID_IMAX;
        if (pid->err[0] < PID_IMIN)
          pid->err[0] = PID_IMIN;
        Kp_term = pid->Kp[0] * pid->err[0]; //p항 = Kp * 오차
        
        pid->integral[0] += pid->err[0] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[0] * pid->integral[0];//i항 = Ki * 오차적분
        
        D_err = (pid->err[0] - pid->err_last[0]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[0] * D_err;//d항 = Kd * 오차미분
	
	pid->output[0] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[0] = pid->err[0];//현재오차를 이전오차로.
      break;
    }
  case 1: //pitch
    {
        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	
        if (pid->err[1] >  PID_IMAX)
          pid->err[1] = PID_IMAX;
        if (pid->err[1] < PID_IMIN)
          pid->err[1] = PID_IMIN;
        Kp_term = pid->Kp[1] * pid->err[1]; //p항 = Kp * 오차
        
        pid->integral[1] += pid->err[1] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[1] * pid->integral[1];//i항 = Ki * 오차적분
        
        D_err = (pid->err[1] - pid->err_last[1]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[1] * D_err;//d항 = Kd * 오차미분
	
	pid->output[1] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[1] = pid->err[1];//현재오차를 이전오차로.
      break;
    }
  case 2:  //yaw
    {
        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	
        
        if (pid->err[2] < -180)                 //correct angle jump ( ex: 180 -> -180)
          pid->err[2] = pid->err[2] + 360;
        else if (pid->err[2] > 180)
          pid->err[2] = pid->err[2] - 360;
        
        if (pid->err[2] >  PID_IMAX)
          pid->err[2] = PID_IMAX;
        if (pid->err[2] < PID_IMIN)
          pid->err[2] = PID_IMIN;
        Kp_term = pid->Kp[2] * pid->err[2]; //p항 = Kp * 오차
        
        pid->integral[2] += pid->err[2] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[2] * pid->integral[2];//i항 = Ki * 오차적분
        
        D_err = (pid->err[2] - pid->err_last[2]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[2] * D_err;//d항 = Kd * 오차미분
	
	pid->output[2] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[2] = pid->err[2];//현재오차를 이전오차로.
      break;
    }
  }	
}
#endif

float Standard_pid_update(__PID * pid, float set, float actual, int axis)
{  
  float Kp_term, Ki_term, Kd_term;
  float D_err = 0.0f;
  float stabilizedKp = 0.0f;
  switch (axis)
  {
  case 0: //roll
    {
        pid->err[0] = set - actual; //오차 = 목표치 - 현재값	
        
        if (pid->err[0] >  PID_IMAX)
          pid->err[0] = PID_IMAX;
        if (pid->err[0] < PID_IMIN)
          pid->err[0] = PID_IMIN;
        
        Kp_term = pid->Kp[0] * pid->err[0]; //p항 = Kp * 오차
        
        //pid->integral[0] += pid->err[0] * deltat;//오차적분 = 오차적분 + 오차 * dt
        //Ki_term = pid->Ki[0] * pid->integral[0];//i항 = Ki * 오차적분
        
        //D_err = (pid->err[0] - pid->err_last[0]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        //Kd_term = pid->Kd[0] * D_err;//d항 = Kd * 오차미분
	
        stabilizedKp = Kp_term;
	//pid->output[0] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	//pid->err_last[0] = pid->err[0];//현재오차를 이전오차로.
      break;
    }
  case 1: //pitch
    {
        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	
        if (pid->err[1] >  PID_IMAX)
          pid->err[1] = PID_IMAX;
        if (pid->err[1] < PID_IMIN)
          pid->err[1] = PID_IMIN;
        Kp_term = pid->Kp[1] * pid->err[1]; //p항 = Kp * 오차
        
        pid->integral[1] += pid->err[1] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[1] * pid->integral[1];//i항 = Ki * 오차적분
        
        D_err = (pid->err[1] - pid->err_last[1]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[1] * D_err;//d항 = Kd * 오차미분
	
	pid->output[1] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[1] = pid->err[1];//현재오차를 이전오차로.
      break;
    }
  case 2:  //yaw
    {
        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	
        
        if (pid->err[2] < -180)                 //correct angle jump ( ex: 180 -> -180)
          pid->err[2] = pid->err[2] + 360;
        else if (pid->err[2] > 180)
          pid->err[2] = pid->err[2] - 360;
        
        if (pid->err[2] >  PID_IMAX)
          pid->err[2] = PID_IMAX;
        if (pid->err[2] < PID_IMIN)
          pid->err[2] = PID_IMIN;
        Kp_term = pid->Kp[2] * pid->err[2]; //p항 = Kp * 오차
        
        pid->integral[2] += pid->err[2] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[2] * pid->integral[2];//i항 = Ki * 오차적분
        
        D_err = (pid->err[2] - pid->err_last[2]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[2] * D_err;//d항 = Kd * 오차미분
	
	pid->output[2] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[2] = pid->err[2];//현재오차를 이전오차로.
      break;
    }
  }
  return stabilizedKp;
}

void Parsing_PID_val(uint8_t* arr, float pid_val[][3])
{
    char* R_PP, * R_PI, * R_PD, * R_P, * R_I, * R_D, * R_YP, * R_YI, * R_YD;

    R_PP = strtok((char*)arr, "PP");
    R_PI = strtok(NULL, "PI");
    R_PD = strtok(NULL, "PD");
    R_P = strtok(NULL, "PRP");
    R_I = strtok(NULL, "RI");
    R_D = strtok(NULL, "RD");
    R_YP = strtok(NULL, "RYP");
    R_YI = strtok(NULL, "YI");
    R_YD = strtok(NULL, "YD");
     
    pid_val[0][0] = atof(R_P);
    pid_val[0][1] = atof(R_I);
    pid_val[0][2] = atof(R_D);
    pid_val[1][0] = atof(R_PP);
    pid_val[1][1] = atof(R_PI);
    pid_val[1][2] = atof(R_PD);
    pid_val[2][0] = atof(R_YP);
    pid_val[2][1] = atof(R_YI);
    pid_val[2][2] = atof(R_YD);
}
