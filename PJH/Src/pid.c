#include "pid.h"

void pid_init(struct PID * pid, float p, float i, float d)
{	
	pid->err[0]		= 0.0;	
        pid->err[1]		= 0.0;	
	pid->err[2]		= 0.0;	

	pid->integral[0]	= 0.0;
        pid->integral[1]	= 0.0;
	pid->integral[2]	= 0.0;

	pid->err_last[0]	= 0.0;
        pid->err_last[1]	= 0.0;
	pid->err_last[2]	= 0.0;

	pid->output[0]		= 0.0;
        pid->output[1]		= 0.0;
	pid->output[2]		= 0.0;
	
	pid->Kp			= p;
	pid->Ki			= i;
	pid->Kd			= d;
}


void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float deltat)
{
  pid_update(pid, setting_angle[0], Euler_angle[0], deltat, 0);
  pid_update(pid, setting_angle[1], Euler_angle[1], deltat, 1);
  //pid_update(pid, setting_angle[2], Euler_angle[2], deltat, 2);
}

void pid_update(__PID * pid, float set, float actual, float deltat, int axis)
{  
  float Kp_term, Ki_term, Kd_term;
  float D_err = 0.0f;
  switch (axis)
  {
  case 0: //roll
    {
        pid->err[0] = set - actual; //���� = ��ǥġ - ���簪	
        Kp_term = pid->Kp * pid->err[0]; //p�� = Kp * ����
        
        pid->integral[0] += pid->err[0] * deltat;//�������� = �������� + ���� * dt
        Ki_term = pid->Ki * pid->integral[0];//i�� = Ki * ��������
        
        D_err = (pid->err[0] - pid->err_last[0]) / deltat;//�����̺� = (�������-��������)/dt
        Kd_term = pid->Kd * D_err;//d�� = Kd * �����̺�
	
	pid->output[0] = Kp_term + Ki_term + Kd_term;//��� = Kp�� + Ki�� + Kd��
	pid->err_last[0] = pid->err[0];//��������� ����������.
      break;
    }
  case 1: //pitch
    {
        pid->err[1] = set - actual; //���� = ��ǥġ - ���簪	
        Kp_term = pid->Kp * pid->err[1]; //p�� = Kp * ����
        
        pid->integral[1] += pid->err[1] * deltat;//�������� = �������� + ���� * dt
        Ki_term = pid->Ki * pid->integral[1];//i�� = Ki * ��������
        
        D_err = (pid->err[1] - pid->err_last[1]) / deltat;//�����̺� = (�������-��������)/dt
        Kd_term = pid->Kd * D_err;//d�� = Kd * �����̺�
	
	pid->output[1] = Kp_term + Ki_term + Kd_term;//��� = Kp�� + Ki�� + Kd��
	pid->err_last[1] = pid->err[1];//��������� ����������.
      break;
    }
  case 2:  //yaw
    {
        pid->err[2] = set - actual; //���� = ��ǥġ - ���簪	
        Kp_term = pid->Kp * pid->err[2]; //p�� = Kp * ����
        
        pid->integral[2] += pid->err[2] * deltat;//�������� = �������� + ���� * dt
        Ki_term = pid->Ki * pid->integral[2];//i�� = Ki * ��������
        
        D_err = (pid->err[2] - pid->err_last[2]) / deltat;//�����̺� = (�������-��������)/dt
        Kd_term = pid->Kd * D_err;//d�� = Kd * �����̺�
	
	pid->output[2] = Kp_term + Ki_term + Kd_term;//��� = Kp�� + Ki�� + Kd��
	pid->err_last[2] = pid->err[2];//��������� ����������.
      break;
    }
  }
	
}

