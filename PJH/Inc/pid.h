#ifndef __PID_H__
#define __PID_H__

typedef struct PID
{
	float err[3]; 
	float err_last[3]; 
	float Kp,Ki,Kd;
	float output[3];
	float integral[3];
}__PID;


void pid_init(struct PID * pid, float p, float i, float d);
void pid_update(__PID * pid, float set, float actual, float deltat, int axis);
void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float deltat);


#endif

