#include "LPF.h"

#define tau     0.01f
#define tau2     0.0001f

void __LPF(float * LPF_Euler_angle, float * Euler_angle, float * preEuler_angle, float deltat)
{
    LPF_Euler_angle[0] = (tau * preEuler_angle[0] + deltat * Euler_angle[0]) / (tau + deltat);
    LPF_Euler_angle[1] = (tau * preEuler_angle[1] + deltat * Euler_angle[1]) / (tau + deltat);
    LPF_Euler_angle[2] = (tau * preEuler_angle[2] + deltat * Euler_angle[2]) / (tau + deltat);
    
//    LPF_Euler_angle[0] = (tau * preEuler_angle[0] + (2.0f/1000.0f) * Euler_angle[0]) / (tau + (2.0f/1000.0f));
//    LPF_Euler_angle[1] = (tau * preEuler_angle[1] + (2.0f/1000.0f) * Euler_angle[1]) / (tau + (2.0f/1000.0f));
//    LPF_Euler_angle[2] = (tau * preEuler_angle[2] + (2.0f/1000.0f) * Euler_angle[2]) / (tau + (2.0f/1000.0f));

    preEuler_angle[0] = LPF_Euler_angle[0];
    preEuler_angle[1] = LPF_Euler_angle[1];
    preEuler_angle[2] = LPF_Euler_angle[2];
}

void __LPFGyro(float * LPF_Gyro, TM_MPU9250_t* MPU9250, float * preGyro, float deltat)
{
    LPF_Gyro[0] = (tau * preGyro[0] + deltat * MPU9250->Gx) / (tau + deltat);
    LPF_Gyro[1] = (tau * preGyro[1] + deltat * MPU9250->Gy) / (tau + deltat);
    LPF_Gyro[2] = (tau * preGyro[2] + deltat * MPU9250->Gz) / (tau + deltat);
    
    preGyro[0] = LPF_Gyro[0];
    preGyro[1] = LPF_Gyro[1];
    preGyro[2] = LPF_Gyro[2];
}