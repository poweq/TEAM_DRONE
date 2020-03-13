#include "LPF.h"

#define tau     0.01f

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