// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones. 

#include "Quaternion.h"

#define Kp      (2.0f * 6.0f) // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki      (2.0f * 0.005f)
#define PI      (3.141592f)

extern float q[4];
extern float eInt[3];// vector to hold integral error for Mahony method
extern float deltat;// integration interval for both filter schemes
extern float Euler_angle[3];

#if 0
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;   

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    }
    else
    {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
#else
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float norm;
    float tx, ty, tz, tw;
    float hx, hy, hz, by;
    float f0, f1, f2, f3, f4, f5;
    float ex, ey, ez;
    float qDotW, qDotX, qDotY, qDotZ;

    // Auxiliary variables to avoid repeated arithmetic
    float qxqx = qx * qx;
    float qxqz = qx * qz;
    float qyqy = qy * qy;
    float qyqz = qy * qz;
    float qzqz = qz * qz;
    float qwqw = qw * qw;
    float qwqy = qw * qy;
    float qwqx = qw * qx;
    float qxqy = qx * qy;
    float qwqz = qw * qz;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    mx /= norm;
    my /= norm;
    mz /= norm;

    // Reference direction of Earth's magnetic feild
    tw = -mx*-qx - my*-qy - mz*-qz;
    tx =  mx* qw + my*-qz - mz*-qy;
    ty = -mx*-qz + my* qw + mz*-qx;
    tz =  mx*-qy - my*-qx + mz* qw;

    hx = qw*tx + qx*tw + qy*tz - qz*ty;
    hy = qw*ty - qx*tz + qy*tw + qz*tx;
    hz = qw*tz + qx*ty - qy*tx + qz*tw;
    by = sqrtf(hx*hx + hy*hy);

    // Gradient decent algorithm corrective step
    f0 = 2.0f *       (qxqz - qwqy);
    f1 = 2.0f *       (qwqx + qyqz);
    f2 = qwqw - qxqx - qyqy + qzqz;

    f3 = 2.0f * by * (0.5f - qyqy - qzqz) + 2.0f * hz *        (qxqz - qwqy);
    f4 = 2.0f * by *        (qxqy - qwqz) + 2.0f * hz *        (qwqx + qyqz);
    f5 = 2.0f * by *        (qwqy + qxqz) + 2.0f * hz * (0.5f - qxqx - qyqy);

    // Error is sum of cross product between estimated direction and measured direction of fields
    ex = (ay * f2 - az * f1) + (my * f5 - mz * f4);
    ey = (az * f0 - ax * f2) + (mz * f3 - mx * f5);
    ez = (ax * f1 - ay * f0) + (mx * f4 - my * f3);

    if(Ki > 0.0f){
      eInt[0] += ex * deltat;
      eInt[1] += ey * deltat;   
      eInt[2] += ez * deltat;  
    }else{
      eInt[0] = 0;
      eInt[1] = 0;
      eInt[2] = 0;
    }

      // Apply feedback terms
    gx += Kp * ex + Ki * eInt[0];
    gy += Kp * ey + Ki * eInt[1];
    gz += Kp * ez + Ki * eInt[2];  


      // Compute rate of change of quaternion
    qDotW = 0.5f * -qx*gx - qy*gy - qz*gz;
    qDotX = 0.5f *  qw*gx + qy*gz - qz*gy;
    qDotY = 0.5f *  qw*gy - qx*gz + qz*gx;
    qDotZ = 0.5f *  qw*gz + qx*gy - qy*gx;

    // Integrate to yield quaternion
    qw += qDotW * deltat;
    qx += qDotX * deltat;
    qy += qDotY * deltat;
    qz += qDotZ * deltat;

    // Normalise quaternion
    norm = sqrtf(qw*qw + qx*qx +qy*qy + qz*qz);
    q[0] = qw / norm;
    q[1] = qx / norm;
    q[2] = qy / norm;
    q[3] = qz / norm; 
}

#endif

void Quternion2Euler(float *q)
{
    float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[0] * q[2] - q[3] * q[1]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    
    Euler_angle[0] = atan2f(a31, a33);
    //Euler_angle[0] = atanf(a31 / a33);
    Euler_angle[1] = asinf(a32);
    Euler_angle[2] = atan2f(a12, a22);
    Euler_angle[0] *= 180.0f / PI;
    Euler_angle[1] *= 180.0f / PI;
    Euler_angle[2] *= 180.0f / PI; 
}