 void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt)
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = acc;
	gyro.x = gyro.x * DEG2RAD;
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;
  
	/* if the three-axis accelerometer is operating, then use Mahony filter*/
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;
  
		/*through vector cross product to show measured attitude error of 
               gyrpscope */
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
  
		/*intergration of error*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
  
		/*PI controller to fuse data*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
  
	/*update quaternion through Runge–Kutta methods in discrete system*/
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
  
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
  
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
  
	/*calculate rotation matrix*/
	imuComputeRotationMatrix();	
  
	/*calculate euler angle from quaternion*/
	state->attitude.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
	state->attitude.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	state->attitude.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
}