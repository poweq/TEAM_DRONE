#include "Barometer.h"

void GY63_MS5611_Init(GY63_MS5611_strc *GY63_MS5611)
{
    GY63_MS5611->I2C_MS5611Addr = MS561101BA_ADDR_CSB_LOW;
    GY63_MS5611->OFF = 0;
    GY63_MS5611->OFF2 = 0;
    GY63_MS5611->SENS = 0;
    GY63_MS5611->SENS2 = 0;
    GY63_MS5611->dT = 0;
    GY63_MS5611->P = 0;
    GY63_MS5611->TEMP = 0;
    GY63_MS5611->T2 = 0;
    GY63_MS5611->D1 = 0;
    GY63_MS5611->D2 = 0;
    GY63_MS5611->refPressure = 0;

    for(int i=0;i<5;i++){
      GY63_MS5611->buffer[i] = 0;
    }
    for(int i=0;i<6;i++){
      GY63_MS5611->calibrationData[i] = 0;
    }
    
    GY63_MS5611->realTEMP = 0;
    GY63_MS5611->realPressure = 0;
    GY63_MS5611->Altitude = 0;
    GY63_MS5611->SeaLevel = 0;
    
    TM_I2C_Init(MS561101BA_I2C, MS561101BA_I2C_PP, MS561101BA_I2C_CLOCK);
    
    GY63_MS5611_Reset(GY63_MS5611);
    GY63_MS5611_PROM(GY63_MS5611);
}

void GY63_MS5611_Reset(GY63_MS5611_strc *GY63_MS5611)
{        
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_RESET);
	Delayms(15); //delay 10 mS needed for device to execute reset	
}

void GY63_MS5611_PROM(GY63_MS5611_strc *GY63_MS5611)
{
    for (int i=0;i<6;i++){
//        TM_I2C_Read16(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_PROM_BASE_ADDR + (i+1)*2, GY63_MS5611->buffer);
        TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_PROM_BASE_ADDR + (i+1)*2);
        TM_I2C_ReadMultiNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, GY63_MS5611->buffer, 2);//read all 14 bytes for callibration data from PROM
        Delayms(1);
        //delay(50); //at least 40 uS
        GY63_MS5611->calibrationData[i] = GY63_MS5611->buffer[0]<<8 | GY63_MS5611->buffer[1]; //pair of bytes goes into each element of callibrationData[i], global variables, 14 uint8_t into 7 uint16_t
    }
}
void GY63_MS5611_getRefPressure(GY63_MS5611_strc *GY63_MS5611)
{
	GY63_MS5611->D1=0;GY63_MS5611->D2=0;
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_D1 + MS561101BA_OSR_4096); //set D2 OSR=4096 (overscan, maximum) 0x48.
	Delayms(20);
    //delay(25000);//must be 15 mS or more
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, 0x00);//initiate and read ADC data, 3 bytes
    TM_I2C_ReadMultiNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, GY63_MS5611->buffer, 3);
	GY63_MS5611->D1 = GY63_MS5611->D1<<8 | GY63_MS5611->buffer[0]; //shifting first MSB byte left
	GY63_MS5611->D1 = GY63_MS5611->D1<<8 | GY63_MS5611->buffer[1]; //another byte
	GY63_MS5611->D1 = GY63_MS5611->D1<<8 | GY63_MS5611->buffer[2]; //LSB byte last
    
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_D2 + MS561101BA_OSR_4096); //set D2 OSR=4096 (overscan, maximum) 0x58.
	Delayms(20);
    //delay(25000); //must be 15 mS or more
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, 0x00); //initiate and read ADC data, 3 bytes.
    TM_I2C_ReadMultiNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, GY63_MS5611->buffer, 3);
	GY63_MS5611->D2 = GY63_MS5611->D2<<8 | GY63_MS5611->buffer[0]; //shifting first MSB byte left
	GY63_MS5611->D2 = GY63_MS5611->D2<<8 | GY63_MS5611->buffer[1]; //another byte
	GY63_MS5611->D2 = GY63_MS5611->D2<<8 | GY63_MS5611->buffer[2]; //LSB byte last
		
	GY63_MS5611->dT = GY63_MS5611->D2 - ((int32_t)GY63_MS5611->calibrationData[4] << 8);
    GY63_MS5611->TEMP = (2000 + (((int32_t)GY63_MS5611->dT * (int32_t)GY63_MS5611->calibrationData[5]) >> 23)); //temperature before second order compensation
    if (GY63_MS5611->TEMP < 2000)  //if temperature of the sensor goes below 20°C, it activates "second order temperature compensation"
    {
        GY63_MS5611->T2 = ((int64_t)pow(GY63_MS5611->dT,2)) >> 31;
        GY63_MS5611->OFF2 = ((int64_t)(5*pow((GY63_MS5611->TEMP-2000),2))) >> 1;
        GY63_MS5611->SENS2 = ((int64_t)(5*pow((GY63_MS5611->TEMP-2000),2))) >> 2;
        if (GY63_MS5611->TEMP < -1500) //if temperature of the sensor goes even lower, below -15°C, then additional math is utilized
        {
            GY63_MS5611->OFF2 = (int64_t)(GY63_MS5611->OFF2 + 7*pow((GY63_MS5611->TEMP+1500),2));
            GY63_MS5611->SENS2 = (int64_t)(GY63_MS5611->SENS2 + 11*pow((GY63_MS5611->TEMP+1500),2)/2);
        }
    }
    else 
        {
            GY63_MS5611->T2 = 0;
            GY63_MS5611->OFF2 = 0;
            GY63_MS5611->SENS2 = 0;
        }
    GY63_MS5611->TEMP = ((2000 + (((int64_t)GY63_MS5611->dT * (int64_t)GY63_MS5611->calibrationData[5]) >> 23))-GY63_MS5611->T2); //second order compensation included
    GY63_MS5611->OFF = (((int64_t)GY63_MS5611->calibrationData[1] << 16) + (((int64_t)GY63_MS5611->calibrationData[3] * GY63_MS5611->dT) >> 7) - GY63_MS5611->OFF2); //second order compensation included
    GY63_MS5611->SENS = (((int64_t)GY63_MS5611->calibrationData[0] << 15) + (((int64_t)GY63_MS5611->calibrationData[2] * GY63_MS5611->dT) >> 8) - GY63_MS5611->SENS2); //second order compensation included
    GY63_MS5611->refPressure = (((GY63_MS5611->D1 * GY63_MS5611->SENS) >> 21) - GY63_MS5611->OFF) >> 15; 
}


void GY63_MS5611_getPressure(GY63_MS5611_strc *GY63_MS5611)
{
	GY63_MS5611->D1=0;GY63_MS5611->D2=0;
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_D1 + MS561101BA_OSR_4096); //set D2 OSR=4096 (overscan, maximum) 0x48.
	Delayms(20);
    //delay(25000);//must be 15 mS or more
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, 0x00);//initiate and read ADC data, 3 bytes
    TM_I2C_ReadMultiNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, GY63_MS5611->buffer, 3);
	GY63_MS5611->D1 = GY63_MS5611->D1<<8 | GY63_MS5611->buffer[0]; //shifting first MSB byte left
	GY63_MS5611->D1 = GY63_MS5611->D1<<8 | GY63_MS5611->buffer[1]; //another byte
	GY63_MS5611->D1 = GY63_MS5611->D1<<8 | GY63_MS5611->buffer[2]; //LSB byte last
    
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, MS561101BA_D2 + MS561101BA_OSR_4096); //set D2 OSR=4096 (overscan, maximum) 0x58.
	Delayms(20);
    //delay(25000); //must be 15 mS or more
    TM_I2C_WriteNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, 0x00); //initiate and read ADC data, 3 bytes.
    TM_I2C_ReadMultiNoRegister(MS561101BA_I2C, GY63_MS5611->I2C_MS5611Addr, GY63_MS5611->buffer, 3);
	GY63_MS5611->D2 = GY63_MS5611->D2<<8 | GY63_MS5611->buffer[0]; //shifting first MSB byte left
	GY63_MS5611->D2 = GY63_MS5611->D2<<8 | GY63_MS5611->buffer[1]; //another byte
	GY63_MS5611->D2 = GY63_MS5611->D2<<8 | GY63_MS5611->buffer[2]; //LSB byte last
		
	GY63_MS5611->dT = GY63_MS5611->D2 - ((int32_t)GY63_MS5611->calibrationData[4] << 8);
    GY63_MS5611->TEMP = (2000 + (((int32_t)GY63_MS5611->dT * (int32_t)GY63_MS5611->calibrationData[5]) >> 23)); //temperature before second order compensation
    //64->32 
    
    if (GY63_MS5611->TEMP < 2000)  //if temperature of the sensor goes below 20°C, it activates "second order temperature compensation"
    {
        //GY63_MS5611->T2 = pow(GY63_MS5611->dT,2)/2147483648;
        GY63_MS5611->T2 = ((int64_t)pow(GY63_MS5611->dT,2)) >> 31;
        //GY63_MS5611->OFF2 = 5*pow((GY63_MS5611->TEMP-2000),2)/2;
        GY63_MS5611->OFF2 = ((int64_t)(5*pow((GY63_MS5611->TEMP-2000),2))) >> 1;
        //GY63_MS5611->SENS2 = 5*pow((GY63_MS5611->TEMP-2000),2)/4;
        GY63_MS5611->SENS2 = ((int64_t)(5*pow((GY63_MS5611->TEMP-2000),2))) >> 2;
        if (GY63_MS5611->TEMP < -1500) //if temperature of the sensor goes even lower, below -15°C, then additional math is utilized
        {
            //GY63_MS5611->OFF2 = GY63_MS5611->OFF2+7*pow((GY63_MS5611->TEMP+1500),2);
            GY63_MS5611->OFF2 = (int64_t)(GY63_MS5611->OFF2 + 7*pow((GY63_MS5611->TEMP+1500),2));
            //GY63_MS5611->SENS2 = GY63_MS5611->SENS2+11*pow((GY63_MS5611->TEMP+1500),2)/2;
            GY63_MS5611->SENS2 = (int64_t)(GY63_MS5611->SENS2 + 11*pow((GY63_MS5611->TEMP+1500),2)/2);
        }
    }
    else 
        {
            GY63_MS5611->T2 = 0;
            GY63_MS5611->OFF2 = 0;
            GY63_MS5611->SENS2 = 0;
        }
//    GY63_MS5611->TEMP = ((2000 + (((int64_t)GY63_MS5611->dT * (int64_t)GY63_MS5611->calibrationData[5]) >> 23))-GY63_MS5611->T2); //second order compensation included
//    GY63_MS5611->OFF = (((unsigned int)GY63_MS5611->calibrationData[1] << 16) + (((int64_t)GY63_MS5611->calibrationData[3] * GY63_MS5611->dT) >> 7)-GY63_MS5611->OFF2); //second order compensation included
//    GY63_MS5611->SENS = (((unsigned int)GY63_MS5611->calibrationData[0] << 15) + (((int64_t)GY63_MS5611->calibrationData[2] * GY63_MS5611->dT) >> 8)-GY63_MS5611->SENS2); //second order compensation included
//    GY63_MS5611->P = (((GY63_MS5611->D1 * GY63_MS5611->SENS) >> 21) - GY63_MS5611->OFF) >> 15;  
    GY63_MS5611->TEMP = ((2000 + (((int64_t)GY63_MS5611->dT * (int64_t)GY63_MS5611->calibrationData[5]) >> 23))-GY63_MS5611->T2); //second order compensation included
    GY63_MS5611->OFF = (((int64_t)GY63_MS5611->calibrationData[1] << 16) + (((int64_t)GY63_MS5611->calibrationData[3] * GY63_MS5611->dT) >> 7) - GY63_MS5611->OFF2); //second order compensation included
    GY63_MS5611->SENS = (((int64_t)GY63_MS5611->calibrationData[0] << 15) + (((int64_t)GY63_MS5611->calibrationData[2] * GY63_MS5611->dT) >> 8) - GY63_MS5611->SENS2); //second order compensation included
    GY63_MS5611->P = (((GY63_MS5611->D1 * GY63_MS5611->SENS) >> 21) - GY63_MS5611->OFF) >> 15; 
    GY63_MS5611->realTEMP = (double)(GY63_MS5611->TEMP)/100;
    GY63_MS5611->realPressure = (double)(GY63_MS5611->P)/100;
    //return P; //returns back pressure P
    getAltitude(GY63_MS5611);
    //getSeaLevel(GY63_MS5611);    
}

// Calculate altitude from Pressure & Sea level pressure
void getAltitude(GY63_MS5611_strc *GY63_MS5611)
{
  if(GY63_MS5611->refPressure > 0){
    GY63_MS5611->Altitude = (145366.45f * (1.0f - pow(((double)GY63_MS5611->P/100) / ((double)GY63_MS5611->refPressure/100), 0.190284f)))*30.48; //30.48 is for feet to cm.
  }
  else{
    GY63_MS5611->Altitude = (145366.45f * (1.0f - pow(((double)GY63_MS5611->P/100) / (double)SeaLevelPressure, 0.190284f)))*30.48;
  }
}

// Calculate sea level from Pressure given on specific altitude
void getSeaLevel(GY63_MS5611_strc *GY63_MS5611)
{
    GY63_MS5611->SeaLevel = ((double)GY63_MS5611->P/100 / pow(1.0f - ((double)GY63_MS5611->Altitude / 145366.45f), 5.257f));
}