#ifndef Barometer_H
#define Barometer_H

#include "defines.h"
#include "stm32f4xx_hal.h"
#include "tm_stm32_i2c.h"

#define MS561101BA

#ifdef MS561101BA

#ifndef MS561101BA_I2C
#define MS561101BA_I2C             I2C1
#define MS561101BA_I2C_PP          TM_I2C_PinsPack_2
#endif

#define MS561101BA_I2C_CLOCK       400000

// addresses of the device
#define MS561101BA_ADDR_CSB_HIGH  (0x76 << 1)   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW   (0x77 << 1)   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

// #define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 8 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
// #define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_COUNT 8 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

#define SeaLevelPressure    (1013.25)

#define Average_Count   15//

typedef struct _GY63_MS5611{
  uint8_t I2C_MS5611Addr;
  int64_t OFF, OFF2, SENS, SENS2;
  int32_t dT, P, TEMP, T2, refPressure;
  uint32_t D1, D2;
  uint16_t calibrationData[6];
  uint8_t buffer[5];
  
  double realTEMP, realPressure;
  double Altitude;
  double SeaLevel;
    //ADD YSH
  double Avr_Altitude[Average_Count];
  double Altitude_sum;
}GY63_MS5611_strc;

void GY63_MS5611_Init(GY63_MS5611_strc * GY63_MS5611);
void GY63_MS5611_Reset(GY63_MS5611_strc * GY63_MS5611);
void GY63_MS5611_PROM(GY63_MS5611_strc *GY63_MS5611);
void GY63_MS5611_getRefPressure(GY63_MS5611_strc *GY63_MS5611);
void GY63_MS5611_getPressure(GY63_MS5611_strc * GY63_MS5611);
void getAltitude(GY63_MS5611_strc *GY63_MS5611);
void getSeaLevel(GY63_MS5611_strc *GY63_MS5611);

  //ADD YSH
uint8_t AverageAltitude(GY63_MS5611_strc *GY63_MS5611);

#endif // MS561101BA

#endif // Barometer_H