/*
 * GccApplication1.c
 *
 * Created: 2020-01-15 오전 11:20:55
 * Author : bitcamp
 */ 
#include "I2C.h"


void I2C_init()
{
	DDRC |= (1 << I2C_SCL);
	DDRC |= (1 << I2C_SDA);
	TWBR = 32;
	TWCR = (1 << TWEN) | (1 << TWEA);
}

void I2C_start()
{
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) );
}

void I2C_transmit(uint8_t data)
{
	TWDR = data;
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)));
}

uint8_t I2C_receive_ACK(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t I2C_receive_NACK()
{
	TWCR = _BV(TWINT) | _BV(TWEN);
	while( !(TWCR & (1 << TWINT)));
	return TWDR;
}

void I2C_stop()
{
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWEA);
}


