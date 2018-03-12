#ifndef I2C_h
#define I2C_h

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "delay.h"
#include "stm32f10x_i2c.h"

void init_I2C1(void);
void I2C_StartTransmission(I2C_TypeDef* I2Cx, uint8_t transmissionDirection,  uint8_t slaveAddress);
void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_ReadData(I2C_TypeDef* I2Cx);

// EEPROM
void I2C_EE_ByteWrite(uint16_t WriteAddr, uint8_t val);
uint8_t I2C_EE_ByteRead( uint16_t ReadAddr);
void I2C_init_EE();

#endif