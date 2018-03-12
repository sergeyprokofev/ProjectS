//#include "stm32f10x_type.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "Delay.h"

#define Max_I2C_delay_cls			100000
#define I2C_ERROR				0xFFFF
#define I2C_BUS_BUSY				0xFFEE

#define EE24_Write_Delay_ms 		5

void EE24Init(I2C_TypeDef* I2Cx);
int EE24WriteByte(I2C_TypeDef* I2Cx, char val, int WriteAddr);
uint8_t EE24ReadByte(I2C_TypeDef* I2Cx, int ReadAddr);
char I2C_Wait_Event(I2C_TypeDef* I2Cx, long event);
void EE24PrintString(I2C_TypeDef* I2Cx, char * string, int adrs);
