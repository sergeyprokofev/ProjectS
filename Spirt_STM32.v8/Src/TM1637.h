#ifndef TM1637_H_
#define TM1637_H_
#include "stm32f10x_gpio.h"

void TN1637_init(void);
void TM1637SetBrightness( GPIO_TypeDef * port, int _CLK, int _DIO, unsigned char brightness);
//void TM1637Start(void);
//void TM1637Stop(void);
//void TM1637ReadAck(void);
//void TM1637WriteByte(unsigned char b);
void TM1637DisplayDecimal( GPIO_TypeDef * port, int _CLK, int _DIO, unsigned int v, unsigned char displaySeparator);


#endif 
