#include "TM1637.h"
#include "stm32f10x.h"
#include "misc.h"
#include "Delay.h"
#include "stm32f10x_gpio.h"

#define uchar unsigned char
#define ushort unsigned short
#define ulong unsigned long
#define uint unsigned int

void TM1637Start(GPIO_TypeDef * port, int _CLK, int _DIO);
void TM1637Stop(GPIO_TypeDef * port, int _CLK, int _DIO);
void TM1637ReadAck(GPIO_TypeDef * port, int _CLK, int _DIO);
void TM1637WriteByte(GPIO_TypeDef * port, int _CLK, int _DIO,unsigned char b);

void TM1637ClkLow(GPIO_TypeDef * port, int _CLK, int _DIO);
void TM1637DatHigh(GPIO_TypeDef * port, int _CLK, int _DIO);
void TM1637DatLow(GPIO_TypeDef * port, int _CLK, int _DIO);
//void TM1637SetBrightness(unsigned char brightness);
/*
#define TM1637_CLK   GPIO_Pin_12  //PORTA
#define TM1637_DIO   GPIO_Pin_13  //PORTA

#define DEF_CLK   GPIO_Pin_12                             // PORTB  Индикатор температура
#define DEF_DIO   GPIO_Pin_13                             //    в узле отбора

#define WAT_CLK   GPIO_Pin_14                             // PORTB  Индикатор температура
#define WAT_DIO   GPIO_Pin_15                             //    в воды на выходе охлодителя

#define PWR_CLK   GPIO_Pin_8  //  PORTA Индикатор температура
#define PWR_DIO   GPIO_Pin_9  //    в воды на выходе охлодителя

#define OTB_CLK   GPIO_Pin_10  // PORTA  Индикатор температура
#define OTB_DIO   GPIO_Pin_11  //    в воды на выходе охлодителя

#define TM1637ClkHigh(); GPIO_SetBits(GPIOB, TM1637_CLK);
#define TM1637ClkLow();  GPIO_ResetBits(GPIOB, TM1637_CLK);
#define TM1637DatHigh(); GPIO_SetBits(GPIOB, TM1637_DIO);
#define TM1637DatLow();  GPIO_ResetBits(GPIOB, TM1637_DIO);
*/

const char segmentMap[] =
{
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00                                            // blank
};

void TN1637_init(void)
{
    /* Configure all unused GPIO port pins in Analog Input mode (floating input
                                 trigger OFF), this will reduce the power consumption and increase the device
                                 immunity against EMI/EMC *************************************************/
//  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);

//    //Indicator LED
//    GPIO_InitStructure.GPIO_Pin = LED;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);

//    //TM1637 4 digit seven segment
//    GPIO_InitStructure.GPIO_Pin = TM1637_CLK | TM1637_DIO;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

//    //DHT11 Humidity dan Temperature Sensor
//    GPIO_InitStructure.GPIO_Pin = DHT11_1Wire;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

    delay_ms(1000);
    //TM1637SetBrightness(7);
}


void TM1637SetBrightness(GPIO_TypeDef * port, int _CLK, int _DIO, unsigned char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness

    TM1637Start(port,_CLK, _DIO);
    TM1637WriteByte(port, _CLK, _DIO, 0x87 + brightness);
    TM1637ReadAck(port, _CLK, _DIO);
    TM1637Stop(port, _CLK,  _DIO);
}

void TM1637Start(GPIO_TypeDef * port, int _CLK, int _DIO)
{
    GPIO_SetBits(port, _CLK); //TM1637ClkHigh();
    GPIO_SetBits(port, _DIO); // TM1637DatHigh();
    delay_us(2);
    GPIO_ResetBits(port, _DIO); //TM1637DatLow();
}

void TM1637Stop(GPIO_TypeDef * port, int _CLK, int _DIO)
{
    GPIO_ResetBits(port, _CLK); // TM1637ClkLow();
    delay_us(2);
    GPIO_ResetBits(port, _DIO); // TM1637DatLow();
    delay_us(2);
    GPIO_SetBits(port, _CLK); // TM1637ClkHigh();
    delay_us(2);
    GPIO_SetBits(port, _DIO); // TM1637DatHigh();
}

void TM1637ReadAck(GPIO_TypeDef * port, int _CLK, int _DIO)
{
    GPIO_ResetBits(port, _CLK); // TM1637ClkLow();
    delay_us(5);
    // while (TM1637Dat); // Ignore the feedback
    GPIO_SetBits(port, _CLK); // TM1637ClkHigh();
    delay_us(2);
    GPIO_ResetBits(port, _CLK); // TM1637ClkLow();
}

void TM1637WriteByte(GPIO_TypeDef * port, int _CLK, int _DIO, unsigned char b)
{
    unsigned char i;
    for ( i = 0; i < 8; ++i)
    {
        GPIO_ResetBits(port, _CLK); // TM1637ClkLow();
        if (b & 0x01)
        {
            GPIO_SetBits(port, _DIO); // TM1637DatHigh();
        }
        else
        {
            GPIO_ResetBits(port, _DIO); // TM1637DatLow();
        }
	delay_us(3);
        b >>= 1;
        GPIO_SetBits(port, _CLK); // TM1637ClkHigh();
	delay_us(3);
    }
}

void TM1637DisplayDecimal( GPIO_TypeDef * port, int _CLK, int _DIO, unsigned int v, unsigned char displaySeparator)
{
    unsigned char digitArr[4];
    unsigned char i;

    for (i = 0; i < 4; ++i)
    {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2 && displaySeparator)
        {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }

    TM1637Start(port, _CLK, _DIO);
    TM1637WriteByte(port, _CLK, _DIO, 0x40);
    TM1637ReadAck(port, _CLK, _DIO);
    TM1637Stop(port, _CLK, _DIO);

    TM1637Start(port, _CLK, _DIO);
    TM1637WriteByte(port, _CLK, _DIO, 0xc0);
    TM1637ReadAck(port, _CLK, _DIO);

    for (i = 0; i < 4; ++i)
    {
      if (i == 0 & digitArr[3] == 0x3f)
      {
        digitArr[3] = 0x00;
      }
      if (i == 1 & digitArr[3] == 0x00 & digitArr[2] == 0x3f)
      {
        digitArr[2] = 0x00;
      }
      
        TM1637WriteByte(port, _CLK, _DIO,digitArr[3 - i]);
        TM1637ReadAck(port, _CLK, _DIO);
    }
    TM1637Stop(port, _CLK, _DIO);
}







