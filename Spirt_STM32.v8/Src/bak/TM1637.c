#include "TM1637.h"
#include "stm32f10x.h"
#include "misc.h"
#include "Delay.h"
#include "stm32f10x_gpio.h"

#define uchar unsigned char
#define ushort unsigned short
#define ulong unsigned long
#define uint unsigned int

void TM1637Start(void);
void TM1637Stop(void);
void TM1637ReadAck(void);
void TM1637WriteByte(unsigned char b);

void TM1637ClkLow(void);
void TM1637DatHigh(void);
void TM1637DatLow(void);
void TM1637SetBrightness(unsigned char brightness);

#define TM1637_CLK   GPIO_Pin_12  //PORTA
#define TM1637_DIO   GPIO_Pin_13  //PORTA

#define DEF_CLK   GPIO_Pin_12  // PORTB  Индикатор температура
#define DEF_DIO   GPIO_Pin_13  //    в узле отбора

#define WAT_CLK   GPIO_Pin_14  // PORTB  Индикатор температура
#define WAT_DIO   GPIO_Pin_15  //    в воды на выходе охлодителя

#define PWR_CLK   GPIO_Pin_8  //  PORTA Индикатор температура
#define PWR_DIO   GPIO_Pin_9  //    в воды на выходе охлодителя

#define OTB_CLK   GPIO_Pin_10  // PORTA  Индикатор температура
#define OTB_DIO   GPIO_Pin_11  //    в воды на выходе охлодителя

#define TM1637ClkHigh(); GPIO_SetBits(GPIOB, TM1637_CLK);
#define TM1637ClkLow();  GPIO_ResetBits(GPIOB, TM1637_CLK);
#define TM1637DatHigh(); GPIO_SetBits(GPIOB, TM1637_DIO);
#define TM1637DatLow();  GPIO_ResetBits(GPIOB, TM1637_DIO);

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


void TM1637SetBrightness(unsigned char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness

    TM1637Start();
    TM1637WriteByte(0x87 + brightness);
    TM1637ReadAck();
    TM1637Stop();
}

void TM1637Start(void)
{
    TM1637ClkHigh();
    TM1637DatHigh();
    delay_us(2);
    TM1637DatLow();
}

void TM1637Stop(void)
{
    TM1637ClkLow();
    delay_us(2);
    TM1637DatLow();
    delay_us(2);
    TM1637ClkHigh();
    delay_us(2);
    TM1637DatHigh();
}

void TM1637ReadAck(void)
{
    TM1637ClkLow();
    delay_us(5);
    // while (TM1637Dat); // Ignore the feedback
    TM1637ClkHigh();
    delay_us(2);
    TM1637ClkLow();
}

void TM1637WriteByte(unsigned char b)
{
    unsigned char i;
    for ( i = 0; i < 8; ++i)
    {
        TM1637ClkLow();
        if (b & 0x01)
        {
            TM1637DatHigh();
        }
        else
        {
            TM1637DatLow();
        }
	delay_us(3);
        b >>= 1;
        TM1637ClkHigh();
	delay_us(3);
    }
}

void TM1637DisplayDecimal(unsigned int v, unsigned char displaySeparator)
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

    TM1637Start();
    TM1637WriteByte(0x40);
    TM1637ReadAck();
    TM1637Stop();

    TM1637Start();
    TM1637WriteByte(0xc0);
    TM1637ReadAck();

    for (i = 0; i < 4; ++i)
    {
        TM1637WriteByte(digitArr[3 - i]);
        TM1637ReadAck();
    }
    TM1637Stop();
}

// ******************* MAIN *****************
/*
int main(void)
{
    u8 count=0;
    u8 state;


    RCC_Configuration(); //HSExternal
    //HSI_clock_init(); //HSInternal
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    MCU_init();
    TM1637DisplayDecimal (8888,0);
    delay_ms(1000);
    //set the brightness
    TM1637SetBrightness(7);

    while (1)
    {
        count++;
        if (count>9999) count=0;
        //blink the LED (not part of seven segment)
        if (count &1) GPIO_SetBits(GPIOC, LED);
        else GPIO_ResetBits(GPIOC, LED);

        //display 4 digit decimal on seven segment
        //TM1637DisplayDecimal (count,0); //count, separator ON/OFF

        state = DHT11GetData();
        switch(state)
        {
            case 1:
            {
                //TM1637DisplayDecimal (1111,0);
                //break;
            }
            case 2:
            {
                //lcd_out(1, 1, "No Sensor Found!");
                TM1637DisplayDecimal (2222,0);
                break;
            }
            case 3:
            {
                //lcd_out(1, 1, "Checksum Error!");
                TM1637DisplayDecimal (3333,0);
                break;
            }
            default:
            {
                TM1637DisplayDecimal (DHT11_data[0]*100+DHT11_data[2],0);
                break;
            }
        }
        delay_ms(2000);
    }
}
*/






