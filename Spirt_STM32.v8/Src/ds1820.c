#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Delay.h"
#include "ds1820.h"
#include "main.h"

///----------------////--------------------////

 
uint8_t send_presence(uint8_t PinNumb) { 	
        GPIO_SetBits(GPIOA,PinNumb);                                            // ���� � 1 ( �������� ������)
        delay_us(20);                                                           // ���� �� ����� 15, �� �� ���� 60 �����������
        GPIO_ResetBits(GPIOA,PinNumb);
	delay_us(420); //420us                                                  // �� ���������� ������� reset, ��������� ���� � 0 ��������� � ������� 480 �����������.
        GPIO_SetBits(GPIOA,PinNumb);                                            // ���� � 1 ( �������� ������)
        delay_us(20);                                                           // ���� �� ����� 15, �� �� ���� 60 �����������
        if(GPIO_ReadInputDataBit (GPIOA, PinNumb)){                                // ���� ������ �������� ��� 1 (�� ����)
          return 1;
        }else{
         return 0;
        }
         
       
}

void one_wire_write_bit(uint8_t bit,uint8_t PinNumb){
        GPIO_ResetBits(GPIOA,PinNumb);                                          // ���� � 0
	delay_us(bit ? 15 : 65);                                                // ���� 1 ���� 15 ������, 0 ���� 65 ������
        GPIO_SetBits(GPIOA,PinNumb);                                            // ���� � 1
	delay_us(bit ? 15 : 80);                                                // ���� 1 ���� 15 ������, 0 ���� 80 ������
}

uint8_t one_wire_read_bit(uint8_t PinNumb)
{
	uint8_t bit = 0;
        GPIO_ResetBits(GPIOA,PinNumb);                                          // ���� � 0
        delay_us(2);
        GPIO_SetBits(GPIOA,PinNumb);                                            // ���� � 1
        delay_us(15);
        switch (PinNumb){
          case GPIO_Pin_3:
                GPIOA->CRL &= ~GPIO_CRL_MODE3;
                GPIOA->CRL &= ~GPIO_CRL_CNF3;
                GPIOA->CRL |=  GPIO_CRL_CNF3_0;
                bit = (GPIOA->IDR&PinNumb?1:0);
                GPIOA->CRL |=  GPIO_CRL_MODE3;
                GPIOA->CRL |=  GPIO_CRL_CNF3_0;  
                break;
          case GPIO_Pin_4:
                GPIOA->CRL &= ~GPIO_CRL_MODE4;
                GPIOA->CRL &= ~GPIO_CRL_CNF4;
                GPIOA->CRL |=  GPIO_CRL_CNF4_0;
                bit = (GPIOA->IDR&PinNumb?1:0);
                GPIOA->CRL |=  GPIO_CRL_MODE4;
                GPIOA->CRL |=  GPIO_CRL_CNF4_0;
                break;
        }
        delay_us(45);
	return bit;
}

void one_wire_write_byte(uint8_t data,uint8_t PinNumb)
{
	for(uint8_t i = 0; i<8; i++) 		one_wire_write_bit(data>>i & 1,PinNumb);
}

void ds18d20_init(uint8_t PinNumb){
   if (send_presence(PinNumb)){
	delay_ms(25);
       	one_wire_write_byte(0xCC,PinNumb); // 0xCC (���������� � ������������� ���������� �� �����)
	one_wire_write_byte(0x4E,PinNumb); // 0x4E (������ � ��������)
	one_wire_write_byte(0x00,PinNumb); // 0x4B (������� ����� �������)
	one_wire_write_byte(0x00,PinNumb); // 0x46 (������ ����� �������)
	//one_wire_write_byte(0x5F,PinNumb); // 0x5F (����������� 11 ���)
        one_wire_write_byte(0x7F,PinNumb); // 0x7F (����������� 12 ���)
        delay_ms(25);
        one_wire_write_byte(0xCC,PinNumb); // 0xCC (���������� � ������������� ���������� �� �����)
        one_wire_write_byte(0x48,PinNumb); // 0x48 ������� � ������ 
  } 
}


void ds18d20_start(uint8_t PinNumb)
{
  if(send_presence(PinNumb)){
	delay_ms(55);
        one_wire_write_byte(0xCC,PinNumb);
	one_wire_write_byte(0x44,PinNumb);
  }
 
}


int ds18d20_read(uint8_t PinNumb){
        int rawTemperature;
        uint16_t data0 = 0;
        uint16_t data1 = 0;
        int  sig =1;
	if ( send_presence(PinNumb)){
        delay_ms(55);
	one_wire_write_byte(0xCC,PinNumb);
	one_wire_write_byte(0xBE,PinNumb);
        delay_ms(55);	
        
	for(uint8_t i = 0; i<8; i++)  {
          data0 += (uint16_t)one_wire_read_bit(PinNumb)<<i;
          delay_us(5);
        }
        for(uint8_t i = 0; i<8; i++)  {
          data1 += (uint16_t)one_wire_read_bit(PinNumb)<<i;
          delay_us(5);
        }
      if (data1>248){
          data0=0xFF-data0;
          data1=0xFF-data1;
          sig=-1;
        }else{ 
          sig=1;
        }
          rawTemperature  = ((uint16_t)((data1 * 256) + (uint16_t)data0)*sig);
        }else{
          rawTemperature= 0;                                                     // ���� ��� ������� ����� , ��������
        }
        return rawTemperature;
        
}


//***************************************************************************
// CRC8
// ��� ��������� ������ �������� 8 ���
// ��� ������ ������� 9 ���
// ���� � ���������� crc == 0, �� ������ �������
//***************************************************************************
uint8_t crc8 (uint8_t data, uint8_t crc)
#define CRC8INIT   0x00
#define CRC8POLY   0x18              //0X18 = X^8+X^5+X^4+X^0
{
	uint8_t bit_counter;
	uint8_t feedback_bit;
	bit_counter = 8;
	do
	{
		feedback_bit = (crc ^ data) & 0x01;
		if ( feedback_bit == 0x01 ) crc = crc ^ CRC8POLY;
		crc = (crc >> 1) & 0x7F;
		if ( feedback_bit == 0x01 ) crc = crc | 0x80;
		data = data >> 1;
		bit_counter--;
	}  while (bit_counter > 0);
	return crc;
}

