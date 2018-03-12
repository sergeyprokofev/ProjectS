#ifndef	_MAIN
#define	_MAIN

#define STM32F1xx /*!< Use STM32F1xx libraries */

#if defined(STM32F1xx) || defined(STM32F1XX)
#ifndef STM32F1xx
#define STM32F1xx
#endif
#ifndef STM32F1XX
#define STM32F1XX
#endif
#include "stm32f10x.h"

#endif

// �������� �� ��������� ��� �������
#define PWR_TEN    14           // �������� �� ����� � ���������
#define END_RAZGON 5060         // ����������� ����� ������� * 100
#define STOP_OTBOR 8901         // ����������� ����� ������ (STOP) * 100
#define T_OTBOR    7620         // ����������� ������ ������ * 100      
#define TYPE       1            // �����: 1-��������, 2- ��������� 

/* �������� �������� ������� */
#define _SAVE_P                 ((uint16_t)0x0001)              /*!<  ������ ������ ���������� */
#define _STOP_                  ((uint16_t)0x0002)              /*!< ������� ���������� */
#define _1S                     ((uint16_t)0x0004)              /*!< ��� 1 ������� */
//#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
//#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */


#define SET_SAVE        PARAM |= _SAVE_P                               // ��������� ����� �������� �������
#define F_SAVE          PARAM &  _SAVE_P                               // �������� ����� �������� �������
#define RSET_SAVE       PARAM  &= ~  _SAVE_P                          // ����� ����� �������� �������
#define SET_1S          PARAM |= _SAVE_P                               // ��������� ����� �������� �������
#define F_1S            PARAM &  _SAVE_P                               // �������� ����� �������� �������
#define RSET_SAVE       PARAM  &= ~  _SAVE_P                          // ����� ����� �������� �������



typedef struct  {
   float t_def;                                                                 // ����������� ������������
   float t_water;                                                               // ����������� ���� �� ������
//   float ochki; 
}Datchiki_All;

typedef struct  Ustavki{
   uint16_t pwr;                                                                 // �������� �� ����� � ���������
   uint16_t end_razgon;                                                          // ����������� ����� �������
   uint16_t stop_otbr;                                                           // ����������� ����� ������ (STOP)
   uint16_t t_otbr;                                                              // ����������� ������ ������
   uint16_t type;                                                                // �����: 1-��������, 2- ���������
 }Ustavki;



typedef struct  {
   char str0[20];                 // 0 ������
   char str1[20];                 // 1 ������
   char str2[20];                 // 2 ������
   char str3[20];                 // 3 ������
}LcdWindows;



#endif

void setup();
void loop();
void lcd_out();
uint32_t InitRCC( void);
void led_flash(void);
void encoderHandler();
//EXTI0_IRQHandler(void);
//EXTI1_IRQHandler(void);
//EXTI2_IRQHandler(void);
void encoderHandler();


/*
#define GPIOA_O13  *((uint8_t *) (BITBAND_PERI((uint32_t)(&GPIOA->ODR),13)))
#define GPIOA_I13  *((uint8_t *) (BITBAND_PERI((uint32_t)(&GPIOA->IDR),13)))

GPIOA_O13 = 0;
GPIOA_O13 = 1;
*/


