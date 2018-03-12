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

// Значения по умолчанию для уставок
#define PWR_TEN    14                                           // Мощность на тенах в процентах
#define END_RAZGON 5060                                         // Температура конца разгона * 100
#define STOP_OTBOR 8901                                         // Температура конец отбора (STOP) * 100
#define T_OTBOR    7620                                         // Температура отбора спирта * 100      
#define TYPE       1                                            // Режим: 1-дестилят, 2- ретификат 
#define BRESENHAM_INIT 100                                      // Количество отсчетов для алгоритма расчета мощьности

/* Описание регистра событий */
#define _SAVE_P                 ((uint16_t)0x0001)              /*!<  Начать запись параметров */
#define _STOP_                  ((uint16_t)0x0002)              /*!< Процесс остановлен */
#define _1S                     ((uint16_t)0x0004)              /*!< Бит 1 Секунда */
//#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
//#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */


#define SET_SAVE        PARAM |= _SAVE_P                        // Установка флага ЗАПИСАТЬ УСТАВКИ
#define F_SAVE          PARAM &  _SAVE_P                        // Проверка флага ЗАПИСАТЬ УСТАВКИ
#define RSET_SAVE       PARAM  &= ~  _SAVE_P                    // Сброс флага ЗАПИСАТЬ УСТАВКИ
#define SET_1S          PARAM |= _SAVE_P                        // Установка флага ЗАПИСАТЬ УСТАВКИ
#define F_1S            PARAM &  _SAVE_P                        // Проверка флага ЗАПИСАТЬ УСТАВКИ
#define RSET_SAVE       PARAM  &= ~  _SAVE_P                    // Сброс флага ЗАПИСАТЬ УСТАВКИ

typedef struct  {
   float t_def;                                                 // Температура дефлегматора
   float t_water;                                               // Температура воды на выходе
//   float ochki; 
}Datchiki_All;

typedef struct  Ustavki{
   uint16_t pwr;                                                // Мощность на тенах в процентах
   uint16_t end_razgon;                                         // Температура конца разгона
   uint16_t stop_otbr;                                          // Температура конец отбора (STOP)
   uint16_t t_otbr;                                             // Температура отбора спирта
   uint16_t type;                                               // Режим: 1-дестилят, 2- ретификат
 }Ustavki;

typedef struct  {
   char str0[20];                                               // 0 строка
   char str1[20];                                               // 1 строка
   char str2[20];                                               // 2 строка
   char str3[20];                                               // 3 строка
}LcdWindows;

void setup();
void loop();
void DispOut();
uint32_t InitRCC( void);
void led_flash(void);
void encoderHandler();
//EXTI0_IRQHandler(void);
//EXTI1_IRQHandler(void);
//EXTI2_IRQHandler(void);
void encoderHandler();


#define DEF_CLK   GPIO_Pin_12                             // PORTB  Индикатор температура
#define DEF_DIO   GPIO_Pin_13                             //    в узле отбора

#define WAT_CLK   GPIO_Pin_14                             // PORTB  Индикатор температура
#define WAT_DIO   GPIO_Pin_15                             //    в воды на выходе охлодителя

#define PWR_CLK   GPIO_Pin_8  //  PORTA Индикатор температура
#define PWR_DIO   GPIO_Pin_9  //    в воды на выходе охлодителя

#define OTB_CLK   GPIO_Pin_10  // PORTA  Индикатор температура
#define OTB_DIO   GPIO_Pin_11  //    в воды на выходе охлодителя


#endif

/*
#define GPIOA_O13  *((uint8_t *) (BITBAND_PERI((uint32_t)(&GPIOA->ODR),13)))
#define GPIOA_I13  *((uint8_t *) (BITBAND_PERI((uint32_t)(&GPIOA->IDR),13)))

GPIOA_O13 = 0;
GPIOA_O13 = 1;
*/


