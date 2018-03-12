#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "misc.h"                                           // работа с прерываниями
#include "Delay.h"
#include "main.h" 
#include "I2C.h"
#include "LCD_I2C.h"
#include <stdio.h>
#include <stdlib.h>
#include "ds1820.h"

#define  SIZE_U sizeof(Ustavki)                             // Размер буфера структуры Уставок
#define  DES_A 0                                            //Адресс уставок десциляции
#define  RET_A DES_A+SIZE_U+4                               //Адресс уставок ретефикации
#define  ENCODER_TRESHOLD    2                              // Для энкодера/ количество прерываний на один щелчек

uint8_t tablePtr = 0x3;
static int8_t table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int currentEncoderValue = 0;

uint8_t PARAM = 0; //Регистор состояния программы

uint8_t rr =0 ,ii=0, temp=0; 
uint16_t        temp16=0; 

TIM_TimeBaseInitTypeDef timer2;
EXTI_InitTypeDef  EXTI_InitStructure;   
NVIC_InitTypeDef NVIC_InitStructure;

volatile  signed int TempMain;
volatile  int f_1s =0 ;
volatile  int f_2s =0 ;

volatile Datchiki_All Dat;                                      // Показания датчиков

Ustavki Des;                                                    // Уставки для Десциляции
Ustavki Ret;                                                    // Уставки для Ретивикации

LcdWindows LW;                                                    // Отображаеая информация на LCD

/* Настройка прерываний */
void EXTI_INIT(void)    
{
/*     А вот и долгожданная настройка таймера TIM2 на 1 секунду */
TIM_TimeBaseStructInit(&timer2);
timer2.TIM_Prescaler =  (SystemCoreClock / 10000) - 1;
timer2.TIM_Period = 9999; // 1000;
timer2.TIM_ClockDivision = 0;
timer2.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &timer2);
TIM_Cmd(TIM2, ENABLE);
TIM_ClearFlag(TIM2, TIM_FLAG_Update);
TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
/*     Настроим прерывание для обноружения нуля 220в  */
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);     // говорим что вывод PA0 используется как внешний вывод прерывания
EXTI_InitStructure.EXTI_Line = EXTI_Line2;;                     // используем линию 0 (она для портов PA0 - PG0)
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             // режим хардварного прерывания
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;          // прерывание по нарастанию импульса
EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       // ??? это я вообще непонял зачем нужно ???
EXTI_Init(&EXTI_InitStructure);                                 // передаем настройку в функцию инициализации
/*     Настроим прерывание для энкоддера  */
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0|GPIO_PinSource1);     // говорим что вывод PA1 и PA2 используется как внешний вывод прерывания
EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;           // используем линию 1 и 2 
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             // режим хардварного прерывания
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // прерывание по нарастанию  и спаду импульса
EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       // ??? это я вообще непонял зачем нужно ???
EXTI_Init(&EXTI_InitStructure);                                 // передаем настройку в функцию инициализации
/* Установим приопрететы прерываний */
NVIC_SetPriority (EXTI0_IRQn, 1);                               // Понизем прерывание энкодера
NVIC_SetPriority (EXTI1_IRQn, 1);                               // Понизем прерывание энкодера
NVIC_SetPriority (EXTI2_IRQn, 2);                               // Понизем прерывание детектора нуля
NVIC_SetPriority (TIM2_IRQn,  4);                               // Понизем прерывание секундного таймера
/* Разшешим прерывания прерываний */
NVIC_EnableIRQ(EXTI0_IRQn);                                     // разрешаем прерывание энкодера
NVIC_EnableIRQ(EXTI1_IRQn);                                     // разрешаем прерывание энкодера
NVIC_EnableIRQ(EXTI2_IRQn);                                     // зрешаем прерывание детектора нуля
NVIC_EnableIRQ(TIM2_IRQn);                                      // Запустим таймер 

}


void EXTI2_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line2) != RESET) // ловим прерывание
        {
               led_flash();
        }
        EXTI_ClearFlag(EXTI_Line2); // сбрасываем флаг прерывания
}

void EXTI0_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line0) != RESET) // ловим прерывание
        {
              encoderHandler();
        }
        EXTI_ClearFlag(EXTI_Line0); // сбрасываем флаг прерывания
}

void EXTI1_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line1) != RESET) // ловим прерывание
        {
            encoderHandler();
        }
        EXTI_ClearFlag(EXTI_Line1); // сбрасываем флаг прерывания
}


void encoderHandler()
{
    tablePtr <<= 2;
    tablePtr |= GPIOA->IDR & 0x3;
    currentEncoderValue += table[(tablePtr & 0x0F)];
    if (currentEncoderValue > ENCODER_TRESHOLD) {
        currentEncoderValue = 0;
        // send increment event 
        printf("%s\n","-1");
        Des.pwr--;
        PARAM |= _SAVE_P;
    } else if (currentEncoderValue < -ENCODER_TRESHOLD) {
        currentEncoderValue = 0;
        // send decrement event 
        printf("%s\n","+1");
        Des.pwr++;
      //  PARAM |= _SAVE_P;
        SET_SAVE;
    }
    
}

uint32_t InitRCC( void) {
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
 
  /* Конфигурацяи  SYSCLK, HCLK, PCLK2 и PCLK1 ---------------------------*/
  /* Включаем HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Ждем пока HSE не выставит бит готовности либо не выйдет таймаут*/
  do {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while( (HSEStatus == 0) && (StartUpCounter != HSEStartUp_TimeOut));
 
  if ( (RCC->CR & RCC_CR_HSERDY) != RESET) {
    HSEStatus = (uint32_t)0x01;
  }
  else {
    HSEStatus = (uint32_t)0x00;
  }
 
  /* Если HSE запустился нормально */
  if ( HSEStatus == (uint32_t)0x01) {
    /* Включаем буфер предвыборки FLASH */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
 
    /* Конфигурируем Flash на 2 цикла ожидания */
    /* Это нужно потому, что Flash не может работать на высокой частоте */
//  FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
//    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
 
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
 
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
 
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
 
    /* Конфигурируем множитель PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    /* При условии, что кварц на 8МГц! */
    /* RCC_CFGR_PLLMULL9 - множитель на 9. Если нужна другая частота, не 72МГц */
    /* то выбираем другой множитель. */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
 
    /* Включаем PLL */
    RCC->CR |= RCC_CR_PLLON;
 
    /* Ожидаем, пока PLL выставит бит готовности */
    while((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }
 
    /* Выбираем PLL как источник системной частоты */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
 
    /* Ожидаем, пока PLL выберется как источник системной частоты */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08) {
    }
  }
  else {
   /* Все плохо... HSE не завелся... Чего-то с кварцем или еще что...
      Надо бы както обработать эту ошибку... Если мы здесь, то мы работаем
      от HSI! */
  }
 
  return HSEStatus;
}


void initAll() // Настройка переферии
{
    init_delay_DWT();                                       // Подготовили delay   
// Включаем тактирование переферии  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);
    
    GPIO_InitTypeDef port;
    //Про эту функцию напишу чуть ниже
    GPIO_StructInit(&port);
    // Настройка светодиодика
    port.GPIO_Mode = GPIO_Mode_Out_PP;
    port.GPIO_Pin = GPIO_Pin_13;
    port.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &port);    
    
    // Настройка порта для DS18D20 - термодатчик Основной
    port.GPIO_Mode = GPIO_Mode_Out_OD;   
    port.GPIO_Speed = GPIO_Speed_2MHz; 
   
    // термодатчик Основной порт А5 | Обратной Воды порт А6
    port.GPIO_Pin = (GPIO_Pin_3|GPIO_Pin_4);
    GPIO_Init(GPIOA, &port);
    
    port.GPIO_Mode = GPIO_Mode_Out_PP;
    port.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &port);
    
    EXTI_INIT();                                            // Настроим и запустим прерывание 
    I2C_init_EE();                                          // Настроим шину чтения EEPROM

}



void TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  
  if( f_1s==3) f_1s=1;
  if( f_1s==4) f_1s=0;
}


int main()
{
  InitRCC();
  initAll();
  setup();
  for (;;) {loop();}
}

void EE_load(uint16_t _addr, uint8_t * src ){
      for (uint8_t i=0; i<SIZE_U; i++){
    *src++=I2C_EE_ByteRead( _addr++);
    }
}

void EE_save(uint16_t _addr, uint8_t * src ){                   // Дергаем раз в пару секунд
      for (uint32_t i=0; i<SIZE_U; i++){
      I2C_EE_ByteWrite( _addr++ ,*src++); 
      delay_ms(12);
     }
      delay_ms(12);

}

void setup(){
// НАстройки LCD 
 LCDI2C_init(0x27,20,4);
 LCDI2C_backlight();
 LCDI2C_clear();
 
 ds18d20_init(GPIO_Pin_3);
 ds18d20_init(GPIO_Pin_4);
 LCDI2C_noBacklight();
 
 LCDI2C_backlight();
 // прочитаем уставки и если их нет зальем по дефолту
 EE_load(DES_A ,(uint8_t *)&Des);
 delay_ms(50);
 EE_load(RET_A ,(uint8_t *)&Ret);
 
 if( Des.type!=1|Ret.type!=2) {
   Des.pwr        = PWR_TEN ;
   Des.end_razgon = END_RAZGON;
   Des.stop_otbr  = STOP_OTBOR;
   Des.t_otbr     = T_OTBOR;
   Des.type       = TYPE;
   Ret=Des;
   Ret.type=2;
   EE_save(DES_A ,(uint8_t *)&Des);
   delay_ms(50);
   EE_save(RET_A ,(uint8_t *)&Ret);
 }
 
}

void led_flash(){
 if(GPIO_ReadInputDataBit (GPIOC, GPIO_Pin_13)){
   GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}else{
GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
}

void loop(){ // Основная программа
 if (F_SAVE){                                               // Проверим стоит флаг записи или нет
          EE_save(DES_A ,(uint8_t *)&Des);                      // Проверим и если нужно запишем параметры
          RSET_SAVE;                                            // Сбросим флаг после записи
          lcd_out();  
 }
   
     
  switch (f_1s){
  case 0:
    if (send_presence(GPIO_Pin_3)==1){
     ds18d20_start(GPIO_Pin_4);
     ds18d20_start(GPIO_Pin_3);
     
    // led_flash(); // моргнем светодиодиком
    
    
    }
    f_1s=3;
    break;
  case 1:
    
   Dat.t_def = (float)ds18d20_read(GPIO_Pin_3)/16;
   Dat.t_water = (float)ds18d20_read(GPIO_Pin_4)/16;
   
   //led_flash(); // моргнем светодиодиком
   lcd_out(); 
    f_1s=4;
    break;
  }
  
if (f_1s==1){
  f_2s == 2 ? f_2s=0 : f_2s++  ;
 
  }else{ 

  }

}



void lcd_out(){
  sprintf(LW.str0,"%s","MK STM32F103C8");
   //printf(LW.str0,"%s","MK STM32F103C8");
  sprintf(LW.str1,"%i   ",Des.pwr);
  sprintf(LW.str2,"Tempr: %.2f %.2f",Dat.t_def, (float)Des.end_razgon/100);
  sprintf(LW.str3,"Water: %.2f    ",Dat.t_water);
  //sprintf(LW.str3,"%s","####################");
 
 LCDI2C_setCursor(0,0);
 LCDI2C_write_String(LW.str0);
 LCDI2C_setCursor(0,1); 
 LCDI2C_write_String(LW.str1);
 LCDI2C_setCursor(0,2); 
 LCDI2C_write_String(LW.str2);
 LCDI2C_setCursor(0,3); 
 LCDI2C_write_String(LW.str3);
 
}
