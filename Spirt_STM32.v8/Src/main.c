#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "misc.h"                                          
#include "Delay.h"
#include "main.h" 
#include "I2C.h"
#include "LCD_I2C.h"
#include "TM1637.h"
#include <stdio.h>
#include <stdlib.h>
#include "ds1820.h"
#include "bresenham.h"
#define  SIZE_U sizeof(Ustavki)                                 //      Размер буфера структуры Уставок
#define  DES_A 0                                                //      Адресс уставок десциляции
#define  RET_A DES_A+SIZE_U+4                                   //      Адресс уставок ретефикации
#define  ENCODER_TRESHOLD    2                                  // Для энкодера/ количество прерываний на один щелчек

#define DEF_PIN GPIO_Pin_6                                      // Датчик температуры отборв
#define WAT_PIN GPIO_Pin_7                                      // Датчик температура воды

uint8_t tablePtr = 0x3;
static int8_t table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int currentEncoderValue = 0;
bresenham_struct bs;                                            // Обявим струстуру Алгоритм Брезенхема для регулирования мощности
uint8_t PARAM = 0;                                              // Регистор состояния программы

uint8_t rr =0 ,ii=0, temp=0; 
uint16_t  temp16=0; 
volatile uint16_t  count_1s=0, flag_1s=0, tiktak=0; 

uint8_t Brightnes=7;                                            // Яркость мерцания 

TIM_TimeBaseInitTypeDef timer2;
EXTI_InitTypeDef  EXTI_InitStructure;   
NVIC_InitTypeDef NVIC_InitStructure;

volatile  signed int TempMain;
volatile  int f_1s =0 ;
volatile  int f_2s =0 ;

volatile Datchiki_All Dat;                                      // Показания датчиков

Ustavki Des;                                                    // Уставки для Десциляции
Ustavki Ret;                                                    // Уставки для Ретивикации

LcdWindows LW;                                                  // Отображаеая информация на LCD

/* Настройка прерываний */
void EXTI_INIT(void)    
{
/*     А вот и долгожданная настройка таймера TIM2 на 1 секунду */
TIM_TimeBaseStructInit(&timer2);
timer2.TIM_Prescaler =  (SystemCoreClock / 7200);// - 1;           // 10000
timer2.TIM_Period = 2500; // 1000;                               // 9999
timer2.TIM_ClockDivision = 0;
timer2.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &timer2);
TIM_Cmd(TIM2, ENABLE);
TIM_ClearFlag(TIM2, TIM_FLAG_Update);
TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
/*     Настроим прерывание для обноружения нуля 220в  */
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);     // говорим что вывод PA0 используется как внешний вывод прерывания
EXTI_InitStructure.EXTI_Line = EXTI_Line3;                      // используем линию 0 (она для портов PA0 - PG0)
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             // режим хардварного прерывания
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;          // прерывание по нарастанию импульса
EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       // ??? это я вообще непонял зачем нужно ???
EXTI_Init(&EXTI_InitStructure);                                 // передаем настройку в функцию инициализации
/*     Настроим прерывание для энкоддера  */
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0|GPIO_PinSource1|GPIO_PinSource2);     // говорим что вывод PA0, PA1, PA2 используется как внешний вывод прерывания
EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2;// используем линию 0,1 & 2
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             // режим хардварного прерывания
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // прерывание по нарастанию  и спаду импульса
EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       // ??? это я вообще непонял зачем нужно ???
EXTI_Init(&EXTI_InitStructure);                                 // передаем настройку в функцию инициализации
/* Установим приопрететы прерываний */
NVIC_SetPriority (EXTI0_IRQn, 1);                               // Понизем прерывание энкодера
NVIC_SetPriority (EXTI1_IRQn, 1);                               // Понизем прерывание энкодера
NVIC_SetPriority (EXTI2_IRQn, 1);                               // Понизем прерывание кнопки энкодера
NVIC_SetPriority (EXTI3_IRQn, 2);                               // Понизем прерывание детектора нуля
NVIC_SetPriority (TIM2_IRQn,  4);                               // Понизем прерывание секундного таймера
/* Разшешим прерывания прерываний */
NVIC_EnableIRQ(EXTI0_IRQn);                                     // разрешаем прерывание энкодера
NVIC_EnableIRQ(EXTI1_IRQn);                                     // разрешаем прерывание энкодера
NVIC_EnableIRQ(EXTI2_IRQn);                                     // разрешаем прерывание кнопки энкодера
NVIC_EnableIRQ(EXTI3_IRQn);                                     // разрешаем прерывание детектора нуля
NVIC_EnableIRQ(TIM2_IRQn);                                      // Запустим таймер 
}


void EXTI0_IRQHandler(void)
{   
   if(EXTI_GetITStatus(EXTI_Line0) != RESET)                    // ловим прерывание
        {
              encoderHandler();
        }       
   EXTI_ClearFlag(EXTI_Line0);                                  // сбрасываем флаг прерывания
}

void EXTI1_IRQHandler(void)
{
     if(EXTI_GetITStatus(EXTI_Line1) != RESET)                  // ловим прерывание
        {
            encoderHandler();
        }        
      EXTI_ClearFlag(EXTI_Line1);                               // сбрасываем флаг прерывания
}

void EXTI2_IRQHandler(void)
{   
   if(EXTI_GetITStatus(EXTI_Line2) != RESET)                    // ловим прерывание
        {
              SET_SAVE;
        }       
   EXTI_ClearFlag(EXTI_Line2);                                  // сбрасываем флаг прерывания
}

void EXTI3_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line3) != RESET)                    // ловим прерывание
        {
               led_flash();
        }
        EXTI_ClearFlag(EXTI_Line3);                             // сбрасываем флаг прерывания
}

void encoderHandler()
{
    tablePtr <<= 2;
    tablePtr |= GPIOA->IDR & 0x3;
    currentEncoderValue += table[(tablePtr & 0x0F)];
    if (currentEncoderValue > ENCODER_TRESHOLD) {
        currentEncoderValue = 0;
        // send increment event 
        // printf("%s\n","-1");
        Des.pwr--;
       // PARAM |= _SAVE_P;
    } else if (currentEncoderValue < -ENCODER_TRESHOLD) {
        currentEncoderValue = 0;
        // send decrement event 
        //printf("%s\n","+1");
        Des.pwr++;
        // PARAM |= _SAVE_P;
       // SET_SAVE;
    }    
}

uint32_t InitRCC( void) {
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
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
  if ( HSEStatus == (uint32_t)0x01) {   
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }
     
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
  
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08) {
    }
  }
  else {
  } 
  return HSEStatus;
}


void initAll() // Настройка переферии
{
    init_delay_DWT();                                           // Подготовили delay   
    // Включаем тактирование переферии  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);
    
    GPIO_InitTypeDef port;
    //Про эту функцию напишу чуть ниже
    GPIO_StructInit(&port);
    
    // настройка пинов для 7 сегментых дисплеев PORTA
    port.GPIO_Pin = (GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
    port.GPIO_Speed = GPIO_Speed_50MHz;
    port.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &port);
    
    // Настройка светодиодика
    port.GPIO_Mode = GPIO_Mode_Out_PP;
    port.GPIO_Pin = GPIO_Pin_13;
    port.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &port);    
    
    // Настройка порта для DS18D20 - термодатчик Основной
    port.GPIO_Mode = GPIO_Mode_Out_OD;   
    port.GPIO_Speed = GPIO_Speed_2MHz; 
   
    // термодатчик Основной порт А6 | Обратной Воды порт А7
    port.GPIO_Pin = (DEF_PIN|WAT_PIN);
    GPIO_Init(GPIOA, &port);    
    
    // настройка пинов для 7 сегментых дисплеев PORTB
    port.GPIO_Pin = (GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    port.GPIO_Speed = GPIO_Speed_50MHz;
    port.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &port);
    
    // настройка пинов для eeprom     
    port.GPIO_Pin = (GPIO_Pin_10 | GPIO_Pin_11);
    port.GPIO_Speed = GPIO_Speed_50MHz;
    port.GPIO_Mode = GPIO_Mode_AF_OD;    
    GPIO_Init(GPIOB, &port);
    
    
    //port.GPIO_Mode = GPIO_Mode_Out_PP;
    //port.GPIO_Pin = GPIO_Pin_5;
    //GPIO_Init(GPIOA, &port);
    
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);
    EXTI_INIT();                                                // Настроим и запустим прерывание 
    I2C_init_EE();                                              // Настроим шину чтения EEPROM         
}


void TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  
  tiktak=1;
  if(f_1s==3) f_1s=1;
  if(f_1s==4) f_1s=0;
  if(count_1s++ >= 10)
  {
    count_1s=0;
    flag_1s =1;    
  }  
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

void EE_save(uint16_t _addr, uint8_t * src )
{                                                               // Дергаем раз в пару секунд
  for (uint32_t i=0; i<SIZE_U; i++)
    {
      I2C_EE_ByteWrite( _addr++ ,*src++); 
      delay_ms(12);
    }
  delay_ms(12);
}

void setup()
{                                                               // НАстройки LCD 
 // LCDI2C_init(0x27,20,4);
 // LCDI2C_backlight();
 // LCDI2C_clear();
 
  ds18d20_init(DEF_PIN);
  ds18d20_init(WAT_PIN);
 // LCDI2C_noBacklight(); 
 // LCDI2C_backlight();
 
  TM1637SetBrightness(GPIOB, DEF_CLK, DEF_DIO, Brightnes);
  TM1637SetBrightness(GPIOA, PWR_CLK, PWR_DIO, Brightnes);
 
  EE_load(DES_A ,(uint8_t *)&Des);                              // прочитаем уставки и если их нет зальем по дефолту
  delay_ms(50);
  EE_load(RET_A ,(uint8_t *)&Ret);
 
  if( Des.type!=1|Ret.type!=2) 
    {
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
  bresenham_init(&bs, BRESENHAM_INIT);                          // Инициализазия алгоритма мощьности
  bresenham_setValue(&bs,Des.pwr);                              // Установка значения
}

void led_flash()
{
  if(bresenham_getNext(&bs))
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
  else
    {
      GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }  
}

void _TikTak()
{  
  if (Brightnes==2) 
    {
      Brightnes=7;
    }
    else
    {
      Brightnes=2;
    }
  TM1637SetBrightness(GPIOB, DEF_CLK, DEF_DIO, Brightnes);
  tiktak=0;
}

void loop()
{  
  // Основная программа
  if (tiktak>=1) _TikTak();
  if(F_SAVE)
  {                                                             // Проверим стоит флаг записи или нет
    EE_save(DES_A ,(uint8_t *)&Des);                            // Проверим и если нужно запишем параметры
    bresenham_setValue(&bs,Des.pwr);                            // Установка значения
    RSET_SAVE;                                                  // Сбросим флаг после записи
    DispOut();  
  }   
     
  switch (f_1s)
  {
  case 0:
    if (send_presence(DEF_PIN)==1)
    {
     ds18d20_start(DEF_PIN);
     ds18d20_start(WAT_PIN);     
    // led_flash(); // моргнем светодиодиком
    }
    f_1s=3;
    break;
  case 1:    
    Dat.t_def = (float)ds18d20_read(DEF_PIN)/16;
    Dat.t_water = (float)ds18d20_read(WAT_PIN)/16;
    led_flash(); // моргнем светодиодиком
    DispOut();  
    
      
    f_1s=4;
    break;
  }
  
  if (f_1s==1)
  {
    f_2s == 2 ? f_2s=0 : f_2s++  ; 
  }
  else
  { 

  }
}

void DispOut(){
  //sprintf(LW.str0,"%s","MK STM32F103C8");
   //printf(LW.str0,"%s","MK STM32F103C8");
  //sprintf(LW.str1,"%i   ",Des.pwr);
  //sprintf(LW.str2,"Tempr: %.2f %.2f",Dat.t_def, (float)Des.end_razgon/100);
  //sprintf(LW.str3,"Water: %.2f    ",Dat.t_water);
  //sprintf(LW.str3,"%s","####################");
 
// LCDI2C_setCursor(0,0);
// LCDI2C_write_String(LW.str0);
 //LCDI2C_setCursor(0,1); 
 //LCDI2C_write_String(LW.str1);
 //LCDI2C_setCursor(0,2); 
// LCDI2C_write_String(LW.str2);
 //LCDI2C_setCursor(0,3); 
 //LCDI2C_write_String(LW.str3);
 
 
 TM1637DisplayDecimal (GPIOB, DEF_CLK, DEF_DIO, (int)(Dat.t_water*100),1);
 TM1637DisplayDecimal (GPIOA, PWR_CLK, PWR_DIO, (int)(Des.pwr),0);
 
}
