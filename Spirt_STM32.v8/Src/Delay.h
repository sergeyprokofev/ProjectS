#ifndef DELAY_H_
#define DELAY_H_


#define    DWT_CYCCNT    *(volatile uint32_t *)0xE0001004
#define    DWT_CONTROL   *(volatile uint32_t *)0xE0001000
#define    SCB_DEMCR     *(volatile uint32_t *)0xE000EDFC
#define    NUM_CYCLES_MS 1000

#define _delay_ms(ms)            DWT_Delay(((ms)*(NUM_CYCLES_MS)))

#include "stm32f10x.h"
#define delay_us(us)            _delay_us(us)
#define delay_ms(ms)            _delay_ms(ms)
#define Delay(ms)               _delay_us(ms)
#define DelayMC(us)             _delay_us(us) 

extern void init_delay_DWT();
extern void _delay_us (uint16_t delay);

uint32_t DWT_Get(void);
uint8_t  DWT_Compare(int32_t tp);
void 	 DWT_Delay(uint32_t us);



#endif /*DELAY_H_*/
