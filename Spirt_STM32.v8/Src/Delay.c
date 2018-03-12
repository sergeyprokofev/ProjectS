#include "Delay.h"

/*Запуск 32 разрядного счетчика для функции delay */
void init_delay_DWT(){  
    
if (!(DWT_CONTROL & 1))                                                         
    {
        SCB_DEMCR  |= 0x01000000;
        DWT_CYCCNT  = 0; 
        DWT_CONTROL|= 1; // enable the counter
    }


}

/* Микросикундный */
void _delay_us (uint16_t delay)
{
	DWT_Delay( delay);
}


volatile void _delay_ (uint32_t delay)
{
    while(delay--);
}



uint32_t DWT_Get(void)
{
    return DWT_CYCCNT;
}
//__inline

uint8_t DWT_Compare(int32_t tp)
{
    return (((int32_t)DWT_Get() - tp) < 0);
}


void DWT_Delay(uint32_t us) // microseconds
{
    int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
    while (DWT_Compare(tp));
}








