#include "delays.h"

void delay_ms(unsigned long nTime)
{ 
	nTime=(CPU_CLOCK/K_Const)*nTime;
  	while(nTime != 0)
  	{nTime--;}
}

void delay_us(unsigned long nTime)
{ 
	nTime=((CPU_CLOCK/1000)/K_Const)*nTime;
  	while(nTime != 0)
  	{nTime--;}
}
