#include "24xxx.h"
//#include "LED.h"
#define EEPROM_HW_ADDRESS      0x57 

void EE24Init(I2C_TypeDef* I2Cx)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStructure;

	if (I2Cx==I2C1)
		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);}
	else
		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	if (I2Cx==I2C1)
		{GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;}			  //I2C1 PINs
	else
		{GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11;}		  //I2C2 PINs

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

           I2C_DeInit(I2Cx);
           I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
           I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
           I2C_InitStructure.I2C_OwnAddress1 = 0xB0;
           I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
           I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
           I2C_InitStructure.I2C_ClockSpeed = 100000;  /* 100kHz */

	I2C_Init(I2Cx, &I2C_InitStructure); 
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
}															 

int EE24WriteByte(I2C_TypeDef* I2Cx, char val, int WriteAddr)
{
	I2C_Cmd(I2Cx, ENABLE);
	
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){return I2C_BUS_BUSY;}					 /* While the bus is busy */
    
	I2C_GenerateSTART(I2Cx, ENABLE);													 /* Send START condition */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_MODE_SELECT)){return I2C_ERROR;}		 /* Test on EV5 and clear it */
    
	I2C_Send7bitAddress(I2Cx, EEPROM_HW_ADDRESS, I2C_Direction_Transmitter);			 /* Send EEPROM address for write */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))		 		 /* Test on EV6 and clear it */
    						   {return I2C_ERROR;}
	I2C_SendData(I2Cx, (char)((WriteAddr & 0xFF00) >> 8));								 /* Send the EEPROM's internal address to write to : MSB of the address first */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){return I2C_ERROR;}	 /* Test on EV8 and clear it */
    
	I2C_SendData(I2Cx, (char)(WriteAddr & 0x00FF));										 /* Send the EEPROM's internal address to write to : LSB of the address */    
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){return I2C_ERROR;}	 /* Test on EV8 and clear it */
     
	I2C_SendData(I2Cx, val);
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){return I2C_ERROR;}	 /* Test on EV8 and clear it */
    
	I2C_GenerateSTOP(I2Cx, ENABLE);													     /* Send STOP condition */																		 //delay between write and read...not less 4ms
	
	I2C_Cmd(I2Cx, DISABLE);
	return 0;							   
}											   	 
												   		
uint8_t EE24ReadByte(I2C_TypeDef* I2Cx, int ReadAddr)
{
    char tmp=0;

	I2C_Cmd(I2Cx, ENABLE);

    if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){I2C_GenerateSTOP(I2Cx, ENABLE); return I2C_BUS_BUSY;}					 /* While the bus is busy */

    I2C_GenerateSTART(I2C1, ENABLE);													 /* Send START condition */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_MODE_SELECT)){return I2C_ERROR;}		 /* Test on EV5 and clear it */
    
	I2C_Send7bitAddress(I2Cx, EEPROM_HW_ADDRESS, I2C_Direction_Transmitter);			 /* Send EEPROM address for write */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))	 			 /* Test on EV6 and clear it */
    		   {return I2C_ERROR;}
	I2C_SendData(I2C1, (char)((ReadAddr & 0xFF00) >> 8));								 /* Send the EEPROM's internal address to read from: MSB of the address first */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){return I2C_ERROR;}	 /* Test on EV8 and clear it */
    
	I2C_SendData(I2C1, (char)( ReadAddr & 0x00FF));										 /* Send the EEPROM's internal address to read from: LSB of the address */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){return I2C_ERROR;}	 /* Test on EV8 and clear it */
    
	I2C_GenerateSTART(I2C1, ENABLE);													 /* Send STRAT condition a second time */
    	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_MODE_SELECT)){return I2C_ERROR;}		 /* Test on EV5 and clear it */
       
	I2C_Send7bitAddress(I2Cx, EEPROM_HW_ADDRESS, I2C_Direction_Receiver);				 /* Send EEPROM address for read */
       	if(I2C_Wait_Event(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED)){return I2C_ERROR;}	 	 /* Test on EV6 and clear it */
	   
	I2C_GenerateSTOP(I2Cx, ENABLE);														 /* Send STOP Condition */
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);

	tmp=I2C_ReceiveData(I2Cx);

    I2C_Cmd(I2Cx, DISABLE);	
													
    return tmp;
}

char I2C_Wait_Event(I2C_TypeDef* I2Cx, long event)
{
	long tmr = 0;
	long err = 0xFF;	
	while ((tmr!=Max_I2C_delay_cls)&(err))
	{
		tmr++;																  
		err=!I2C_CheckEvent(I2Cx, event);						  
	}
	if (tmr==Max_I2C_delay_cls)
		{
			
			return 1;	   						//ERROR
		}
	else
		{return 0;}							//OK
}

void EE24PrintString(I2C_TypeDef* I2Cx, char * string, int adrs)
{
	int cnt=adrs;
	while (*string != 0x00)
	{ 
		EE24WriteByte(I2Cx, *string++,cnt);
		cnt++;
		delay_ms(EE24_Write_Delay_ms);
	}
}

