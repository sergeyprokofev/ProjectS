#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include <stdio.h>


/***************************************************************************//**
 * Global variables, private define and typedef
 ******************************************************************************/
#define RTCClockOutput_Enable  /* RTC Clock/64 is output on tamper pin(PC.13) */
#define COM1 0
#define COM2 1
__IO uint32_t  TimeDisplay = 0;

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void RTC_Configuration(void);
void NVIC_Configuration(void);
uint32_t Time_Regulate(void);
void Time_Adjust(void);
void Time_Show(void);
void Time_Display(uint32_t TimeVar);
uint8_t USART_Scanf(uint32_t value);
void RTC_IRQHandler(void);
void STM_COMInit(uint16_t COM, USART_InitTypeDef* USART_InitStruct);


/**
  * @brief  The Low Speed External (LSE) clock is used as RTC clock source.
  * The RTC clock can be output on the Tamper pin (PC.13). To enable this functionality,
  * uncomment the corresponding line: #define RTCClockOutput_Enable.
  * The RTC is in the backup (BKP) domain, still powered by VBAT when VDD is switched off,
  * so the RTC configuration is not lost if a battery is connected to the VBAT pin.
  * A key value is written in backup data register1 (BKP_DR1) to indicate if the RTC
  * is already configured.
  * @param  None
  * @retval None
  */
void RTC_Timer(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Initialize LED which connected on PB8, Enable the Clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_COMInit(COM1, &USART_InitStructure);

    /* NVIC configuration */
    NVIC_Configuration();

    if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
    {
        /* Backup data register value is not correct or not yet programmed (when
           the first time the program is executed) */

        printf("\r\n\n RTC not yet configured....");

        /* RTC Configuration */
        RTC_Configuration();

        printf("\r\n RTC configured....");

        /* Adjust time by values entred by the user on the hyperterminal */
        Time_Adjust();

        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    }
    else
    {
        /* Check if the Power On Reset flag is set */
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            printf("\r\n\n Power On Reset occurred....");
        }
        /* Check if the Pin Reset flag is set */
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            printf("\r\n\n External Reset occurred....");
        }

        printf("\r\n No need to configure RTC....");
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();

        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }

#ifdef RTCClockOutput_Enable
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Disable the Tamper Pin */
    BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                                 functionality must be disabled */

    /* Enable RTC Clock Output on Tamper Pin */
    BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
#endif

    /* Clear reset flags */
    RCC_ClearFlag();

    /* Display time in infinite loop */
    Time_Show();
}

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);

        /* Toggle LED1 */
        //STM_EVAL_LEDToggle(LED1);
        GPIOB->ODR ^= GPIO_Pin_8;

        /* Enable time update */
        TimeDisplay = 1;

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
}

/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg 0, COM1
  *     @arg 1, COM2
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_COMInit(uint16_t COM, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_TypeDef* COM_USART[2] = {USART1, USART2};
    GPIO_TypeDef*  COM_TX_PORT[2] = {GPIOA, GPIOD};
    GPIO_TypeDef*  COM_RX_PORT[2] = {GPIOA, GPIOD};
    const uint32_t COM_USART_CLK[2] = {RCC_APB2Periph_USART1, RCC_APB1Periph_USART2};
    const uint32_t COM_TX_PORT_CLK[2] = {RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOD};
    const uint32_t COM_RX_PORT_CLK[2] = {RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOD};
    const uint16_t COM_TX_PIN[2] = {GPIO_Pin_9, GPIO_Pin_5};
    const uint16_t COM_RX_PIN[2] = {GPIO_Pin_10, GPIO_Pin_6};

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);


    /* Enable UART clock */
    if (COM == COM1)
    {
        RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }
    else
    {
        /* Enable the USART2 Pins Software Remapping */
        GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
        RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
    GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(COM_USART[COM], USART_InitStruct);

    /* Enable USART */
    USART_Cmd(COM_USART[COM], ENABLE);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    	int a = 0;
    }

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval Current time RTC counter value
  */
uint32_t Time_Regulate(void)
{
    uint32_t Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;

    printf("\r\n==============Time Settings=====================================");
    printf("\r\n  Please Set Hours");

    while (Tmp_HH == 0xFF)
    {
        Tmp_HH = USART_Scanf(23);
    }
    printf(":  %d", Tmp_HH);
    printf("\r\n  Please Set Minutes");
    while (Tmp_MM == 0xFF)
    {
        Tmp_MM = USART_Scanf(59);
    }
    printf(":  %d", Tmp_MM);
    printf("\r\n  Please Set Seconds");
    while (Tmp_SS == 0xFF)
    {
        Tmp_SS = USART_Scanf(59);
    }
    printf(":  %d", Tmp_SS);

    /* Return the value to store in RTC counter register */
    return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}

/**
  * @brief  Adjusts time.
  * @param  None
  * @retval None
  */
void Time_Adjust(void)
{
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Change the current time */
    RTC_SetCounter(Time_Regulate());
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

/**
  * @brief  Displays the current time.
  * @param  TimeVar: RTC counter value.
  * @retval None
  */
void Time_Display(uint32_t TimeVar)
{
    uint32_t THH = 0, TMM = 0, TSS = 0;

    /* Reset RTC Counter when Time is 23:59:59 */
    if (RTC_GetCounter() == 0x0001517F)
    {
        RTC_SetCounter(0x0);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }

    /* Compute  hours */
    THH = TimeVar / 3600;
    /* Compute minutes */
    TMM = (TimeVar % 3600) / 60;
    /* Compute seconds */
    TSS = (TimeVar % 3600) % 60;

    printf("Time: %d:%d:%2d\r", THH, TMM, TSS);
}

/**
  * @brief  Shows the current time (HH:MM:SS) on the Hyperterminal.
  * @param  None
  * @retval None
  */
void Time_Show(void)
{
    printf("\n\r");

    /* Infinite loop */
    while (1)
    {
        /* If 1s has been elapased */
        if (TimeDisplay == 1)
        {
            /* Display current time */
            Time_Display(RTC_GetCounter());
            TimeDisplay = 0;
        }
    }
}

/**
  * @brief  Gets numeric values from the hyperterminal.
  * @param  None
  * @retval None
  */
uint8_t USART_Scanf(uint32_t value)
{
    uint32_t index = 0;
    uint32_t tmp[2] = {0, 0};

    while (index < 2)
    {
        /* Loop until RXNE = 1 */
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
        {}
        tmp[index++] = (USART_ReceiveData(USART1));
        if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
        {
            printf("\n\rPlease enter valid number between 0 and 9");
            index--;
        }
    }
    /* Calculate the Corresponding value */
    index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
    /* Checks */
    if (index > value)
    {
        printf("\n\rPlease enter valid number between 0 and %d", value);
        return 0xFF;
    }
    return index;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif
