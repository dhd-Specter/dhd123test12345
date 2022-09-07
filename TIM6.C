

#include "at32f4xx.h"
#include "ctrlmode.h"
#include "function.h"	
#include "SciToOprt.h"
#include "application.h"
#include "ntc.h"
#include "spi.h"



void TIM6_vInit(void)
{
    TMR_TimerBaseInitType  TIM_TimeBaseStructure;
    NVIC_InitType          NVIC_InitStructure;
	
	
	  RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6,ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TMR_DIV = 47;   //48M/(47+1)= 1M
    TIM_TimeBaseStructure.TMR_Period = 1000;    //1000/1M = 0.004S	= 1mS
    TIM_TimeBaseStructure.TMR_ClockDivision = 0;
    TIM_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;

    TMR_TimeBaseInit(TMR6, &TIM_TimeBaseStructure);

    /* Enable the TIM6 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TMR6_GLOBAL_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	


    /* TIM Interrupts enable */
    TMR_INTConfig(TMR6, TMR_FLAG_Update, ENABLE);

    /* TIM6 enable counter */
    TMR_Cmd(TMR6, ENABLE);	 
}

void TMR6_GLOBAL_IRQHandler(void)
{

	
 if(TMR_GetFlagStatus(TMR6,TMR_FLAG_Update) != RESET)
	{
		TMR_ClearFlag(TMR6, TMR_FLAG_Update);
		


		
		
	}
	
	
	
}




