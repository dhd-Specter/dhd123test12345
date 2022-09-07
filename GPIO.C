

#include "at32f4xx.h"
#include "spi.h"



void GPIO_vInit(void)
{               

  GPIO_InitType   GPIO_InitStructure;


	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);//1234
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_SYSCFGCOMP, ENABLE);         ///<Enable SYSCFG clock	
	
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_13;       //外部指示灯
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_IN;//
  GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;//GPIO_PuPd_UP;//
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_2 | GPIO_Pins_10 | GPIO_Pins_11 | GPIO_Pins_6;       //PB6监控数据
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
  GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;//
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_PU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
              
}





