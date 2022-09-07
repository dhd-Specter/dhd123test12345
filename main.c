
#include "at32f4xx.h"
#include "function.h"
#include "SciToOprt.h"
#include "SEGGER_RTT.h"
#include "ntc.h"
#include "spi.h"
#include "ctrlmode.h"
#include "application.h"
u32 initDelay;

u16  uwKeySpeedUartOrg,uwKeySpeedUartOrg1,uwKeySpeedUartOrg2,uwKeySpeedRefRec,uwKeySpeedRefFil; 
u16  TEMP_uACVol,TEMP_uwDisSpdFdb,TEMP_uwIqFdb;

extern void TIM1_vInit(void);
extern void ADC_DMA_vInit(void);
extern void GPIO_vInit(void);
extern void SysTick_vInit(void);
extern void NVIC_Configure(void);




int main(void)
{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_DBGMCU, ENABLE);

  for(initDelay = 0; initDelay < 450000;initDelay ++)
  {
    __nop();
  }
	ParDefault(0,(120));	
	
	DelayTime = 1000;//1S
	
	GPIO_vInit();		


  g_lDisBusVol = 0;
  g_uVdcDis =3100;

	TIM1_vInit();
	ADC_DMA_vInit();
  SysTick_vInit();
  NVIC_Configure();
	

	SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel, "JScope_I2I2I2I2", &JS_RTT_UpBuffer[0], sizeof(JS_RTT_UpBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  uwKeySpeedRefFil = 1500;
//	RunCmd = 0;
		
	while (1)
  {
		
	
		ctm_CtrlModeSchdResv();
		
    if((DelayTime == 0)&&(g_uErrorCode == C_NO_ERR))
		{
			
			RunCmd = 1;
			if(SpeedSourceTemp >= 50)  RunCmd = 1;
			 else if(SpeedSourceTemp <= 30) 
			 {
				 RunCmd = 0;
		   }
		}


		
		if(uwDisSpdFdb==0) 
		{
			TEMP_uwDisSpdFdb=0;
		}	
		else
		{
			TEMP_uwDisSpdFdb=(unsigned long)uwDisSpdFdb*2/pr[PM_POLES];     //
		}
  }
}


