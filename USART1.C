


#include "ctrlmode.h"
#include "function.h"	
#include "SciToOprt.h"
#include "application.h"
#include "spi.h"
#include "ntc.h"
#include "core_cm4.h"

unsigned char Send[8];
unsigned char SendCnt;
unsigned short Send_CRC;
unsigned short SendTime;

unsigned char Recv[8];
unsigned short Recv_CRC;
unsigned char uartstp;
unsigned char uartskp;
unsigned char uarttim;
unsigned char RunFlag;
unsigned short SetFreq = 0xc8;

extern unsigned int CRC16(unsigned char *udata, unsigned char length);


void USART1_vInit(void)
{
 
 	USART_InitType     USART_InitStructure;
	GPIO_InitType      GPIO_InitStructure;

	
	
  RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE );
	
/* Configure PB6 in as USART1_TX */  
  GPIO_InitStructure.GPIO_Pins   =  GPIO_Pins_6;
  GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OutType =  GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_MaxSpeed =  GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Pull  =  GPIO_Pull_NOPULL;     
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
/* Configure PB7 in as USART1_RX*/  
  GPIO_InitStructure.GPIO_Pins   =  GPIO_Pins_7;
  GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pull  =  GPIO_Pull_NOPULL;     
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinsSource6,  GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, GPIO_PinsSource7,  GPIO_AF_0);	
	
	USART_Reset(USART1);
	USART_StructInit(&USART_InitStructure);
	
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);	   
	
	
	USART_INTConfig(USART1, USART_INT_RDNE, ENABLE);
  USART_INTConfig(USART1, USART_INT_TRAC, ENABLE);

}



void USART1_IRQHandler(void)
{
	 if(USART_GetITStatus(USART1,USART_INT_RDNE) == SET)
	 {
		 
     uarttim = 0;
		 if(uartstp < 8) 	{Recv[uartstp] = USART1->DT; uartstp++;}
		 
		 USART_ClearITPendingBit(USART1,USART_INT_RDNE);
	 }
	 
	 if(USART_GetITStatus(USART1,USART_INT_TRAC) == SET)
	 {
     
		 
		 if(SendCnt < 8) 
		 {
		  USART1->DT = Send[SendCnt];
			SendCnt++;
		 }
		 else SendCnt = 0;
		 
		 USART_ClearITPendingBit(USART1,USART_INT_TRAC);
	 }
}




void USART1_Send(void)
{
  Send[0] = g_uErrorCode;  
	Send[1] = adAverageCurrent;
	Send[2] = uwTemper;
  Send[3] = 0x00;
	Send[4] = (TEMP_uwDisSpdFdb >> 8);   
	Send[5] = (TEMP_uwDisSpdFdb & 0xff);

	Send_CRC = CRC16(Send,6);
	Send[6] = (unsigned char)((Send_CRC&0xff00)>>8);
	Send[7] = (unsigned char)(Send_CRC&0xff);
  USART1->DT = Send[0];
	SendCnt = 1;
}

void USART1_Recv(void)
{
	unsigned short tmp;
	
  tmp =  ((((unsigned int)Recv[6])<<8)|((unsigned int)Recv[7]));
	
	Recv_CRC = CRC16(Recv,6);
	
	if(tmp == Recv_CRC)
	{
    if(Recv[2] == 0x01) RunCmd = 1;
		 else if(Recv[2] == 0x00)  RunCmd = 0;
		
		uwKeySpeedRefFil = (((u16)Recv[0] << 8) + Recv[1])*pr[PM_POLES]/2;
		if(uwKeySpeedRefFil > 3500)  uwKeySpeedRefFil = 3500;
	}
}


void USART1_Recv_Monitor(void)
{
 if(uartstp == uartskp)	 {if(uarttim<5)uarttim++; }
	switch(uartstp)
	{
	  case 0:   uarttim=0; 
	  break;
	  case 1:
	  case 2:
	  case 3:
	  case 4:
	  case 5:
	  case 6:
	  case 7:
			 	if(uarttim>1){uartstp=0;uartskp=0;uarttim=0;}
	  break;
	  case 8:	if(uarttim==0) USART1_Recv(); 
				     else  {uartstp=0;uartskp=0;uarttim=0;}
            
	  break;

	  default: if(uarttim>1){uartstp=0;uartskp=0;uarttim=0;	}
	  break;
	}

	 uartskp = uartstp;
}





//CRC产生的功能
unsigned int CRC16(unsigned char *udata, unsigned char length)
{
	unsigned char i; 
	unsigned int tmp_crc;
	unsigned int reg_crc = 0xffff; 
	while(length--){	 
	reg_crc ^= *udata++;
	for(i=0;i<8;i++)
	{
		if(reg_crc&0x01)
		{ // LSB(b0)=1 
	  	  reg_crc=(reg_crc>>1)^0xa001;
		}
		else
	 	{ reg_crc=reg_crc >>1; }
	}
  }
  tmp_crc = reg_crc; 
  reg_crc =	((reg_crc<<8)&0xFF00);
  tmp_crc =	((tmp_crc>>8)&0x00FF);
  reg_crc = reg_crc + tmp_crc;
  return(reg_crc); // 最后回传CRC 暂存器的值
}  





