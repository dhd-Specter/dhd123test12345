/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    10-October-2011
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTION_H
#define __FUNCTION_H

#include "performance.h"




u16 mth_uwSqrt(u32 M);
void sin_voSinCos(u16 degree,SINCOS *v);
void cdt_Clarke(CDT_CLARKE_IN *in,CDT_CLARKE_OUT *out);
void cdt_InvClarke (CDT_INVCLARKE_IN *in,CDT_INVCLARKE_OUT *out);
void cdt_Park(CDT_PARK_IN *in,CDT_PARK_OUT *out);
void cdt_InvPark(CDT_INVPARK_IN *in,CDT_INVPARK_OUT *out);

void PWM_EN_DUTY(void);
void PWM_DIS_DUTY(void);
void ParDefault(u16 numstart,u16 numEnd);

u16 uwabs(s16 in);
u32 ulabs(s32 in);

void mpd_STOP(void);
#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
