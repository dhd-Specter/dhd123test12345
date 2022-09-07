
#include "SciToOprt.h"
#include "performance.h"
#include "function.h"
#include "ctrlmode.h"
//Global variables used in this system
static const u16 sin_swSinTab[SINTABSIZE] = { 0, \
                    804         , \
                   1609         , \
                   2410         , \
                   3212         , \
                   4011         , \
                   4808         , \
                   5602         , \
                   6393         , \
                   7179         , \
                   7962         , \
                   8739         , \
                   9512         , \
                   10278        , \
                   11039        , \
                   11793        , \
                   12539        , \
                   13279        , \
                   14010        , \
                   14732        , \
                   15446        , \
                   16151        , \
                   16846        , \
                   17530        , \
                   18204        , \
                   18868        , \
                   19519        , \
                   20159        , \
                   20787        , \
                   21403        , \
                   22005        , \
                   22594        , \
                   23170        , \
                   23731        , \
                   24279        , \
                   24812        , \
                   25329        , \
                   25832        , \
                   26319        , \
                   26790        , \
                   27245        , \
                   27683        , \
                   28105        , \
                   28510        , \
                   28898        , \
                   29268        , \
                   29621        , \
                   29956        , \
                   30273        , \
                   30571        , \
                   30852        , \
                   31113        , \
                   31356        , \
                   31580        , \
                   31785        , \
                   31971        , \
                   32137        , \
                   32285        , \
                   32412        , \
                   32521        , \
                   32609        , \
                   32678        , \
                   32728        , \
                   32757        , \
                   32767        , \
                   32757        , \
                   32728        , \
                   32678        , \
                   32609        , \
                   32521        , \
                   32412        , \
                   32285        , \
                   32137        , \
                   31971        , \
                   31785        , \
                   31580        , \
                   31356        , \
                   31113        , \
                   30852        , \
                   30571        , \
                   30273        , \
                   29956        , \
                   29621        , \
                   29268        , \
                   28898        , \
                   28510        , \
                   28105        , \
                   27683        , \
                   27245        , \
                   26790        , \
                   26319        , \
                   25832        , \
                   25329        , \
                   24811        , \
                   24279        , \
                   23731        , \
                   23170        , \
                   22594        , \
                   22005        , \
                   21403        , \
                   20787        , \
                   20159        , \
                   19519        , \
                   18868        , \
                   18204        , \
                   17530        , \
                   16846        , \
                   16151        , \
                   15446        , \
                   14732        , \
                   14010        , \
                   13279        , \
                   12538        , \
                   11793        , \
                   11039        , \
                   10278        , \
                   9512         , \
                   8739         , \
                   7962         , \
                   7179         , \
                   6393         , \
                   5602         , \
                   4808         , \
                   4011         , \
                   3212         , \
                   2410         , \
                   1608         , \
                    804         , \
                     0          , \
                   64731        , \
                   63927        , \
                   63125        , \
                   62323        , \
                   61524        , \
                   60727        , \
                   59933        , \
                   59142        , \
                   58356        , \
                   57573        , \
                   56796        , \
                   56023        , \
                   55257        , \
                   54496        , \
                   53742        , \
                   52996        , \
                   52256        , \
                   51525        , \
                   50803        , \
                   50089        , \
                   49384        , \
                   48689        , \
                   48005        , \
                   47331        , \
                   46667        , \
                   46016        , \
                   45377        , \
                   44748        , \
                   44132        , \
                   43530        , \
                   42941        , \
                   42365        , \
                   41804        , \
                   41256        , \
                   40724        , \
                   40206        , \
                   39703        , \
                   39216        , \
                   38745        , \
                   38290        , \
                   37852        , \
                   37430        , \
                   37025        , \
                   36637        , \
                   36267        , \
                   35914        , \
                   35579        , \
                   35262        , \
                   34964        , \
                   34682        , \
                   34422        , \
                   34179        , \
                   33955        , \
                   33750        , \
                   33564        , \
                   33398        , \
                   33250        , \
                   33123        , \
                   33014        , \
                   32926        , \
                   32857        , \
                   32807        , \
                   32778        , \
                   32769        , \
                   32778        , \
                   32807        , \
                   32857        , \
                   32926        , \
                   33014        , \
                   33123        , \
                   33250        , \
                   33398        , \
                   33564        , \
                   33750        , \
                   33955        , \
                   34179        , \
                   34422        , \
                   34683        , \
                   34964        , \
                   35262        , \
                   35579        , \
                   35914        , \
                   36267        , \
                   36637        , \
                   37025        , \
                   37430        , \
                   37852        , \
                   38290        , \
                   38745        , \
                   39216        , \
                   39703        , \
                   40206        , \
                   40724        , \
                   41256        , \
                   41804        , \
                   42365        , \
                   42941        , \
                   43530        , \
                   44132        , \
                   44748        , \
                   45376        , \
                   46016        , \
                   46667        , \
                   47331        , \
                   48005        , \
                   48689        , \
                   49384        , \
                   50089        , \
                   50802        , \
                   51525        , \
                   52256        , \
                   52996        , \
                   53742        , \
                   54496        , \
                   55257        , \
                   56023        , \
                   56796        , \
                   57573        , \
                   58356        , \
                   59142        , \
                   59933        , \
                   60727        , \
                   61524        , \
                   62323        , \
                   63125        , \
                   63927        , \
                   64731        , \
                   65535        }; //         256      360.00    0.0000


u16 mth_uwSqrt(u32 M)
{
  int N, i;
  int tmp, ttp;
    
  if (M == 0) return 0;            
    
  N = 0;
  tmp = (M >> 30);
  M <<= 2;
  if (tmp > 1)
	{
    N ++;
    tmp -= N;
  }
    
  for (i=15; i>0; i--)
	{
    N <<= 1;
        
    tmp <<= 2;
    tmp += (M >> 30);
        
    ttp = N;
    ttp = (ttp<<1)+1;
        
    M <<= 2;
    if (tmp >= ttp)
	  {
      tmp -= ttp;
      N ++;
    }
  }
  return N;
}
void sin_voSinCos(u16 degree,SINCOS *v)
{
  s16	sin_swFisDeg,	sin_swNxtDeg,	sin_swDltDeg;
  u16 sin_uwDegVal;

//=======================================================================
//    Calculate Cos(degree_p)
//=======================================================================
  sin_swFisDeg = sin_swSinTab[((degree + 8192) & 0x7fff)>>7];    //High 8 bit, 32767 to 255
  sin_swNxtDeg = sin_swSinTab[(((degree + 8192) & 0x7fff)>>7) + 1 ];
  sin_swDltDeg = sin_swNxtDeg - sin_swFisDeg;
	if(sin_swDltDeg > 255) sin_swDltDeg = 255;
	if(sin_swDltDeg < -255) sin_swDltDeg = -255;
  sin_uwDegVal = (degree & 0x007f);    //Low 7 bit, 127 to 32512
  v->swCos = (sin_swDltDeg * sin_uwDegVal >> 7) + sin_swFisDeg;

//=======================================================================
//    Calculate Sin(degree_p)
//=======================================================================
  sin_swFisDeg = sin_swSinTab[(degree & 0x7fff)>>7];    //High 8 bit, 32767 to 255
  sin_swNxtDeg = sin_swSinTab[((degree & 0x7fff)>>7) + 1 ];
  sin_swDltDeg = sin_swNxtDeg - sin_swFisDeg;
	if(sin_swDltDeg > 255) sin_swDltDeg = 255;
	if(sin_swDltDeg < -255) sin_swDltDeg = -255;
  sin_uwDegVal = (degree & 0x007f);    //Low 7 bit, 127 to 32512
  v->swSin = (sin_swDltDeg * sin_uwDegVal >> 7) + sin_swFisDeg;
}

void cdt_Clarke (CDT_CLARKE_IN *in,CDT_CLARKE_OUT *out)
{
  SLONG_UNION Temp;

  Temp.sl = (s32)in->swa * 21846- (s32)in->swb * 10923 - (s32)in->swc * 10923;

  if (ulabs(Temp.sl) < 0x3fffffff)
  {
  	Temp.sl = Temp.sl<<1;
    out->swAlfa= Temp.sw.hi;
  }
  else
  {
    if (Temp.sl> 0)
	    out->swAlfa = 0x7fff;
    else
	    out->swAlfa = 0x8001;
	}
//	out->swAlfa = Temp.sw.hi;
	//iAlfa = 2/3 * ia - 1/3 * ib - 1/3 * ic

	Temp.sl = (s32)in->swb * 18919 - (s32)in->swc * 18919;

//	out->swBeta = Temp.sw.hi;
	//ibeta = 1 / sqrt(3) (ib - ic)

  if (ulabs(Temp.sl) < 0x3fffffff)
	{
  	Temp.sl = Temp.sl<<1;
    out->swBeta= Temp.sw.hi;
  }
  else
  {
    if (Temp.sl> 0)
	    out->swBeta = 0x7fff;
    else
	    out->swBeta = 0x8001;
	}

}

void cdt_InvClarke (CDT_INVCLARKE_IN *in,CDT_INVCLARKE_OUT *out)
{
  //Ia = IAlfa
  //Ib = (sqrt(3)*Ibeta - IAlfa)/2
  //Ic = -(sqrt(3)*Ibeta + IAlfa)/2

  SLONG_UNION Temp;
  //Ia = IAlfa
  out->swa = in->swAlfa;

  //Ib = (sqrt(3)*Ibeta - IAlfa)/2
  Temp.sl = (s32)in->swBeta * 56755 - ((s32)in->swAlfa << 15);
  out->swb= Temp.sw.hi;

  //Ic = -(sqrt(3)*Ibeta + IAlfa)/2
  Temp.sl = (s32)in->swBeta * 56755 + ((s32)in->swAlfa << 15);
  out->swc= - Temp.sw.hi;
}

void cdt_Park(CDT_PARK_IN *in,CDT_PARK_OUT *out)
{
  ULONG_UNION tmp;
  SLONG_UNION Temp;
  SINCOS Sincos;

  tmp.ul = in->uldegree;  
  sin_voSinCos(tmp.uw.hi ,&Sincos);

  Temp.sl = (s32)in->swAlfa * Sincos.swCos + (s32)in->swBeta * Sincos.swSin;  
  if (ulabs(Temp.sl) < 0x3fffffff)
  {
    Temp.sl = Temp.sl<<1;
    out->swd= Temp.sw.hi;
  }
  else
  {
    if (Temp.sl> 0)
      out->swd = 0x7fff;
    else
      out->swd = 0x8001;
  }
 
  Temp.sl = - (s32)in->swAlfa * Sincos.swSin + (s32)in->swBeta * Sincos.swCos;
  if (ulabs(Temp.sl) < 0x3fffffff)
  {
    Temp.sl = Temp.sl<<1;
    out->swq= Temp.sw.hi;
  }
  else
  {
    if (Temp.sl> 0)
      out->swq = 0x7fff;
    else
      out->swq = 0x8001;
  }
}



void cdt_InvPark(CDT_INVPARK_IN *in,CDT_INVPARK_OUT*out)
{

  ULONG_UNION temp1;
  SLONG_UNION Temp;
  SINCOS Sincos;

  temp1.ul = in->uldegree;
  sin_voSinCos(temp1.uw.hi ,&Sincos);

  Temp.sl = (s32)in->swd * Sincos.swCos - (s32)in->swq * Sincos.swSin;

  if (ulabs(Temp.sl) < 0x3fffffff)
	{
  	Temp.sl = Temp.sl<<1;
    out->swAlfa= Temp.sw.hi;
  }
  else
	{
    if (Temp.sl> 0)
      out->swAlfa = 0x7fff;
    else
      out->swAlfa = 0x8001;
	}

  Temp.sl = (s32)in->swd * Sincos.swSin + (s32)in->swq * Sincos.swCos;
  if (ulabs(Temp.sl) < 0x3fffffff)
	{
  	Temp.sl = Temp.sl<<1;
    out->swBeta= Temp.sw.hi;
  }
  else
	{
    if (Temp.sl> 0)
      out->swBeta = 0x7fff;
    else
      out->swBeta = 0x8001;
	}
}

void PWM_EN_DUTY(void)
{
  TMR1->CCE |= 0x1fff;
}

void PWM_DIS_DUTY(void)
{
  TMR1->CCE &= 0xfaaa;
}


void ParDefault(u16 numstart,u16 numEnd)
{
	u16 temp;

	temp = numstart;
	for( ;temp<numEnd;temp++ )
	{
		*(SysParaAttrCharNEW[temp].ParAdd) = SysParaAttrCharNEW[temp].OrgVal0;
	}

}

u16 uwabs(s16 in)
{
	u16 out;

	if(in<0)
	 out = -in;
	else
	 out = in;
	
	return(out); 
}

u32 ulabs(s32 in)
{
	u32 out;

	if(in<0)
	 out = -in;
	else
	 out = in; 

  return(out); 
}

void mpd_STOP(void)
{
  TMR1->CCE |= 0x1fff;  

  TMR1->CC1 = 0;	
  TMR1->CC2 = 0;	
  TMR1->CC3 = 0;			 //下管子打开，上管子全部关闭
}
