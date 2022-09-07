

#ifndef __SCITOOPRT_H__
#define __SCITOOPRT_H__

#include "at32f4xx.h"


#define MOTORFLASH




/*------------	Group0	 --------*/
#define	GROUP0			0
#define	RELS0			  (GROUP0+0)		//	0	00-00	Release
#define	RELS55			(GROUP0+1)		//	1	00-01	Release
#define	RELS56			(GROUP0+2)		//	2	00-02	Release
#define	RELS57			(GROUP0+3)		//	3	00-03	Release
#define	RELS58			(GROUP0+4)		//	4	00-04	Release
#define	RELS59			(GROUP0+5)		//	5	00-05	Release
#define	RELS60			(GROUP0+6)		//	6	00-06	Release
/*------------	Group1	 --------*/
#define	GROUP1			(GROUP0+7)
#define	RELS29			(GROUP1+0)		//	7	01-00	Release
#define	RELS30			(GROUP1+1)		//	8	01-01	Release
#define	RELS31			(GROUP1+2)		//	9	01-02	Release
#define	RELS32			(GROUP1+3)		//	10	01-03	Release
#define	RELS33			(GROUP1+4)		//	11	01-04	Release
#define	RELS34			(GROUP1+5)		//	12	01-05	Release
#define	RELS35			(GROUP1+6)		//	13	01-06	Release
#define	RELS36			(GROUP1+7)		//	14	01-07	Release
#define	RELS37			(GROUP1+8)		//	15	01-08	Release
#define	RELS38			(GROUP1+9)		//	16	01-09	Release
#define	RELS1			(GROUP1+10)		//	17	01-10	Release
#define	RELS2			(GROUP1+11)		//	18	01-11	Release
#define	RELS3			(GROUP1+12)		//	19	01-12	Release
#define	RELS4			(GROUP1+13)		//	20	01-13	Release
#define	RELS5			(GROUP1+14)		//	21	01-14	Release
#define	RELS6			(GROUP1+15)		//	22	01-15	Release
#define	RELS7			(GROUP1+16)		//	23	01-16	Release
#define	RELS8			(GROUP1+17)		//	24	01-17	Release
#define	RELS9			(GROUP1+18)		//	25	01-18	Release
#define	RELS10			(GROUP1+19)		//	26	01-19	Release
#define	RELS11			(GROUP1+20)		//	27	01-20	Release

#define	GROUP2			(GROUP1+21)
#define	AUTOCUR			(GROUP2+0)		//	28	02-00	Auto tuning Current(0.1A)
#define	SPDMAX  		(GROUP2+1)		//	29	02-01	FLW_ID_GAIN
#define	RATED_RPM		(GROUP2+2)		//	30	02-02	Motor  Rated RPM
#define	PM_POLES		(GROUP2+3)  	//	31	02-03	Motor  pole No.
#define RATED_POW          (GROUP2+4)		//  32  02-04	NS Cnt
#define SPDLOOPEN         (GROUP2+5)		//  33  02-05	SPDLOOPEN
#define POW_LIM         (GROUP2+6)		//  34  02-06	Fun Cur LPF
#define POWLIM_EN         (GROUP2+7)		//  35  02-07	PLL BW
#define SPD_SW          (GROUP2+8)		//  36  02-08	SPD_SW
#define FLW_ID_GAIN          (GROUP2+9)		//  37  02-09	FLW_ID_GAIN
#define SUN_EN        (GROUP2+10)		//  38  02-10	HFVI LPF
#define FLW_ID_INTE     (GROUP2+11)		//  39  02-11	FLW_ID_INTE
#define SLR_K           (GROUP2+12)		//  40  02-12	0:Common ASR  1:New ASR
#define HFVI            (GROUP2+13)		//  41  02-13	HFVI Degree Com
#define TFED_LPF        (GROUP2+14)		//  42  02-14	New ASR Tor Fed LPF Unit:0.01s
#define SPDHALL180      (GROUP2+15)		//  43	02-15	SPDHALL180
#define IniUMag         (GROUP2+16)		//  44  02-16	IniUMag
#define	I_RATE			(GROUP2+17)		//	45	02-17	MOTOR RATED Cur 0.1A
#define	SW_VER			(GROUP2+18)		//	46	02-18	Software Version
#define VELCMD		    (GROUP2+19)		//	47	02-19	Velocity command(NM/0.01Hz)
#define	CTRLM			(GROUP2+20)		//	48	02-20	Control Methods
#define	PWMFRE			(GROUP2+21)		//	49	02-21	Carry Frequency
#define	VELDIR			(GROUP2+22)		//	50	02-22	Reverse Operation
#define IsREF           (GROUP2+23)		//	51	02-23	Stator current reference(PU value, Q15)

/*------------	Group3	 Control parameter--------*/
#define	GROUP3			(GROUP2+24)
#define	FB		    	(GROUP3+0)		//	52	03-00	Freq. Base value(0.1Hz)
#define	IniMode		    (GROUP3+1)		//	53	03-01	IniMode
#define	MAG_VOLT		(GROUP3+2)		//	54	03-02	MAG_VOLT
#define	MAG_TIME		(GROUP3+3)		//	55	03-03	MAG_TIME
#define	IFCurACC		(GROUP3+4)		//	56	03-04	IFCurACC
#define	IQ_LIM			(GROUP3+5)		//	57	03-05	IQ_LIM
#define	ATT_DURATION	(GROUP3+6)		//	58	03-06	DC testing each section duration.(ms)
#define	SPD_BST	    	(GROUP3+7)		//	59	03-07	SPD_BST
#define	IFFOCSPD		(GROUP3+8)		//	60	03-08	IFFOCSPD
#define IFCURAUTOSPD	(GROUP3+9)		//	61	03-09 	IFCURAUTOSPD
#define THEHALL180		(GROUP3+10)		//	62	03-10  	THEHALL180
#define	VELCMDUPLIM		(GROUP3+11)		//	63	03-11	VELCMDUPLIM
#define	VELCMDDOWNLIM	(GROUP3+12)		//	64	03-12	VELCMDDOWNLIM
#define	CLR_D_KP		(GROUP3+13)		//	65	03-13	Cur_P
#define	CLR_D_KI		(GROUP3+14)		//	66	03-14	Cur_I
#define	RunTimeLimCnt 			(GROUP3+15)		//	67	03-15	RunTimeLimCnt
#define	RunTimLimEn		  	(GROUP3+16)		//	68	03-16 	RunTimLimEn
#define	MOTORNUM		  	(GROUP3+17)		//	69	03-17 	MOTORNUM
#define	NS_UCMD		    (GROUP3+18)		//	70	03-18	NS U_CMD Q15
#define	J_MOTOR		    (GROUP3+19)		//	71	03-19	Inertia of motor
#define	ID_REF    		(GROUP3+20)		//	72	03-20	ID_REF
#define	TE_LIM          (GROUP3+21)		//	73	03-21	Te_Lim
#define	VF_FBASE		(GROUP3+22)		//	74	03-22	Base Freqency(0.01Hz)
#define	VMAX			(GROUP3+23)		//	75	03-23	Max Output Voltage(0.1V)
#define	VF_FMID1		(GROUP3+24)		//	76	03-24	Mid Output Freq.1(0.01Hz)
#define	VF_VMID1		(GROUP3+25)		//	77	03-25	Mid Output Voltage 1(0.1V)
#define	VF_FMID2		(GROUP3+26)		//	78	03-26	Mid Output Freq.2(0.01Hz)
#define	VF_VMID2		(GROUP3+27)		//	79	03-27	Mid Output Voltage 2(0.1V)
#define	FMIN			(GROUP3+28)		//	80	03-28	Min Output Freq.(0.01Hz)
#define	VMIN			(GROUP3+29)		//	81	03-29	Min Output Voltage(0.1V)
#define	ACCTIME			(GROUP3+30)		//	82	03-30	1st Acceleration time(0.1s)
#define	DECTIME			(GROUP3+31)		//	83	03-31	1st Deceleration time(0.1s)
#define	RELS39    		(GROUP3+32)		//	84	03-32	Release
#define	RELS40      	(GROUP3+33)		//	85	03-33	Release
#define	RELS41       	(GROUP3+34)		//	86	03-34	Release
#define	RELS42			(GROUP3+35)		//	87	03-35	Release
#define	RELS43			(GROUP3+36)		//	88	03-36	Release
#define DTCRATIO 		(GROUP3+37)		//	89	03-37	DTC Linear Ratio.Q16
#define DTCDu			(GROUP3+38)		//	90	03-38	DTC Voltage Drop Coef.Q18
#define RELS44			(GROUP3+39)		//	91	03-39	Release
#define	RELS45			(GROUP3+40)		//	92	03-40	Release
#define	RELS46			(GROUP3+41)		//	93	03-41	Release
#define	RELS47			(GROUP3+42)		//	94	03-42	Release
#define SLR_SHAPE	  	(GROUP3+43)		//	96	03-44	Width of velocity loop mid-freq.Unit 0.1

/*------------	Group4	 parameter End--------*/
#define	GROUP4	        (GROUP3+44)		//
#define	TFED_EN	        (GROUP4+0)		//  96  04-00	New ASR Tor Feed En
#define	OB_BW	        (GROUP4+1)		//  97  04-01	Observer BW //Unit:0.1Hz
#define	SLR_ANTINOISE	(GROUP4+2)		//  98  04-02	New ASR Unit:0.1%   k
#define	PR_ID_CMD       (GROUP4+3)		//  99  04-03	Id_ref=pr[PR_ID_CMD]/1000*最大采样电流
#define INEG_DEL_SAL    (GROUP4+4)		//  100  04-04	200;//SAL Delay Angl
#define PR_SAL_MAG      (GROUP4+5)		//  101  04-05	SAL Inj Vol //Q15
#define RsPM            (GROUP4+6)		//  102  04-06	Me:687;//        //0.001om;
#define PMLD            (GROUP4+7)		//  103  04-07	0.01mH;
#define PMLQ            (GROUP4+8)		//  104  04-08	0.01mH;
#define IF_IQ_GAIN      (GROUP4+9)		//  105  04-09	IF_IQ_GAIN
#define IF_IQ_INTE      (GROUP4+10)		//  106  04-10	IF_IQ_INTE
#define BEMF            (GROUP4+11)		//  107  04-11	Motor Rated Vol //Unit:0.1V
#define SLR_HF_RSPS     (GROUP4+12)		//  108  04-12	New ASR Cf //Unit:%
#define JL_RATIO        (GROUP4+13)		//  109  04-13	New ASR Load J Ratio Cf //Unit:0.1
#define SLR_ANTIOVSHT   (GROUP4+14)		//  110  04-14	New ASR Vel Cmd LPF Cf //Unit:0.1%  TFED_LPF=1,SLR_ANTIOVSHT=100:10%超调  >300:无超调:
#define SWVER_YM			  (GROUP4+15)		//  111  04-15	SWVER_YM
#define SWVER_DV		    (GROUP4+16)		//  112  04-16	SWVER_DV
#define SLR_KP			(GROUP4+17)		//  113  04-17	ASR Kp %
#define SLR_KI			(GROUP4+18)		//  114  04-18	ASR Ki %
#define SLR_ERR_LPF		(GROUP4+19)		//  115  04-19	ASR Error LPF ms
#define BRAK_TIM  		(GROUP4+20)		//  116  04-20	BRAK_TIM
#define DTC_EN  		(GROUP4+21)		//  117  04-21	DBC Enable
#define PWM_MODE  		(GROUP4+22)		//  118  04-22	Ini Degree PWM_MODE
#define RELS53  		(GROUP4+23)		//  119  04-23	Release

#define	GROUP5	        (GROUP4+24)
#define	EEROM_EN	    (GROUP5+0)		//  120  05-00  Para Reset

#define	GROUP6	        (GROUP5+1)

#define	EPMAX		    (GROUP6+00)		//	95	06-00	Max index

/*-----	--------------		---------------------------*/
#define	PRMAX		EPMAX

#define	DSP_G0		(EPMAX +00)

typedef struct 
{	struct
	{ 		
		u16 ZoomLev:2;	//数据的放大倍数 00 -- 0倍， 01 -- 10倍，10 -- 100倍，11 -- 1000倍，
		u16 ChangePro: 2;// 数据修改属性，0 -- 不可以修改， 1 -- 运行时不能修改，10 -- 可以任何时刻修改，11 -- 在特定状态下才能修改
		u16 E2ROMBlock: 2;	//E2ROM块
		u16 E2ROMAdd	: 8;	//E2ROM地址
		u16 LowLmtPro: 1;	//取值范围:0--1如果为0:LowLmt是被设置参数的下限；如果为1:LowLmt为索引的另外一个参数
		u16 UpLmtPro: 1;	//取值范围:0--1如果为0:LowLmt是被设置参数的上限；如果为1:LowLmt为索引的另外一个参数
	}ParaPro;
	u16 RES;    //   未使用
	u16 OrgVal0;//	默认值
	u16 OrgVal1;//	未使用
	u16 LowLmt;	//被设置参数的下限置，或者下限值的索引
	u16 UpLmt;	//被设置参数的上限值，或者上限值的索引
	u16 *ParAdd; //参数变量地址
}PARA_ATTR_NEW;

extern u16 pr[PRMAX];
extern const PARA_ATTR_NEW  SysParaAttrCharNEW[];

#endif // __SCITOOPRT_H__

