#include "SciToOprt.h"



u16 pr[PRMAX];

#ifdef MOTORFLASH
const PARA_ATTR_NEW  SysParaAttrCharNEW[120]=
{
	//P0
	{{0,1,0, 0,0,0},0,     55,   0,    0, 1000,&pr[(GROUP0+0)]},		//RELS			0  	00-00	Release
	{{0,1,0, 2,0,0},0,      0,   0,    0, 1000,&pr[(GROUP0+1)]},		//RELS			1	  00-01	Release
	{{0,1,0, 4,0,0},0,      0,   0,    0, 1000,&pr[(GROUP0+2)]},		//RELS			2	  00-02	Release
	{{0,1,0, 6,0,0},0,      0,   0,    0, 1000,&pr[(GROUP0+3)]},		//RELS			3	  00-03	Release
	{{0,1,0, 8,0,0},0,      0,   0,    0, 1000,&pr[(GROUP0+4)]},		//RELS			4	  00-04	Release
	{{0,1,0,10,0,0},0,      0,   0,    0, 1000,&pr[(GROUP0+5)]},	  //RELS			5	  00-05	Release
	{{0,1,0,12,0,0},0,      0,   0,    0, 1000,&pr[(GROUP0+6)]},	  //RELS			6	  00-06	Release
	//P1
	{{0,1,0,14,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+0)]},		//RELS			7	  01-00	Release
	{{0,1,0,16,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+1)]},		//RELS			8	  01-01	Release
	{{0,1,0,18,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+2)]},		//RELS			9	  01-02	Release
	{{0,1,0,20,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+3)]},		//RELS			10	01-03	Release
	{{0,1,0,22,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+4)]},		//RELS			11	01-04	Release
	{{0,1,0,24,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+5)]},	  //RELS			12	01-05	Release
	{{0,1,0,26,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+6)]},	  //RELS			13	01-06	Release
	{{0,1,0,28,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+7)]},		//RELS			14	01-07	Release
	{{0,1,0,30,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+8)]},		//RELS			15	01-08	Release
	{{0,1,0,32,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+9)]},		//RELS			16	01-09	Release
	{{0,1,0,34,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+10)]},		//RELS			17	01-10	Release
	{{0,1,0,36,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+11)]},		//RELS			18	01-11	Release
	{{0,1,0,38,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+12)]},		//RELS			19	01-12	Release
	{{0,1,0,40,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+13)]},		//RELS			20	01-13	Release
	{{0,1,0,42,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+14)]},		//RELS			21	01-14	Release
	{{0,1,0,44,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+15)]},	  //RELS			22	01-15	Release
	{{0,1,0,46,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+16)]},	  //RELS			23	01-16	Release
	{{0,1,0,48,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+17)]},		//RELS			24	01-17	Release
	{{0,1,0,50,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+18)]},		//RELS			25	01-18	Release
	{{0,1,0,52,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+19)]},		//RELS			26	01-19	Release
	{{0,1,0,54,0,0},0,      0,   0,    0, 1000,&pr[(GROUP1+20)]},		//RELS			27	01-20	Release
	//P2
	{{0,1,0,56,0,0},0,     20,   0,    1, 1000,&pr[(GROUP2+0)]},		//AUTOCUR		  28	02-00	Auto tuning Current(0.1A)
	{{0,1,0,58,0,0},0,  15000,   0,    1, 1000,&pr[(GROUP2+1)]},		//SPDMAX	    29	02-01	SPDMAX(0.1Hz)
	{{0,1,0,60,0,0},0,   3000,   0,    1, 9999,&pr[(GROUP2+2)]},		//RATED_RPM		30	02-02	Motor Rated RPM(10RPM)
	{{0,1,0,62,0,0},0,      6,   0,    2, 1000,&pr[(GROUP2+3)]},		//PM_POLES		31	02-03	Motor pole No（极数）
	{{0,1,0,64,0,0},0,   5000,   0,    1, 1000,&pr[(GROUP2+4)]},		//RATED_POW   32  02-04	RATED_POW
	{{0,1,0,66,0,0},0,      1,   0,    0, 2000,&pr[(GROUP2+5)]},	  //SPDLOOPEN	  33  02-05	SPDLOOPEN	 0:POWER 1:SPD
	{{0,1,0,68,0,0},0,   1000,   0,    1, 6000,&pr[(GROUP2+6)]},		//POW_LIM		  34  02-06	POW_LIM
	{{0,1,0,70,0,0},0,      0,   0,    1, 1000,&pr[(GROUP2+7)]},		//POWLIM_EN	  35  02-07	POWLIM_EN
	{{0,1,0,72,0,0},0,    170,   0,    1, 1000,&pr[(GROUP2+8)]},		//SPD_SW		  36  02-08	SPD_SW Hz
	{{0,1,0,74,0,0},0,    500,   0,    1, 1000,&pr[(GROUP2+9)]},		//FLW_ID_GAIN	37  02-09	HFVI PLL Cf
	{{0,1,0,76,0,0},0,      0,   0,    0, 1000,&pr[(GROUP2+10)]},		//SUN_EN  	  38  02-10	SUN_EN
	{{0,1,0,78,0,0},0,     30,   0,    0, 1000,&pr[(GROUP2+11)]},		//FLW_ID_INTE	39  02-11	FLW_ID_INTE     Q10
	{{0,1,0,80,0,0},0,      0,   0,    0, 1000,&pr[(GROUP2+12)]},		//SLR_K     	40  02-12	0:Common ASR  2:New ASR
	{{0,1,0,82,0,0},0,    183,   0,    0, 1000,&pr[(GROUP2+13)]},		//HFVI      	41  02-13	HFVI Degree Com
	{{0,1,0,84,0,0},0,      0,   0,    0, 1000,&pr[(GROUP2+14)]},		//TFED_LPF    42  02-14	New ASR Tor Fed LPF Unit:0.01s
	{{0,1,0,86,0,0},0,      1,   0,    0, 1000,&pr[(GROUP2+15)]},	  //SPDHALL180 	43	02-15	SPDHALL180  0:60  1:180
	{{0,1,0,88,0,0},0,    828,   0,    0, 1000,&pr[(GROUP2+16)]},	  //IniUMag	  	44	02-16	IniUMag
	{{0,1,0,90,0,0},0,     33,   0,    0, 1000,&pr[(GROUP2+17)]},		//I_RATE		  45	02-17	Motor Rated Current(0.1A，有效值)
	{{0,1,0,92,0,0},0,    127,   0,    0,65535,&pr[(GROUP2+18)]},		//SW_VER		  46	02-18	Software Version
	{{0,1,0,94,0,0},0,   1750,   0,    0, 6000,&pr[(GROUP2+19)]},		//VELCMD		  47	02-19	Velocity command(0.1Hz)	 1900 2800
	{{0,1,0,96,0,0},0,      7,   0,    0,  256,&pr[(GROUP2+20)]},		//CTRLM			  48	02-20	Control Methods 1:VF 2：CS 3：PG 7:IF 10:DC
	{{0,1,0,98,0,0},0,   1000,   0,  100, 3000,&pr[(GROUP2+21)]},		//PWMFRE		  49	02-21	PWM Carry Frequency(10Hz)
	{{0,1,0,100,0,0},0,     1,   0,    0,    1,&pr[(GROUP2+22)]},		//VELDIR		  50	02-22	Velocity direction
	{{0,1,0,102,0,0},0,  8000,   0,    0,65535,&pr[(GROUP2+23)]},		//IsREF     	51	02-23	CS current reference(Pu, Q15) 6000
	//P3
	{{0,1,0,104,0,0},0, 10000,   0,    1,65535,&pr[(GROUP3+0)]},		//FB			      52	03-00	Freq Base value(0.1Hz)
	{{0,1,0,106,0,0},0,     2,   0,    0, 1000,&pr[(GROUP3+1)]},		//IniMode 		  53	03-01	IniMode
	{{0,1,0,108,0,0},0,   200,   0,    0, 1000,&pr[(GROUP3+2)]},		//MAG_VOLT		  54	03-02	MAG_VOLT(0.1V)
	{{0,1,0,110,0,0},0,     8,   0,    0, 1000,&pr[(GROUP3+3)]},		//MAG_TIME		  55	03-03	MAG_TIME(TBC)
	{{0,1,0,112,0,0},0,    10,   0,    0, 1000,&pr[(GROUP3+4)]},		//IFCurACC		  56	03-04	IFCurACC
	{{0,1,0,114,0,0},0, 12000,   0,    0, 2000,&pr[(GROUP3+5)]},	  //IQ_LIM		    57	03-05	IQ_LIM
	{{0,1,0,116,0,0},0,  1000,   0,    1,65535,&pr[(GROUP3+6)]},	  //ATT_DURATION	58	03-06	Auto tuning each section duration.(ms)
  {{0,1,0,118,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+7)]},    //SPD_BST		    59	03-07	SPD_BST
	{{0,1,0,120,0,0},0,   600,   0,    0,10000,&pr[(GROUP3+8)]},		//IFFOCSPD		  60	03-08	IFFOCSPD(0.1Hz)	 40Hz Sensorless
	{{0,1,0,122,0,0},0,    20,   0,    0, 1000,&pr[(GROUP3+9)]},		//IFCURAUTOSPD	61	03-09 IFCURAUTOSPD(0.1Hz)
	{{0,1,0,124,0,0},0,     1,   0,    0, 1000,&pr[(GROUP3+10)]},		//THEHALL180	  62	03-10 THEHALL180  0:60  1:180
	{{0,1,0,126,0,0},0,  3200,   0,    0, 1000,&pr[(GROUP3+11)]},		//VELCMDUPLIM	  63	03-11	VELCMDUPLIM
	{{0,1,0,128,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+12)]},		//VELCMDDOWNLIM	64	03-12	VELCMDDOWNLIM
	{{0,1,0,130,0,0},0,   100,   0,    1, 1000,&pr[(GROUP3+13)]},		//CLR_D_KP		  65	03-13 Cur Loop PI Cf
	{{0,1,0,132,0,0},0,   100,   0,    1, 1000,&pr[(GROUP3+14)]},		//CLR_D_KI		  66	03-14 Cur Loop PI Cf
	{{0,1,0,134,0,0},0,    25,   0,    0, 1000,&pr[(GROUP3+15)]},	  //RunTimeLimCnt	67	03-15	RunTimeLimCnt
	{{0,1,0,136,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+16)]},	  //RunTimLimEn		68	03-16 RunTimLimEn
	{{0,1,0,138,0,0},0,   270,   0,    0, 1000,&pr[(GROUP3+17)]},		//MOTORNUM			69	03-17 MOTORNUM
	{{0,1,0,140,0,0},0,  1300,   0,    1,32767,&pr[(GROUP3+18)]},		//NS_UCMD		    70	03-18	NS U_CMD Q15
	{{0,1,0,142,0,0},0,   400,   0,    1,32767,&pr[(GROUP3+19)]},		//J_MOTOR		    71	03-19	Inertia of motor 0.000001kg.m^2
	{{0,1,0,144,0,0},0,     0,   0,    0,10000,&pr[(GROUP3+20)]},		//ID_REF        72	03-20 ID_REF 1000
	{{0,1,0,146,0,0},0,   100,   0,    0, 1000,&pr[(GROUP3+21)]},	  //TE_LIM        73	03-21 TE_LIM
	{{0,1,0,148,0,0},0,  5000,   0,    1,65535,&pr[(GROUP3+22)]},		//VF_FBASE		  74	03-22	VF Motor Rated Freq(0.01Hz)
	{{0,1,0,150,0,0},0,  1500,   0,    0, 3000,&pr[(GROUP3+23)]},		//VMAX			    75	03-23	VF Max Output Voltage(0.1V)
	{{0,1,0,152,0,0},0,   200,   0,    0, 3000,&pr[(GROUP3+24)]},		//VF_FMID1		  76	03-24	VF Mid Output Freq.1(0.01Hz)
	{{0,1,0,154,0,0},0,   200,   0,    0, 3000,&pr[(GROUP3+25)]},		//VF_VMID1		  77	03-25	VF Mid Output Voltage 1(0.1V)
	{{0,1,0,156,0,0},0,   100,   0,    0, 3000,&pr[(GROUP3+26)]},	  //VF_FMID2		  78	03-26	VF Mid Output Freq.2(0.01Hz)
	{{0,1,0,158,0,0},0,   100,   0,    0, 3000,&pr[(GROUP3+27)]},	  //VF_VMID2		  79	03-27	VF Mid Output Voltage 2(0.1V)
	{{0,1,0,160,0,0},0,     0,   0,    0, 3000,&pr[(GROUP3+28)]},		//FMIN			    80	03-28	VF Min Output Freq.(0.01Hz)
	{{0,1,0,162,0,0},0,    45,   0,    0, 3000,&pr[(GROUP3+29)]},		//VMIN			    81	03-29	VF Min Output Voltage(0.1V)
	{{0,1,0,164,0,0},0,    10,   0,    0, 3000,&pr[(GROUP3+30)]},		//ACCTIME		    82	03-30	Acceleration time(0.1s)
	{{0,1,0,166,0,0},0,    10,   0,    0, 3000,&pr[(GROUP3+31)]},		//DECTIME		    83	03-31	Deceleration time(0.1s)
	{{0,1,0,168,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+32)]},	  //RELS    		  84	03-32 Release
	{{0,1,0,170,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+33)]},		//RELS      	  85	03-33 Release
	{{0,1,0,172,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+34)]},		//RELS       	  86	03-34 Release
	{{0,1,0,174,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+35)]},		//RELS			    87	03-35 Release
	{{0,1,0,176,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+36)]},		//RELS			    88	03-36 Release
	{{0,1,0,178,0,0},0, 24000,   0,    1,65535,&pr[(GROUP3+37)]},		//DTCRATIO 		  89	03-37	DTC Linear Ratio Q16
	{{0,1,0,180,0,0},0,  9200,   0,    1,65535,&pr[(GROUP3+38)]},		//DTCDu			    90	03-38	DTC Voltage Drop Cf Q18
	{{0,1,0,182,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+39)]},		//RELS			    91	03-39	Release
	{{0,1,0,184,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+40)]},		//RELS			    92	03-40	Release
	{{0,1,0,186,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+41)]},	  //RELS			    93	03-41	Release
	{{0,1,0,188,0,0},0,     0,   0,    0, 1000,&pr[(GROUP3+42)]},	  //RELS			    94	03-42	Release
	{{0,1,0,190,0,0},0,    50,   0,    1, 1000,&pr[(GROUP3+43)]},		//SLR_SHAPE	  	95	03-43	Width of velocity loop mid-freq.Unit 0.1
	//P4~~~~
	{{0,1,0,192,0,0},0,     1,   0,    0,    1,&pr[(GROUP4+0)]},		//TFED_EN       96  04-00   New ASR Tor Feed En
	{{0,1,0,194,0,0},0,   300,   0,    0, 1000,&pr[(GROUP4+1)]},		//OB_BW         97  04-01   Observer BW //Unit:0.1Hz
	{{0,1,0,196,0,0},0,   100,   0,    0,10000,&pr[(GROUP4+2)]},		//SLR_ANTINOISE 98  04-02   New ASR Unit:0.1%   k
	{{0,1,0,198,0,0},0,     0,   0,    0,10000,&pr[(GROUP4+3)]},		//RELS     		  99  04-03   RELS
	{{0,1,0,200,0,0},0,   186,   0,    0, 1000,&pr[(GROUP4+4)]},		//INEG_DEL_SAL  100 04-04   SAL Delay Angl
	{{0,1,0,202,0,0},0,  6000,   0,    0,32767,&pr[(GROUP4+5)]},		//PR_SAL_MAG    101 04-05   SAL Inj Vol //Q15
	{{0,1,0,204,0,0},0,  1600,   0,    1,32767,&pr[(GROUP4+6)]},		//RsPM          102 04-06   0.001om; 1315 3000  1600
	{{0,1,0,206,0,0},0,   410,   0,    1,32767,&pr[(GROUP4+7)]},		//PMLD   		    103 04-07   0.01mH;   600  810   410
	{{0,1,0,208,0,0},0,   480,   0,    1,32767,&pr[(GROUP4+8)]},		//PMLQ   		    104 04-08   0.01mH;   600  880   480
	{{0,1,0,210,0,0},0,  1500,   0,    0, 1000,&pr[(GROUP4+9)]},		//IF_IQ_GAIN    105 04-09   IF_IQ_GAIN
	{{0,1,0,212,0,0},0,   100,   0,    0, 1000,&pr[(GROUP4+10)]},		//IF_IQ_INTE   	106 04-10   IF_IQ_INTE
	{{0,1,0,214,0,0},0,  1120,   0,    1,32767,&pr[(GROUP4+11)]},	  //BEMF          107 04-11   Motor Rated Vol //相电压幅值Unit:0.1V
	{{0,1,0,216,0,0},0,    15,   0,    0,  200,&pr[(GROUP4+12)]},	  //SLR_HF_RSPS   108 04-12   New ASR Cf //Unit:%
	{{0,1,0,218,0,0},0,     0,   0,    0, 1000,&pr[(GROUP4+13)]},		//JL_RATIO      109 04-13   New ASR Load J Ratio Cf //Unit:0.1
	{{0,1,0,220,0,0},0,   100,   0,    0, 1000,&pr[(GROUP4+14)]},		//SLR_ANTIOVSHT 110 04-14   New ASR Vel Cmd LPF Cf //Unit:0.1%  TFED_LPF=1,SLR_ANTIOVSHT=100:10%超调  >300:无超调:
	{{0,1,0,222,0,0},0,  1902,   0,    0, 1000,&pr[(GROUP4+15)]},		//SWVER_YM			111 04-15   SWVER_YM
	{{0,1,0,224,0,0},0,  2700,   0,    0, 1000,&pr[(GROUP4+16)]},		//SWVER_DV			112 04-16   SWVER_DV
	{{0,1,0,226,0,0},0,    60,   0,    1, 1000,&pr[(GROUP4+17)]},		//SLR_KP		    113 04-17   Old ASR Kp %
	{{0,1,0,228,0,0},0,    10,   0,    1, 1000,&pr[(GROUP4+18)]},		//SLR_KI		    114 04-18   Old ASR Ki %
	{{0,1,0,230,0,0},0,     2,   0,    0, 1000,&pr[(GROUP4+19)]},		//SLR_ERR_LPF	  115 04-19   Old ASR Error LPF ms
	{{0,1,0,232,0,0},0,    50,   0,    0, 1000,&pr[(GROUP4+20)]},		//BRAK_TIM 		  116 04-20   BRAK_TIM(0.001s)
	{{0,1,0,234,0,0},0,     1,   0,    0,    1,&pr[(GROUP4+21)]},	  //DTC_EN  		  117 04-21   DBC Enable
	{{0,1,0,236,0,0},0,     1,   0,    0, 1000,&pr[(GROUP4+22)]},	  //PWM_MODE  	  118 04-22   Ini Degree PWM_MODE
	{{0,1,0,238,0,0},0,     0,   0,    0, 1000,&pr[(GROUP4+23)]},		//RELS  		    119 04-23   Release
};


#endif









