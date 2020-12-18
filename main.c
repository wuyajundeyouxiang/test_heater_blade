/******************************************************************************
* Copyright (C) 2018, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.B
*/ 
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2018-05-08  1.0  Lux First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "bt.h"
#include "gpio.h"
#include "hc32f005.h"
#include "sysctrl.h"
#include "stdint.h"

#include "adt.h"
#include "flash.h"
#include "mycommon.h"
#include "adc.h"
#include "bgr.h"
#include "uart.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 
#define LED1PORT	                        GpioPort2       //GpioPort1 //GpioPort0
#define LED1PIN		                        GpioPin5        //GpioPin4	//GpioPin1
#define LED2PORT	                        GpioPort2       //GpioPort2 //GpioPort0//
#define LED2PIN		                        GpioPin4        //GpioPin4 //GpioPin2//
#define KEYPORT		                        GpioPort3
#define KEYPIN		                        GpioPin6
#define MOTOR_PORT	                        GpioPort3
#define MOTOR_PIN	                        GpioPin5
#define USB_PORT	                        GpioPort1
#define USB_PIN		                        GpioPin5
#define CHR_PORT	                        GpioPort1
#define CHR_PIN		                        GpioPin4
#define T_CHE_PORT                          GpioPort2
#define T_CHE_PIN                           GpioPin6
#define PWM_PORT                            GpioPort3       //GpioPort3,GpioPin4,GpioAf5
#define PWM_PIN                             GpioPin2                        
#define PWM_AFX                             GpioAf3
#define PWM_TIMX                            M0P_ADTIM6      //PWM_TIMX M0P_ADTIM4
#define PWM_CHX                             GCMBR
#define PWM_CMPX                            AdtCompareB
#define PWM_AdtCHx                          AdtCHxB         //GCMAR
#define ADC_CHx                             AdcExInputCH4
#define KEY_STATE_0                         0           
#define KEY_STATE_1                         1
#define KEY_STATE_2                         2
#define KEY_STATE_3                         3
#define KEY_STATE_4                         4
#define KEY_STATE_5                         5
#define KEY_STATE_6                         6
#define SINGLE_KEY_TIME                     3       	    //  SINGLE_KEY_TIME*10MS = 30MS  
#define KEY_INTERVAL                        40      	    //  KEY_INTERVAL*10MS    = 400MS 
#define LONG_KEY_TIME                       100      	    //  LONG_KEY_TIME*10MS   = 600mS 
#define TIM_1S                              1000
#define VOLTAGE_L	                        3300
#define adcLen	                            4

static uint16_t         index           = 0;
static uint16_t         adcResult_Temp[adcLen]={0}; 
static uint32_t         adcResult_Sum   = 0;
static uint16_t         adcAve          = 0;
static uint16_t         bFullFlag       = 0;

unsigned char 			g1MS_Cnt  		= 0;
unsigned char 			g100MS_Cnt		= 0;
unsigned char 			g200MS_Cnt		= 0;
unsigned char 			g500MS_Cnt		= 0;
unsigned char 			g1000MS_Cnt		= 0;
volatile unsigned char 	gFlag10ms		= 0;
volatile unsigned char 	gFlag100ms		= 0;
volatile unsigned char 	gFlag200ms		= 0;
volatile unsigned char 	gFlag500ms		= 0;
volatile unsigned char 	gFlag1000ms		= 0;
volatile unsigned long  gTime0Count  	= 0;
unsigned long  			gstartTime	  	= 0;
MOTORFLAG gMotor;
static volatile uint32_t u32BtTestFlag = 0;
static volatile uint32_t u32Cnt = 0;

void App_LedInit(void);
void iniPwm(void);
void iniTemperature(void);
void delay(uint32_t u32Cnt);
void _UartBaudCfg(void);
void Led1On(void);
void Led1Off(void);
void Led2On(void);
void Led2Off(void);
void sendChar(char);
void sendStr(char* pStr);
void sendIntStr(int num);
void getBatVol(void);
void blink_LedALL(int num);
void testLed(void);
void testPwm(void);

void Timer_Init(void)
{
	stc_bt_cfg_t   	  stcCfg;
    en_result_t       enResult = Error;
    uint16_t          u16ArrData = 0xFE88;
    uint16_t          u16InitCntData = 0xFE88;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv64;
    stcCfg.enTog   = BtTogDisable;
    stcCfg.enCT    = BtTimer;
    stcCfg.enMD    = BtMode2;
    
    if (Ok != Bt_Init(TIM0, &stcCfg))
    {
        enResult = Error;
    }
    
    Bt_ClearIntFlag(TIM0);
    Bt_EnableIrq(TIM0);
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);
    
    Bt_ARRSet(TIM0, u16ArrData);
    Bt_Cnt16Set(TIM0, u16InitCntData);
    Bt_Run(TIM0);
}

void SetSysClock24M(void)
{		
	M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
	M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 7;
													
	M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C08 ) ); //4M
	M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C06 ) ); //8M
	M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C04 ) ); //16M
	M0P_SYSCTRL->RCH_CR = *((uint16_t *)( 0X00100C00 ) ); //24M
	M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
	M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 0;
}

void App_AdvTimerInit(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    en_adt_compare_t          enAdtCompareA;
    en_adt_compare_t          enAdtCompareB;
	en_adt_compare_t          enAdtCompareX;
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIMACfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIMBCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIMXCfg;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIMACfg);
    DDL_ZERO_STRUCT(stcAdtTIMBCfg);
    DDL_ZERO_STRUCT(stcAdtTIMXCfg);    

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);
    
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntDown;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div1024;
    
    Adt_Init(PWM_TIMX, &stcAdtBaseCntCfg);    
    Adt_SetPeriod(PWM_TIMX, u16Period);    
	enAdtCompareX = PWM_CMPX;
	Adt_SetCompareValue(PWM_TIMX, enAdtCompareX, u16CHA_PWMDuty);
    
    stcAdtTIMXCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIMXCfg.bOutEn = TRUE;
    stcAdtTIMXCfg.enPerc = AdtCHxPeriodLow;
    stcAdtTIMXCfg.enCmpc = AdtCHxCompareInv;
    stcAdtTIMXCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIMXCfg.enStaOut = AdtCHxPortOutLow;
    stcAdtTIMXCfg.enStpOut = AdtCHxPortOutLow;
    Adt_CHxXPortCfg(PWM_TIMX, PWM_AdtCHx, &stcAdtTIMXCfg);
}

void App_AdvTimerPortInit(void)
{
    stc_gpio_cfg_t         stcTIM4Port;    
    DDL_ZERO_STRUCT(stcTIM4Port);    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    
    stcTIM4Port.enDir  = GpioDirOut;
    Gpio_Init(PWM_PORT, PWM_PIN, &stcTIM4Port);
    Gpio_SetAfMode(PWM_PORT,PWM_PIN,PWM_AFX);
}

void LED_Init(void)
{
	stc_gpio_cfg_t stcGpioCfg;
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    
	stcGpioCfg.enDir = GpioDirOut;
	stcGpioCfg.enPu = GpioPuDisable;
	stcGpioCfg.enPd = GpioPdDisable;    
	Gpio_Init(LED1PORT, LED1PIN, &stcGpioCfg);	
	Gpio_Init(LED2PORT, LED2PIN, &stcGpioCfg);
	Led1On();
	Led2On();
	
	//delay1ms(1000);
	delay(1000);
	Led1Off();
	Led2Off();
}

void Key_Init(void)
{
	stc_gpio_cfg_t stcGpioCfg;    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu  = GpioPuEnable;
    stcGpioCfg.enPd  = GpioPdDisable;
    stcGpioCfg.enOD  = GpioOdDisable;
    Gpio_Init(KEYPORT, KEYPIN, &stcGpioCfg); 
}

unsigned char key_driver(void) 
{     
    static unsigned char key_state = 0;
    static unsigned int  key_time = 0;
    unsigned char key_press, key_return; 

    key_return = N_KEY;

    key_press = Gpio_GetInputIO(KEYPORT, KEYPIN);

    switch (key_state)     
    {       
        case KEY_STATE_0:
            if (!key_press)
            {
                key_time = 0;
                key_state = KEY_STATE_1;
            }        
            break;
        case KEY_STATE_1:
            if (!key_press)                     
            {
                key_time++;
                if(key_time>=SINGLE_KEY_TIME)
                {
                    key_state = KEY_STATE_2;
                }
            }         
            else key_state = KEY_STATE_0;
            break;
        case KEY_STATE_2:
            if(key_press)
            { 
                 key_return = S_KEY;
                 key_state = KEY_STATE_0;
            } 
            else
            {
                key_time++;
                if(key_time >= LONG_KEY_TIME)
                {
                    key_return = L_KEY;
                    key_state = KEY_STATE_3;                    
                }
            }
            break;
      case KEY_STATE_3:                         
          if (key_press) 
          {
              key_state = KEY_STATE_0;          
          }        
          break;
        default:                               
            key_state = KEY_STATE_0;
            break;
    }
    return  key_return;                         
} 

unsigned char key_read(void)                            
{ 
    static unsigned char key_state1=0, key_time1=0;
    unsigned char key_return,key_temp;
    key_return = N_KEY;
    key_temp = key_driver();

    switch(key_state1) 
    {         
        case KEY_STATE_0:
            if (key_temp == S_KEY )
            { 
                 key_time1 = 0;
                 key_state1 = KEY_STATE_1;
            }             
            else
            {
                 key_return = key_temp;
            }
            break;
        case KEY_STATE_1:
            if (key_temp == S_KEY)
            {                 
                 key_time1 = 0;                 
                 key_state1 = KEY_STATE_2;
            } 
            else
            {
                key_time1++;
                if(key_time1 >= KEY_INTERVAL)
                 { 
                      key_return = S_KEY;					  
                      key_state1 = KEY_STATE_0;
                 }              
             }              
             break;
         case KEY_STATE_2:
            if (key_temp == S_KEY)
            {                 
                 key_time1 = 0;                 
                 key_state1 = KEY_STATE_3;
            } 
            else
            {
                key_time1++;
                if(key_time1 >= KEY_INTERVAL)
                 { 
                      key_return = D_KEY;
                      key_state1 = KEY_STATE_0;
                 }              
             }              
             break;
         case KEY_STATE_3:
            if (key_temp == S_KEY)
            {                 
                 key_time1 = 0;                 
                 key_state1 = KEY_STATE_4;
            } 
            else
            {
                key_time1++;
                if(key_time1 >= KEY_INTERVAL)
                 { 
                      key_return = T_KEY;
                      key_state1 = KEY_STATE_0;
                 }              
             }              
             break;
         case KEY_STATE_4:
            if (key_temp == S_KEY)
            {                 
                 key_time1 = 0;                 
                 key_state1 = KEY_STATE_5;
            } 
            else
            {
                key_time1++;
                if(key_time1 >= KEY_INTERVAL)
                 { 
                      key_return = Q_KEY;
                      key_state1 = KEY_STATE_0;
                 }              
             }              
             break;
         case KEY_STATE_5:
            if (key_temp == S_KEY)
            {                 
                 key_return = Six_KEY;                 
                 key_state1 = KEY_STATE_0;
            } 
            else
            {
                key_time1++;
                if(key_time1 >= KEY_INTERVAL)
                 { 
                      key_return = F_KEY;
                      key_state1 = KEY_STATE_0;
                 }              
             }              
             break;
        default:
            key_state1 = KEY_STATE_0;
            break;
    }
    return key_return;
}

static uint16_t u16AdcResult;
static uint16_t u16AdcResult_Temp;
stc_adc_cfg_t      stcAdcCfg;
stc_adc_norm_cfg_t stcAdcNormCfg; 

void iniBatVol(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    Adc_Enable();
    Bgr_BgrEnable();   
    stcAdcCfg.enAdcOpMode = AdcNormalMode;
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv8;
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime4Clk;
    stcAdcCfg.enAdcRefVolSel = RefVolSelInBgr1p5;
    stcAdcCfg.bAdcInBufEn = TRUE;
    stcAdcCfg.u32AdcRegHighThd = 0u;
    stcAdcCfg.u32AdcRegLowThd = 0u;
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

	stcAdcNormCfg.enAdcNormModeCh = AdcAVccDiV3Input;
    stcAdcNormCfg.bAdcResultAccEn = FALSE;
    Adc_ConfigNormMode(&stcAdcCfg, &stcAdcNormCfg);
	
}

uint8_t u8RxData[2] = {0x55,0x00};
void Uart0_IRQHandler(void)
{
    if(TRUE == Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);

        u8RxData[1] = Uart_ReceiveData(M0P_UART0);
        //u8RxFlg = 1;
    }
}

void iniUart(void)
{
	stc_gpio_cfg_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    ///<TX
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPort0, GpioPin1, &stcGpioCfg);	

    ///<RX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPort0, GpioPin2, &stcGpioCfg);	
	Gpio_SetAfMode(GpioPort0, GpioPin1, GpioAf3);
    Gpio_SetAfMode(GpioPort0, GpioPin2, GpioAf3);

	stc_uart_cfg_t  stcCfg;
    _UartBaudCfg();
    stcCfg.enRunMode = UartMode1;
    Uart_Init(M0P_UART1, &stcCfg);
   
    Uart_EnableIrq(M0P_UART1, UartRxIrq);
    Uart_ClrStatus(M0P_UART1, UartRC);
    EnableNvic(UART1_IRQn, IrqLevel3, TRUE);
}

void Motor_Init(void)
{
	gMotor.flag= 1;
	gMotor.MaxCnt= 2;
	gMotor.cnt=0;

	stc_gpio_cfg_t stcGpioCfg;
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    
	stcGpioCfg.enDir = GpioDirOut;
	stcGpioCfg.enPu = GpioPuDisable;
	stcGpioCfg.enPd = GpioPdEnable;    
	Gpio_Init(MOTOR_PORT, MOTOR_PIN, &stcGpioCfg);
	Gpio_WriteOutputIO(MOTOR_PORT, MOTOR_PIN, FALSE);
}

void openMotor(void)
{
	Gpio_WriteOutputIO(MOTOR_PORT, MOTOR_PIN, TRUE);
}

void closeMotor(void)
{
	Gpio_WriteOutputIO(MOTOR_PORT, MOTOR_PIN, FALSE);
}

void iniCharge(void)
{
	stc_gpio_cfg_t stcGpioCfg;    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu  = GpioPuEnable;
    stcGpioCfg.enPd  = GpioPdDisable;
    stcGpioCfg.enOD  = GpioOdDisable;
    Gpio_Init(USB_PORT, USB_PIN, &stcGpioCfg);
	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu  = GpioPuEnable;
    stcGpioCfg.enPd  = GpioPdDisable;
    stcGpioCfg.enOD  = GpioOdDisable;
    Gpio_Init(CHR_PORT, CHR_PIN, &stcGpioCfg);
}

void scanInputPower(void)
{
	static int cnt1 = 0;
	static int cnt2 = 0;
	static int cnt3 = 0;
	static int cnt4 = 0;
	if(Gpio_GetInputIO(USB_PORT, USB_PIN))
	{			
		cnt1=0;
		cnt2++;
		if(cnt2>=2)
		{
			cnt2 = 0;
			sendStr("TIP:CHG_UNPLUG");
			sendMsg(CHG_UNPLUG);
		}
	}
	else
	{
		cnt2=0;
		cnt1++;
		if(cnt1>=2)
		{
			cnt1 = 0;
			sendStr("TIP:CHG_PLUG");
			sendMsg(CHG_PLUG);

			if(Gpio_GetInputIO(CHR_PORT, CHR_PIN))
			{			
				cnt3=0;
				cnt4++;
				if(cnt4>=2)
				{
					cnt4 = 0;
					sendStr("TIP:CHG_NOFULL");
					sendMsg(CHG_NOFULL);
				}
			}
			else
			{
				cnt4=0;
				cnt3++;
				if(cnt3>=2)
				{
					cnt3 = 0;
					sendStr("TIP:CHG_FULL");
					sendMsg(CHG_FULL);
				}
			}
		}
	}
}

void App_Init(void)
{
	SetSysClock24M();
	Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);
	SystemCoreClockUpdate();

	Timer_Init();
	LED_Init();		
	Key_Init();
	Motor_Init();
	
	iniPwm();
	iniTemperature();
	iniBatVol();
	iniCharge();

	iniUart();
	state = standby;
}

void changePwmDuty(uint16_t u16CHX_PWMDuty)
{
	PWM_TIMX->PWM_CHX = u16CHX_PWMDuty;
}

void delay(uint32_t u32Cnt)
{
	unsigned long oldtime = gTime0Count;
	while(gTime0Count - oldtime < u32Cnt);
}

void Tim0_IRQHandler(void)
{
    if (TRUE == Bt_GetIntFlag(TIM0))
    {
        Bt_ClearIntFlag(TIM0);
			
		++g1MS_Cnt;
		++gTime0Count;
		if(g1MS_Cnt >= 10)
		{						
			g1MS_Cnt    = 0;
			gFlag10ms   = 1;
			++g100MS_Cnt;
			if(g100MS_Cnt >= 10)
			{	
				++g200MS_Cnt;
				if(g200MS_Cnt>=2)
				{
					g200MS_Cnt=0;
					gFlag200ms=1;
				}
				++g500MS_Cnt;
				if(g500MS_Cnt>=5)
				{
					g500MS_Cnt=0;
					gFlag500ms=1;
				}
				g100MS_Cnt = 0;
				gFlag100ms = 1;
				++g1000MS_Cnt;			
				if(g1000MS_Cnt>=10)
				{
					g1000MS_Cnt = 0;
					gFlag1000ms = 1;
					//testLed();
				}
			}
		}
    }
}

void Uart1_IRQHandler(void)
{
    if(TRUE == Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);

        u8RxData[1] = Uart_ReceiveData(M0P_UART1);
//        u8RxFlg = 1;
    }
}


static int flag = 0;
void testLed(void)
{		
	if(flag==0)
	{
		Led1On();
		Led2On();
		flag = 1;
	}
	else
	{
		Led1Off();
		Led2Off();
		flag = 0;
	}		
}

void led01_test(void)
{		
	if(flag==0)
	{
		Led1Off();
		flag = 1;
	}
	else
	{
		Led1On();
		flag = 0;
	}		
}


void Led1On(void)
{
	Gpio_WriteOutputIO(LED1PORT, LED1PIN, FALSE);
}

void Led2On(void)
{
	Gpio_WriteOutputIO(LED2PORT, LED2PIN, FALSE);
}

void Led1Off(void)
{
	Gpio_WriteOutputIO(LED1PORT, LED1PIN, TRUE);
}

void Led2Off(void)
{
	Gpio_WriteOutputIO(LED2PORT, LED2PIN, TRUE);
}

void LedAllOff(void)
{
	Led1Off();
	Led2Off();
}

void iniTemperature(void)
{
	stc_gpio_cfg_t stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;    

    Gpio_Init(T_CHE_PORT, T_CHE_PIN, &stcGpioCfg);
	Gpio_WriteOutputIO(T_CHE_PORT, T_CHE_PIN, TRUE);//       GpioPort3   GpioPin5

    //////////////////
	Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR ??????
    Adc_Enable();
    Bgr_BgrEnable();   
    stcAdcCfg.enAdcOpMode = AdcNormalMode;
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv8;
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime4Clk;
    stcAdcCfg.enAdcRefVolSel = RefVolSelInBgr1p5;
    stcAdcCfg.bAdcInBufEn = TRUE;
    stcAdcCfg.u32AdcRegHighThd = 0u;
    stcAdcCfg.u32AdcRegLowThd = 0u;
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

	stcAdcNormCfg.enAdcNormModeCh = ADC_CHx;//AdcExInputCH4;//AdcExInputCH2;
    stcAdcNormCfg.bAdcResultAccEn = FALSE;
    Adc_ConfigNormMode(&stcAdcCfg, &stcAdcNormCfg);
    ///////////////////
}

void openTemperature(void)
{
	Gpio_WriteOutputIO(T_CHE_PORT, T_CHE_PIN, FALSE);
}

void closeTemperature(void)
{
	Gpio_WriteOutputIO(T_CHE_PORT, T_CHE_PIN, TRUE);
}

void checkTemperature(void)
{	
	getBatVol();
	
	openTemperature();
	stcAdcNormCfg.enAdcNormModeCh = ADC_CHx;
    stcAdcNormCfg.bAdcResultAccEn = FALSE;
    Adc_ConfigNormMode(&stcAdcCfg, &stcAdcNormCfg);
	
	delay1ms(1);
	Adc_Start();
    while(FALSE != Adc_PollBusyState());
    Adc_GetResult(&u16AdcResult_Temp);	
	closeTemperature();
	
	adcResult_Temp[index++] = u16AdcResult_Temp;
	if(index>=adcLen)
	{
		index=0;
		bFullFlag=1;
	}
	else
	{
		
	}
	adcResult_Sum += u16AdcResult_Temp;	
	
	if(bFullFlag)
	{
		adcAve = adcResult_Sum/adcLen;
		adcResult_Sum -= adcResult_Temp[index];
	}
	else
	{
		adcAve = adcResult_Sum/index;
	}
}

void getBatVol(void)
{
	stcAdcNormCfg.enAdcNormModeCh = AdcAVccDiV3Input;
    stcAdcNormCfg.bAdcResultAccEn = FALSE;
    Adc_ConfigNormMode(&stcAdcCfg, &stcAdcNormCfg);
	
	Adc_Start();
    while(FALSE != Adc_PollBusyState());
    Adc_GetResult(&u16AdcResult);
}

unsigned long regR 	= 0;
unsigned long regR1 = 0;

unsigned long vcc 	= 0;
unsigned long temv 	= 0;
unsigned long A = 0;
unsigned long B = 0;
unsigned long U = 0;
unsigned long I = 0;
volatile  uint16_t u16CHX_PWMDuty = 0;
void setPwmDuty(void);
//
#define targetA_base		1313	////1310//1346//1900//1313//1800//1400//1700//1900	//2000	//1120			// 6000
#define DELTA_PRE	16
#define DELTA_SMK	3
unsigned long targetA = targetA_base;
void adjustStep(int step)
{
	if(regR > targetA)
	{
		if(u16CHX_PWMDuty>step)
			u16CHX_PWMDuty-=step;
		state = smk;
		Led1Off();
		Led2On();
		gstartTime = gTime0Count;
	}				
	else
	{
		u16CHX_PWMDuty+=step;
		if(u16CHX_PWMDuty > 0x98)
		{
			u16CHX_PWMDuty = 0x98;
		}				
	}	
}

#define constDuty	30
void scan(void)
{    
    if(gFlag10ms)
    {
    	gFlag10ms = 0;
    	sendMsg(key_read());		
		
		if(state == smk)
		{
			checkTemperature();
			vcc = u16AdcResult*4500/4096;
			temv = u16AdcResult_Temp*1500*2/4096;
			U = vcc - temv;
			I = U / 10;
			regR = temv*1000/I;
		}			

		if(state == smk)
		{
			if(regR > targetA)
			{
				if(u16CHX_PWMDuty>DELTA_SMK)
					u16CHX_PWMDuty-=DELTA_SMK;
			}				
			else
			{
				u16CHX_PWMDuty+=DELTA_SMK;
				if(u16CHX_PWMDuty > 0x98/4*3)
				{
					u16CHX_PWMDuty = 0x98/4*3;
				}
			}	
			sendStr("X regR=");
			sendIntStr(regR);
//			u16CHX_PWMDuty = constDuty; 					// jackie
			PWM_TIMX->PWM_CHX = u16CHX_PWMDuty;	
		}					
	} 

	if(gFlag500ms)
	{
		gFlag500ms = 0;
	}
	
	if(gFlag200ms)
	{
		gFlag200ms = 0;
		if(gMotor.flag)
		{
			gMotor.cnt+=1;
			if(gMotor.cnt>gMotor.MaxCnt)
			{
				gMotor.flag 	= 0;
				gMotor.cnt 		= 0;
				gMotor.MaxCnt 	= 0;
				closeMotor();
			}
		}
		
		if(state == standby || state == charge)
		{
			scanInputPower();
		}		
		
		if(state == preHeat)
		{
			checkTemperature();
			vcc = u16AdcResult*4500/4096;
			temv = u16AdcResult_Temp*1500*2/4096;
			U = vcc - temv;
			I = U / 10;
			regR = temv*1000/I;
		  		
			if(regR > targetA && gTime0Count - gstartTime > 10000)					//&& gTime0Count - gstartTime > 10000
			{
				if(u16CHX_PWMDuty>DELTA_PRE)
					u16CHX_PWMDuty-=DELTA_PRE;
				state = smk;
				if(gMotor.flag==0)
				{
					gMotor.flag  = 1;
					gMotor.MaxCnt= 2;
					gMotor.cnt=0;
					openMotor();
				}
				Led1Off();
				Led2On();
				gstartTime = gTime0Count;
			}				
			else	
			{
				u16CHX_PWMDuty+=DELTA_PRE;
				if(u16CHX_PWMDuty > 150)
				{
					u16CHX_PWMDuty = 150;
				}				
			}
			
			sendStr("regR =");
			sendIntStr(regR);
			sendStr("PERAR =");
			sendIntStr(PWM_TIMX->PERAR);
//			u16CHX_PWMDuty = constDuty;                                 // jackie
			PWM_TIMX->PWM_CHX = u16CHX_PWMDuty;
			sendStr("PWM_CHX =");
			sendIntStr(PWM_TIMX->PWM_CHX);
    	}
	}		

	if(gFlag1000ms)
	{
		gFlag1000ms = 0;
		getBatVol();
/*
		checkTemperature();
		vcc = u16AdcResult*4500/4096;		
		//temv = u16AdcResult_Temp*1500*1190/510/4096;
		temv = u16AdcResult_Temp*1500*2/4096;
		U = vcc - temv;
		//I = U / 3;
		I = U / 10;
		regR = temv*1000/I;
		regR1 = 1071*1000*u16AdcResult_Temp/(459*u16AdcResult - 357*u16AdcResult_Temp);
		sendStr("VCC=");
		sendIntStr(vcc);
		sendStr("temv=");
		sendIntStr(temv);
		sendStr("regR=");
		sendIntStr(regR); 
*/
		sendStr("targetA=");
		sendIntStr(targetA);
		switch(state)
		{
			case sleepD:
				sendStr("state=sleepD\r\n");
				break;
			case sleep:
				sendStr("state=sleep\r\n");
				break;
			case standby:
				sendStr("state=standby\r\n");
				break;
			case smk:
				sendStr("state=smk\r\n");
				break;
			case charge:
				sendStr("state=charge\r\n");
				break;
			case preHeat:
				sendStr("state=preHeat\r\n");
				break;
			case debug:
				sendStr("state=debug\r\n");
				break;
			case upgrade:
				sendStr("state=upgrade\r\n");
				break;
			default:
				break;
		}
	}
}


void testPwm(void)
{
	if(flag==0)
	{
		changePwmDuty(0x3000);
		flag = 1;
	}
	else
	{
		changePwmDuty(0x6000);
		flag = 0;
	}	
}

void iniPwm(void)
{
	App_AdvTimerInit(0x0099, 0x0000, 0x0000);  		
    App_AdvTimerPortInit(); 
}

void startPwm(void)
{
	Adt_StartCount(PWM_TIMX);
}

void stopPwm(void)
{
	Adt_StopCount(PWM_TIMX);
}

void setPwmDuty(void)
{	
	PWM_TIMX->PWM_CHX = u16CHX_PWMDuty;
}

void clrPwmDuty()
{
	PWM_TIMX->PWM_CHX = 0;
}

void blink_LedALL(int num)
{
	int i=0;
	for(i=0;i<num;i++)
	{
		Led1On();
		Led2On();
		delay(200);
		Led1Off();
		Led2Off();
		delay(200);
	}
}

void blink_Led1(int num)
{
	int i=0;
	for(i=0;i<num;i++)
	{
		Led1On();
		delay(200);
		Led1Off();
		delay(200);
	}
}

void blink_Led2(int num)
{
	int i=0;
	for(i=0;i<num;i++)
	{
		Led2On();
		delay(200);
		Led2Off();
		delay(200);
	}
}

void setIniCheckTemp(void)
{
	int i=0;	
	for(i=0;i<adcLen;i++)
	{
		adcResult_Temp[i] = 0;
	}
	index = 0;
	adcAve = 0;
	bFullFlag = 0;
	adcResult_Sum = 0;	
}

void standbyfun(void)
{ 
    double dBatVol = 0;
    switch(msg)
    {
    	case CHG_PLUG:
			Led1On();
			state = charge;
			break;
    	case F_KEY:
			blink_LedALL(1);
			state = sleep;
			break;
        case S_KEY:
			if(u16AdcResult > (VOLTAGE_L*4096/4500))
			{
				blink_Led2(3);
			}
			else
			{
				blink_Led1(3);
			}
			//sendChar('S');
			sendStr("s\r\n");
            break;
		case L_KEY:
			sendStr("L\r\n");			
			if(u16AdcResult > (VOLTAGE_L*4096/4500))
			{
				Led1On();
				if(gMotor.flag==0)
				{
					gMotor.flag  = 1;
					gMotor.MaxCnt= 2;
					gMotor.cnt=0;
					openMotor();
					sendStr("gMotor open");
				}				
				gstartTime = gTime0Count;
				u16CHX_PWMDuty = 72;
				setIniCheckTemp();
				setPwmDuty();
				startPwm();
				state = preHeat;
			}
			else
			{
				blink_LedALL(3);
			}			
			break;
        case T_KEY:
			targetA += 33;
			if(targetA > targetA_base + 33*1)
			{
				targetA = targetA_base;
			}
			blink_LedALL(1);
            break;
		case Q_KEY:
            state = upgrade;
			Led1On();
			Led2On();
            break;
        default:
            break;
    }
}

void preHeatfun(void)
{
    switch(msg)
    {
        case L_KEY:
            state = standby;
			sendStr("preHeatfun\r\n");	
            stopPwm();            
			Led1Off();
            break;
        default:
            break;
    }
}

void exitSmk(void)
{
	Led2Off();
	state=standby;
	stopPwm();
	sendStr("exitSmk\r\n");			
}

void smkfun(void)
{
	switch(msg)
	{
		case L_KEY:
			exitSmk();
			if(gMotor.flag==0)
			{
				gMotor.flag  = 1;
				gMotor.MaxCnt= 2;
				gMotor.cnt=0;
				openMotor();
			}
			break;
		case T_KEY:
            //state = debug;
			//Led2On();
			targetA += 33;
			if(targetA > targetA_base + 33*1)
			{
				targetA = targetA_base;
			}
			blink_LedALL(1);
			Led2On();
            break; 
		default:
			if(gTime0Count - gstartTime >= 240000)
			{
				exitSmk();
				if(gMotor.flag==0)
				{
					gMotor.flag  = 1;
					gMotor.MaxCnt= 2;
					gMotor.cnt=0;
					openMotor();
				}
			}
			break;
	}
}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
void debugfun(void);
void upgradefun(void);

#define STRLEN	16
static char outstr[STRLEN]={0};
char* Itoa(unsigned int ni,int dd)
{
	int i = 0;
	char j=0,temp[16];
	unsigned int n,num=ni;
	
	while(num>=dd)
	{
		n=num%dd;
		if(n>9)
			temp[i]=n+0x37;
		else 
			temp[i]=n+0x30;
		num=num/dd;
		i++;
  }
  n=num;
	if(n>9)
		temp[i]=n+0x37;
	else 
		temp[i]=n+0x30;
  j=0;
  for(;i>=0;i--)
	{
		outstr[j]=temp[i];
		j++;
	}
  outstr[j++]='\r';
  outstr[j++]='\n';
  outstr[j]=0;
  return outstr;
}

void sendStr(char* pStr)
{
	int i=0;
	while(pStr[i])
	{
		Uart_SendDataPoll(M0P_UART1,pStr[i++]);
		delay(1);
	}
}

void sendChar(char c)
{
	Uart_SendDataPoll(M0P_UART1,c);
}

void sendIntStr(int num)
{	
	char*p = Itoa(num,10);
	sendStr(p);
}

void sleepfun(void)
{
	switch(msg)
	{
		case F_KEY:
			blink_LedALL(1);
			state = standby;
			sendStr("sleepfun\r\n");	
			break;
		default:
			break;
	}	
}

void chargefun(void)
{	
	switch(msg)
	{
		case CHG_UNPLUG:			
			LedAllOff();
			state = standby;
			sendStr("chargefun1\r\n");
			break;
		case CHG_FULL:
			Led1Off();
			Led2On();
			sendStr("chargefun2\r\n");
			break;
		case CHG_NOFULL:
			Led1On();
			Led2Off();
			sendStr("chargefun3\r\n");
			break;
		default:
			break;
	}	
}

void testFun(void)								
{
	sendStr("66 ha ha !\r\n");
	sendIntStr(12546);
}
	
int32_t main(void)
{    
	App_Init();
    while(1)
    {
        scan();
    	msg=getMsg();
    	switch(state)
        {
            case standby:
                standbyfun();
                break;
            case preHeat:
                preHeatfun();
                break;
    		case smk:
                smkfun();
                break;
            case debug:
                debugfun();
                break;
    		case upgrade:
    			upgradefun();
    			break;
    		case sleep:
    			sleepfun();
    			break;
    		case charge:
    			chargefun();
    			break;
            default:
                break;
        }	
    }
}

static void _UartBaudCfg(void)
{
    uint16_t timer=0;
    stc_uart_baud_cfg_t stcBaud;
    stc_bt_cfg_t stcBtCfg;
    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);

    stcBaud.bDbaud  = 1u;
    stcBaud.u32Baud = 19200u;
    stcBaud.enMode  = UartMode1;
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    timer = Uart_SetBaudRate(M0P_UART1, &stcBaud);

    stcBtCfg.enMD = BtMode2;
    stcBtCfg.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtCfg);
    Bt_ARRSet(TIM1,timer);
    Bt_Cnt16Set(TIM1,timer);
    Bt_Run(TIM1);
}

void debugfun(void)
{
	switch(msg)
	{
		case T_KEY:
			Led2Off();
			state = standby;
			sendStr("debugfun\r\n");
			break;
		default:
			break;
	}
}

void upgradefun(void)
{
	switch(msg)
	{
		case Q_KEY:
			Led2Off();
			Led1Off();
			Sysctrl_SetFunc(SysctrlSWDUseIOEn,FALSE);
			state = standby;
			sendStr("upgradefun\r\n");
			break;
		default:
			break;
	}
}

#undef adcLen

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


