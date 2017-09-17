/*
 *  rommon Simple Monitor
 * 
 *  Copyright (C) 1999-2004 by Dep. of Computer Science and Engineering
 *                   Tomakomai National College of Technology, JAPAN
 *  Copyright (C) 2015      by Roi Takeuchi
 *
 *  上記著作権者は，以下の (1)～(4) の条件か，Free Software Foundation 
 *  によって公表されている GNU General Public License の Version 2 に記
 *  述されている条件を満たす場合に限り，本ソフトウェア（本ソフトウェア
 *  を改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，その適用可能性も
 *  含めて，いかなる保証も行わない．また，本ソフトウェアの利用により直
 *  接的または間接的に生じたいかなる損害に関しても，その責任を負わない．
 * 
 *  @(#) $Id: $
 */
/*******************************************************************************
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
  ******************************************************************************/

/*
 *	rommon Serial Functions
 */
#include "sys_defs.h"
#include "sil.h"

#define VECT_TAB_OFFSET     0x00		/* Vector Table base offset field. 
                                		   This value must be a multiple of 0x200. */
#define HSE_STARTUP_TIMEOUT 0x0500		/* Time out for HSE start up */
#define HSI_VALUE           16000000	/* Value of the Internal oscillator in Hz*/
#define HSE_VALUE           8000000

/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      8
#define PLL_N      336

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      7

/* PLLI2S_VCO = (HSE_VALUE Or HSI_VALUE / PLL_M) * PLLI2S_N
   I2SCLK = PLLI2S_VCO / PLLI2S_R */
#define PLLI2S_N   192
#define PLLI2S_R   5

#define  RCC_CFGR_SW       (RCC_CFGR_SW_0 | RCC_CFGR_SW_1)
#define  RCC_CFGR_SWS      (RCC_CFGR_SWS_0 | RCC_CFGR_SWS_1)
#define  RCC_CFGR_HPRE     (RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_3)
#define  RCC_CFGR_PPRE1    (RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_2)
#define  RCC_CFGR_PPRE2    (RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_2)
#define  RCC_PLLCFGR_PLLM  (RCC_PLLCFGR_PLLM_0 | RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_2 \
                          | RCC_PLLCFGR_PLLM_3 | RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLM_5)
#define  RCC_PLLCFGR_PLLN  (RCC_PLLCFGR_PLLN_0 | RCC_PLLCFGR_PLLN_1 | RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_3 \
                          | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLN_8)
#define  RCC_PLLCFGR_PLLP  (RCC_PLLCFGR_PLLP_0 | RCC_PLLCFGR_PLLP_1)
#define  RCC_PLLCFGR_PLLSRC (RCC_PLLCFGR_PLLSRC_HSE)


/*
 *  GPIO PIN POSITION
 */
#define TX2_PINPOS         2
#define RX2_PINPOS         3
#define TX1_PINPOS         6
#define RX1_PINPOS         7

/*
 *  USART Priority
 */
#define MAIN_PRIORITY      0
#define SUB_PRIORITY       0

/*
 *  GPIO Configuration Mode enumeration
 */
#define GPIO_Mode_IN    0x0					/* GPIO Input Mode */
#define GPIO_Mode_OUT   GPIO_MODER_MODER0	/* GPIO Output Mode */
#define GPIO_Mode_AF    GPIO_MODER_MODER1	/* GPIO Alternate function Mode */
#define GPIO_Mode_AN    GPIO_MODER_MODER2	/* GPIO Analog Mode */

/*
 *  GPIO Output type enumeration
 */
#define GPIO_OType_PP   0x0
#define GPIO_OType_OD   0x1

/*
 *  GPIO Output Maximum frequency enumeration
 */
#define GPIO_Speed_2MHz   0x0						/* Low speed */
#define GPIO_Speed_25MHz  GPIO_OSPEEDER_OSPEEDR0	/* Medium speed */
#define GPIO_Speed_50MHz  GPIO_OSPEEDER_OSPEEDR1	/* Fast speed */
#define GPIO_Speed_100MHz GPIO_OSPEEDER_OSPEEDR2	/* High speed on 30 pF (80 MHz Output max speed on 15 pF) */

/*
 *  GPIO Configuration PullUp PullDown enumeration
 */
#define GPIO_PuPd_NOPULL 0x0
#define GPIO_PuPd_UP     GPIO_PUPDR_PUPDR0
#define GPIO_PuPd_DOWN   GPIO_PUPDR_PUPDR1

/*
 *  GPIO Pin Source
 */ 
#define GPIO_PinSource0         0x00
#define GPIO_PinSource1         0x01
#define GPIO_PinSource2         0x02
#define GPIO_PinSource3         0x03
#define GPIO_PinSource4         0x04
#define GPIO_PinSource5         0x05
#define GPIO_PinSource6         0x06
#define GPIO_PinSource7         0x07
#define GPIO_PinSource8         0x08
#define GPIO_PinSource9         0x09
#define GPIO_PinSource10        0x0A
#define GPIO_PinSource11        0x0B
#define GPIO_PinSource12        0x0C
#define GPIO_PinSource13        0x0D
#define GPIO_PinSource14        0x0E
#define GPIO_PinSource15        0x0F

/*
 *  AF 7 selection
 */
#define GPIO_AF_USART1          0x07	/* USART1 Alternate Function mapping */
#define GPIO_AF_USART2          0x07	/* USART2 Alternate Function mapping */
#define GPIO_AF_USART3          0x07	/* USART3 Alternate Function mapping */
#define GPIO_AF_I2S3ext         0x07	/* I2S3ext Alternate Function mapping */

/*
 *  USART Word Length
 */
#define USART_WordLength_8b     0x0000
#define USART_WordLength_9b     USART_CR1_M

/*
 *  USART Stop Bits
 */ 
#define USART_StopBits_1        0x0000
#define USART_StopBits_0_5      USART_CR2_STOP_0
#define USART_StopBits_2        USART_CR2_STOP_1
#define USART_StopBits_1_5      USART_CR2_STOP

/*
 *  USART Parity
 */
#define USART_Parity_No         0x0000
#define USART_Parity_Even       USART_CR1_PCE
#define USART_Parity_Odd        (USART_CR1_PCE | USART_CR1_PS) 

/*
 *  USART MODE
 */
#define USART_Mode_Rx           USART_CR1_RE
#define USART_Mode_Tx           USART_CR1_TE

/** @defgroup USART_Hardware_Flow_Control 
  * @{
  */ 
#define USART_HardwareFlowControl_None       0x0000
#define USART_HardwareFlowControl_RTS        USART_CR3_RTSE
#define USART_HardwareFlowControl_CTS        USART_CR3_CTSE
#define USART_HardwareFlowControl_RTS_CTS    (USART_CR3_RTSE | USART_CR3_CTSE)

#define CR1_CLEAR_MASK          (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE)
#define CR3_CLEAR_MASK          (USART_CR3_RTSE | USART_CR3_CTSE)


typedef struct {
	unsigned int SYSCLK_Frequency;	/*  SYSCLK clock frequency expressed in Hz */
	unsigned int HCLK_Frequency;	/*  HCLK clock frequency expressed in Hz */
	unsigned int PCLK1_Frequency;	/*  PCLK1 clock frequency expressed in Hz */
	unsigned int PCLK2_Frequency;	/*  PCLK2 clock frequency expressed in Hz */
}RCC_ClocksTypeDef;

extern unsigned int IsrVector[];

/*
 *  Modified 16bit Port
 */
static void sil_mdh_mem(VP addr, VH mask, VH val)
{
	VH tmpreg  = sil_reh_mem(addr) & ~mask;
	sil_wrh_mem(addr, tmpreg | val);
}

#if ROM_EXEC == 1
/*
 *  Modified 32bit Port
 */
static void sil_mdw_mem(VP addr, VW mask, VW val)
{
	VW tmpreg  = sil_rew_mem(addr) & ~mask;
	sil_wrw_mem(addr, tmpreg | val);
}

static void setup_gpio_source(VP base, unsigned char source, unsigned char select)
{
	VW tmpreg, regoff;
	regoff = TOFF_GPIO_AFR0+((source>>1) & 0x4);
	tmpreg = (sil_rew_mem((VP)(base+regoff)) & ~(0xF << ((source & 0x07) * 4)))
			    | (select << ((source & 0x07) * 4));
	sil_wrw_mem((VP)(base+regoff), tmpreg);
}

/*
 * Configures the System clock source, PLL Multiplier and Divider factors,
 * AHB/APBx prescalers and Flash settings
 * @Note   This function should be called only once the RCC clock configuration
 *         is reset to the default reset state (done in SystemInit() function).
 * @param  None
 * @retval None
 */
static void SetSysClock(void)
{
	unsigned int StartUpCounter = 0, HSEStatus = 0;

	/* Enable HSE */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR), 0, RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do{
		HSEStatus = sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR)) & RCC_CR_HSERDY;
		StartUpCounter++;
	}while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR)) & RCC_CR_HSERDY) != 0){
		/* Enable high performance mode, System frequency up to 168 MHz */
		sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_APB1ENR), 0, RCC_APB1ENR_PWREN);
		sil_mdw_mem((VP)(TADR_PWR_BASE+TOFF_PWR_CR), 0, PWR_CR_PMODE);

		/* HCLK = SYSCLK / 1*/
		sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR), 0, RCC_CFGR_HPRE_DIV1);

		/* PCLK2 = HCLK / 2*/
		sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR), 0, RCC_CFGR_PPRE2_DIV2);

		/* PCLK1 = HCLK / 4*/
		sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR), 0, RCC_CFGR_PPRE1_DIV4);

		/* Configure the main PLL */
		sil_wrw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR), PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24));

 		/* Enable the main PLL */
		sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR), 0, RCC_CR_PLLON);

		/* Wait till the main PLL is ready */
		while((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR)) & RCC_CR_PLLRDY) == 0)
		{
		}

		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		sil_wrw_mem((VP)(TADR_FLASH_R_BASE+TOFF_FLASH_ACR), FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS);

		/* Select the main PLL as system clock source */
		sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR), RCC_CFGR_SW, RCC_CFGR_SW_PLL);

		/* Wait till the main PLL is used as system clock source */
		while((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR)) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		{
		}
	}
	else{
		/* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
	}

	/* PLLI2S clock used as I2S clock source */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR), RCC_CFGR_I2SSRC, 0);

	/* Configure PLLI2S */
	sil_wrw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLI2SCFGR), (PLLI2S_N << 6) | (PLLI2S_R << 28));

	/* Enable PLLI2S */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR), 0, RCC_CR_PLLI2SON);

	/* Wait till PLLI2S is ready */
	while((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR)) & RCC_CR_PLLI2SRDY) == 0)
	{
	}
}
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const unsigned char APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

/*
 * Returns the frequencies of different on chip clocks; SYSCLK, HCLK, 
 * PCLK1 and PCLK2.
 * @note   The system frequency computed by this function is not the real 
 *         frequency in the chip. It is calculated based on the predefined 
 *         constant and the selected clock source:
 * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
 * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
 * @note     If SYSCLK source is PLL, function returns values based on HSE_VALUE(**) 
 *           or HSI_VALUE(*) multiplied/divided by the PLL factors.         
 * @note     (*) HSI_VALUE is a constant defined in stm32f4xx.h file (default value
 *               16 MHz) but the real value may vary depending on the variations
 *               in voltage and temperature.
 * @note     (**) HSE_VALUE is a constant defined in stm32f4xx.h file (default value
 *                25 MHz), user has to ensure that HSE_VALUE is same as the real
 *                frequency of the crystal used. Otherwise, this function may
 *                have wrong result.
 *
 * @note   The result of this function could be not correct when using fractional
 *         value for HSE crystal.
 *
 * @param  RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold
 *         the clocks frequencies.
 *
 * @note   This function can be used by the user application to compute the 
 *         baudrate for the communication peripherals or configure other parameters.
 * @note   Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
 *         must be called to update the structure's field. Otherwise, any
 *         configuration based on this function will be incorrect.
 *
 * @retval None
 */
static void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
	unsigned int tmp = 0, presc = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

	/* Get SYSCLK source -------------------------------------------------------*/
	tmp = sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR)) & RCC_CFGR_SWS;

	switch (tmp){
	case 0x00:  /* HSI used as system clock source */
		RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
		break;
	case 0x04:  /* HSE used as system clock  source */
		RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
		break;
	case 0x08:  /* PLL used as system clock  source */

		/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
		   SYSCLK = PLL_VCO / PLLP
		 */
		pllsource = (sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR)) & RCC_PLLCFGR_PLLSRC) >> 22;
		pllm = sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR)) & RCC_PLLCFGR_PLLM;

		if (pllsource != 0)	/* HSE used as PLL clock source */
			pllvco = (HSE_VALUE / pllm) * ((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR)) & RCC_PLLCFGR_PLLN) >> 6);
		else				/* HSI used as PLL clock source */
			pllvco = (HSI_VALUE / pllm) * ((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR)) & RCC_PLLCFGR_PLLN) >> 6);

		pllp = (((sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR)) & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
		RCC_Clocks->SYSCLK_Frequency = pllvco/pllp;
		break;
	default:
		RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
		break;
	}
	/* Compute HCLK, PCLK1 and PCLK2 clocks frequencies ------------------------*/

	/* Get HCLK prescaler */
	tmp = sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR)) & RCC_CFGR_HPRE;
	tmp = tmp >> 4;
	presc = APBAHBPrescTable[tmp];
	/* HCLK clock frequency */
	RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;

	/* Get PCLK1 prescaler */
	tmp = sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR)) & RCC_CFGR_PPRE1;
	tmp = tmp >> 10;
	presc = APBAHBPrescTable[tmp];
	/* PCLK1 clock frequency */
	RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

	/* Get PCLK2 prescaler */
	tmp = sil_rew_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR)) & RCC_CFGR_PPRE2;
	tmp = tmp >> 13;
	presc = APBAHBPrescTable[tmp];
	/* PCLK2 clock frequency */
	RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
}

void sys_printf(const char *s);

/*
 * Setup the microcontroller system
 * Initialize the Embedded Flash Interface, the PLL and update the SystemFrequency variable.
 * @param  None
 * @retval None
 */
void hardware_init_hook(void)
{
	RCC_ClocksTypeDef RCC_ClocksStatus;
	unsigned int  tmpreg = 0x00, apbclock = 0x00;
	unsigned int  integerdivider = 0x00;
	unsigned int  fractionaldivider = 0x00;

#if ROM_EXEC == 1
	/* Reset the RCC clock configuration to the default reset state ------------*/
	/* Set HSION bit */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR), 0, RCC_CR_HSION);

	/* Reset CFGR register */
	sil_wrw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CFGR), 0x00000000);

	/* Reset HSEON, CSSON and PLLON bits */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR), RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEON, 0);

	/* Reset PLLCFGR register */
	sil_wrw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_PLLCFGR), RCC_CFGR_MCO2PRE_2 | RCC_CFGR_MCO1PRE_2
					 | RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_HPRE_0);

	/* Reset HSEBYP bit */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CR), RCC_CR_HSEBYP, 0);

	/* Disable all interrupts */
	sil_wrw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_CIR), 0x00000000);

	/* SystemInit_ExtMemCtl(); no ExtMem */

	/* Configure the System clock source, PLL Multiplier and Divider factors, 
       AHB/APBx prescalers and Flash settings ----------------------------------*/
	SetSysClock();
#endif

	/* Configure the Vector Table location add offset address ------------------*/
	sil_wrw_mem((VP)(TADR_SCB_BASE+TOFF_SCB_VTOR), (VW)IsrVector);	/* Vector Table Relocation in Internal SRAM */

#if ROM_EXEC == 1
	/* enable APB1 peripheral clock for USART2 */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_APB1ENR), 0, RCC_APB1ENR_USART2EN);

	/* enable the peripheral clock for the pins used by USART2, PA2 for TX and PA3 for RX */
	sil_mdw_mem((VP)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), 0, RCC_AHB1ENR_GPIOAEN);

	/* This sequence sets up the TX pin so they work correctly with the USART2 peripheral */
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_MODER), (GPIO_MODER_MODER2 << (TX2_PINPOS*2)), (GPIO_Mode_AF << (TX2_PINPOS*2)));

	/* Speed mode configuration */
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_OSPEEDR), (GPIO_OSPEEDER_OSPEEDR2 << (TX2_PINPOS*2)), (GPIO_Speed_50MHz << (TX2_PINPOS*2)));

	/* Output mode configuration*/
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_OTYPER), (GPIO_OTYPER_OT << TX2_PINPOS), (GPIO_OType_PP << TX2_PINPOS));

	/* Pull-up Pull down resistor configuration*/
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_PUPDR), (GPIO_PUPDR_PUPDR2 << (TX2_PINPOS*2)), (GPIO_PuPd_UP << (TX2_PINPOS*2)));

	/* This sequence sets up the RX pin so they work correctly with the USART2 peripheral */
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_MODER), (GPIO_MODER_MODER2 << (RX2_PINPOS*2)), (GPIO_Mode_AF << (RX2_PINPOS*2)));

    /* Speed mode configuration */
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_OSPEEDR), (GPIO_OSPEEDER_OSPEEDR2 << (RX2_PINPOS*2)), (GPIO_Speed_50MHz << (RX2_PINPOS*2)));

	/* Output mode configuration*/
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_OTYPER), (GPIO_OTYPER_OT << RX2_PINPOS), (GPIO_OType_PP << RX2_PINPOS));

	/* Pull-up Pull down resistor configuration*/
	sil_mdw_mem((VP)(TADR_GPIOA_BASE+TOFF_GPIO_PUPDR), (GPIO_PUPDR_PUPDR2 << (RX2_PINPOS*2)), (GPIO_PuPd_UP << (RX2_PINPOS*2)));

	/* The RX and TX pins are now connected to their AF so that the USART2 can take over control of the pins */
	setup_gpio_source((VP)TADR_GPIOA_BASE, GPIO_PinSource2, GPIO_AF_USART2);
	setup_gpio_source((VP)TADR_GPIOA_BASE, GPIO_PinSource3, GPIO_AF_USART2);
#endif

	/* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit :
	   Set STOP[13:12] bits according to USART_StopBits value */
	sil_mdh_mem((VP)(TADR_USART2_BASE+TOFF_USART_CR2), USART_CR2_STOP, USART_StopBits_1);

	/* Configure the USART Word Length, Parity and mode: 
	   Set the M bits according to USART_WordLength value 
	   Set PCE and PS bits according to USART_Parity value
	   Set TE and RE bits according to USART_Mode value */
	sil_mdh_mem((VP)(TADR_USART2_BASE+TOFF_USART_CR1), CR1_CLEAR_MASK, USART_WordLength_8b | USART_Parity_No | USART_Mode_Rx | USART_Mode_Tx);

	/* Configure the USART HFC : 
	   Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
	sil_mdh_mem((VP)(TADR_USART2_BASE+TOFF_USART_CR3), CR3_CLEAR_MASK, USART_HardwareFlowControl_None);

	/* Configure the USART Baud Rate */
	RCC_GetClocksFreq(&RCC_ClocksStatus);

	apbclock = RCC_ClocksStatus.PCLK1_Frequency;

	/* Determine the integer part */
	if ((sil_reh_mem((VP)(TADR_USART2_BASE+TOFF_USART_CR1)) & USART_CR1_OVER8) != 0){
		integerdivider = ((25 * apbclock) / (2 * (DEFAULT_SPEED)));	/* 8 Samples */
		tmpreg = (integerdivider / 100) << 4;
		fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
		tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & 0x07;
	}
	else{ /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
		integerdivider = ((25 * apbclock) / (4 * (DEFAULT_SPEED)));	/* 16 Samples */
		tmpreg = (integerdivider / 100) << 4;
		fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
		tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & 0x0F;
	}
	/* Write to USART BRR register */
	sil_wrh_mem((VP)(TADR_USART2_BASE+TOFF_USART_BRR), tmpreg);

	sil_mdh_mem((VP)(TADR_USART2_BASE+TOFF_USART_CR1), 0, USART_CR1_RXNEIE);

    /* Enable the selected USART by setting the UE bit in the CR1 register */
	sil_mdh_mem((VP)(TADR_USART2_BASE+TOFF_USART_CR1), 0, USART_CR1_UE);
}


void sys_putc(int c)
{
	while((sil_reh_mem((VP)(TADR_USART2_BASE+TOFF_USART_SR)) & USART_SR_TC) == 0);
	sil_wrh_mem((VP)(TADR_USART2_BASE+TOFF_USART_DR), (c & 0xFF));
}

void sys_puthex(unsigned int val)
{
	int i, j;

	for(i = 32; i > 0 ; i -= 4){
		j = (val >> (i-4)) & 15;
		if(j < 10)
			sys_putc(j + '0');
		else
			sys_putc(j - 10 + 'A');
	}
}

void sys_printf(const char *s)
{
	int c;

	while((c = *s++) != 0){
		if(c == '\n')
			sys_putc('\r');
		sys_putc(c);
	}
}

/*
 *  未定義の例外が入った場合の処理
 */
void NMI_Handler(void)
{
    sys_printf("NMI Exception occurs. !\n");
    while(1);
}

void HardFault_Handler(void)
{
    sys_printf("Hard Fault Exception occurs. !\n");
    while(1);
}

void MemManage_Handler(void)
{
    sys_printf("Memory Manager Exception occurs. !\n");
    while(1);
}

void BusFault_Handler(void)
{
    sys_printf("Bus Fault Exception occurs. !\n");
    while(1);
}


void UsageFault_Handler(void)
{
    sys_printf("Usage Fault Exception occurs. !\n");
    while(1);
}

void DebugMon_Handler(void)
{
    sys_printf("Debug Monitor Exception occurs. !\n");
    while(1);
}

void PendSV_Handler(void)
{
    sys_printf("Pend SV Exception occurs. !\n");
    while(1);
}

