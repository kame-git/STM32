/*
 * file name: adc_test02
 * description: Single conversion mode using interrupt.
 * date: 2017/08/20
 * note: analog input is not 5V torrerant.
 * note: シングル変換モード
 */

#include "sys_defs.h"
#include "sil.h"
#include <stdint.h>

#define sil_orw_mem(a, b) sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b) sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
/* modified write */
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

/* Utilities function */
#define DELAY_LOOP (500*1000)	/* ROMとRAMでは実行速度が異なるので注意 */

void busy_wait(void)
{
	long i;

	for (i = 0; i < DELAY_LOOP; i++)
		sil_dly_nse(1000);	/* 1 micro sec */
}

/* ADC関連マクロ定義 */
#define A0_PINPOS	0		/* A0 */
#define A1_PINPOS	1		/* A1 */

#define GPIO_MODER_ANALOG	0x03	/* Analog input */

#define ADC_CR2_ADON			0x00000001
#define ADC_SQR1_L				0x00F00000	/* シーケンス回数の位置 */
#define ADC_SQR1_L_1_CONVERSION	0x0			/* シーケンス回数 */
#define ADC_SQR3_SQ1			0x0000001F	/* シーケンス番号1 */
#define ADC_CH0					0x0
#define ADC_CR2_SWSTART			0x40000000
#define ADC_SR_EOC				0x00000002
#define INTNO_ADC				18			/* ADC1, ADC2 and ADC3 global Interrupt */
#define ADC_STB_DELAY_US		3			/* ADON後の待ち時間 */
#define ADC_CR1_EOCIE			0x20		/* EOC interrupt enable */

/* レジスタ値表示（デバッグ用） */
void print_register(char *register_name, uint32_t address) 
{
	uint32_t temp;

	temp = sil_rew_mem((uint32_t *)(address));
	sys_printf(register_name);
	sys_printf("=");
	sys_puthex(temp);
	sys_printf("\n");
}

/* NVIC関連マクロ定義 */
#define TADR_NVIC_ISER      0xE000E100	/* NVIC INT SET ENABLEレジスタ */
/* NVIC INT SET ENABLEレジスタ */
#define TREG_NVIC_ISER      ((volatile unsigned long *)(TADR_NVIC_ISER))		

/* 割込み初期化(NVIC) */
void int_init(void)
{
	/* 優先度は規定のまま使用 */
	
	/* NVIC ISER setup */
	/* 割込み番号を32で割る（0x05）ことでISERxの番号を計算 */
	/* 割込み番号と0X1Fの論理積でbit位置を計算 */
	//TREG_NVIC_ISER[INTNO_ADC >> 0x05] = 0x01 << (INTNO_ADC & 0x1F);
	TREG_NVIC_ISER[0] =	1 << INTNO_ADC;
}

void adc_init(void)
{
	/* just finished clock setting at stm32f4xx.c */
	/* PCLK2 is 90MHz */

	/* RCC APB2 periperal clock enable register(RCC_APB2ENR) */
	/* Bit 8 ADC1EN: ADC1 clock enalbe */
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE + TOFF_RCC_APB2ENR),
			RCC_APB2ENR_ADC1EN);
	print_register("RCC_APB2ENR", TADR_RCC_BASE + TOFF_RCC_APB2ENR);

	/* GPIOA setup analog mode */
	sil_modw_mem((uint32_t *)(TADR_GPIOA_BASE + TOFF_GPIO_MODER),
			GPIO_MODER_MODER2 << (A0_PINPOS * 2),
			GPIO_MODER_ANALOG << (A0_PINPOS * 2));
	print_register("GPIOA_MODER", TADR_GPIOA_BASE + TOFF_GPIO_MODER);

	/* ADC enable */
	sil_orw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_CR2), ADC_CR2_ADON);
	sil_dly_nse(ADC_STB_DELAY_US*1000);
	print_register("ADC_CR2", TADR_ADC1_BASE + TOFF_ADC_CR2);

	/* ADC channel select ch0 on ADC1 */
	/* ADC Regular channel sequence length is 1 conversion */
	sil_modw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SQR1), 
			ADC_SQR1_L, ADC_SQR1_L_1_CONVERSION << 20);
	sil_modw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SQR1), 
			ADC_SQR1_L, 0xf);
	print_register("ADC_SQR1", TADR_ADC1_BASE + TOFF_ADC_SQR1);

	/* ADC 1st conversion in regure sequence is channel 0 */
	sil_modw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SQR3),
			ADC_SQR3_SQ1, ADC_CH0);

	/* 割込み許可 */
	sil_orw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_CR1),
			ADC_CR1_EOCIE);
	print_register("ADC_CR1", TADR_ADC1_BASE + TOFF_ADC_CR1);

}

/* 割込みハンドラとmain関数で使用するグローバル変数の宣言 */
volatile uint8_t adc_eoc_flag;	/* 割込み発生通知用フラグ */
volatile uint16_t ad_data;		/* AD変換値 */

/* ADCグローバル割込み */
void ADC_IRQHandler(void)
{
	uint32_t temp;

	sys_printf("Interrupt occured!\n");

	/* ステータスレジスタを確認し、ADC1のEOCが1か確認 */
	temp = sil_rew_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SR));
	if ((temp & ADC_SR_EOC) != 0) {
		/* IDRからデータを取得 */
		ad_data = sil_reh_mem((uint16_t *)(TADR_ADC1_BASE + TOFF_ADC_DR));
		sys_printf("To set a flag\n");
		adc_eoc_flag = 1;	/* 割込み発生を通知 */
	}

	/* IDRからデータを取得するとEOCは自動でクリア */
	
}

void _main(void)
{
	uint16_t data;

	print_register("RCC_CR", TADR_RCC_BASE + TOFF_RCC_CR);
	print_register("RCC_CFGR", TADR_RCC_BASE + TOFF_RCC_CFGR);
	print_register("RCC_PLLCFGR", TADR_RCC_BASE + TOFF_RCC_PLLCFGR);
	print_register("RCC_AHB1ENR", TADR_RCC_BASE + TOFF_RCC_AHB1ENR);

	int_init();		/* 割込み設定 */
	print_register("NVIC_ISER0", TADR_NVIC_ISER);

	adc_init();		/* ADC設定と割込み許可 */

	for(;;) {
		/* ADC start */
		sil_orw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_CR2), ADC_CR2_SWSTART);

		while (adc_eoc_flag == 0); /* EOC割込み発生待ち */

		data = ad_data;		/* グローバル変数の値をコピー */
		adc_eoc_flag = 0;	/* 割込み通知フラグをクリア */

		/* データが右にアライメントされているので、不要なビットを削除 */
		data &= 0x0FFF;

		sys_puthex(data);
		sys_printf("\n");
		busy_wait();
		busy_wait();
		busy_wait();
		busy_wait();
		busy_wait();
		busy_wait();
		busy_wait();
		busy_wait();
	}
}

