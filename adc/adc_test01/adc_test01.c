/*
 * file name: adc_test01
 * description: Single conversion mode
 * date: 2017/08/15
 * note: analog input is not 5V torrerant.
 */

#include "sys_defs.h"
#include "sil.h"
#include <stdint.h>

/* extend sil */
#define sil_orw_mem(a, b) sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b) sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
/* modified write */
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

/* Utilities function */
#define DELAY_LOOP (500*1000)
void busy_wait(void)
{
	long i;

	for (i = 0; i < DELAY_LOOP; i++)
		sil_dly_nse(1000);	/* 1 micro sec */
}

#define A0_PINPOS	0		/* A0 */
#define A1_PINPOS	1		/* A1 */

#define GPIO_MODER_ANALOG	0x03	/* Analog input */

/* ADC registers */
#define ADC_CR2_ADON	0x00000001
#define ADC_SQR1_L		0x00F00000
#define ADC_SQR1_L_1_CONVERSION	0x0
#define ADC_SQR3_SQ1	0x0000001F
#define ADC_CH0			0x0
#define ADC_CR2_SWSTART 0x40000000
#define ADC_SR_EOC		0x00000002

void print_register(char *register_name, uint32_t address) 
{
	uint32_t temp;

	temp = sil_rew_mem((uint32_t *)(address));
	sys_printf(register_name);
	sys_printf("=");
	sys_puthex(temp);
	sys_printf("\n");
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
#define ADC_STB_DELAY_US 3
	sil_dly_nse(ADC_STB_DELAY_US*1000);
	print_register("ADC_CR2", TADR_ADC1_BASE + TOFF_ADC_CR2);

	/* ADC channel select ch0 on ADC1 */
	/* ADC Regular channel sequence length is 1 conversion */
	sil_modw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SQR1), 
			ADC_SQR1_L, ADC_SQR1_L_1_CONVERSION << 20);
	/*
	sil_modw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SQR1), 
			ADC_SQR1_L, 0xf);
	 */
	print_register("ADC_SQR1", TADR_ADC1_BASE + TOFF_ADC_SQR1);

	/* ADC 1st conversion in regure sequence is channel 0 */
	sil_modw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SQR3),
			ADC_SQR3_SQ1, ADC_CH0);
}

/* when make, added DEBUG=1 */

void _main(void)
{
	uint16_t ad_data;

	print_register("RCC_CR", TADR_RCC_BASE + TOFF_RCC_CR);
	print_register("RCC_CFGR", TADR_RCC_BASE + TOFF_RCC_CFGR);
	print_register("RCC_PLLCFGR", TADR_RCC_BASE + TOFF_RCC_PLLCFGR);
	print_register("RCC_AHB1ENR", TADR_RCC_BASE + TOFF_RCC_AHB1ENR);

	adc_init();

	for(;;) {
		/* ADC start */
		sil_orw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_CR2), ADC_CR2_SWSTART);

		/* check end of ADC */
		while ((sil_rew_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SR)) &
					ADC_SR_EOC) == 0) {
		}
		 
		/* get ADC data */
		ad_data = sil_reh_mem((uint16_t *)(TADR_ADC1_BASE + TOFF_ADC_DR));
		ad_data &= 0x0FFF;	/* right alignment */

		/* flag(EOC:End of conversion) clear */
		/* does automatic clear when read ADC_DR? */
		
		sys_puthex(ad_data);
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

