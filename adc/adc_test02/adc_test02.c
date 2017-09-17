/*
 * file name: adc_test02
 * description: Single conversion mode using interrupt.
 * date: 2017/08/20
 * note: analog input is not 5V torrerant.
 * note: �V���O���ϊ����[�h
 */

#include "sys_defs.h"
#include "sil.h"
#include <stdint.h>

#define sil_orw_mem(a, b) sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b) sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
/* modified write */
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

/* Utilities function */
#define DELAY_LOOP (500*1000)	/* ROM��RAM�ł͎��s���x���قȂ�̂Œ��� */

void busy_wait(void)
{
	long i;

	for (i = 0; i < DELAY_LOOP; i++)
		sil_dly_nse(1000);	/* 1 micro sec */
}

/* ADC�֘A�}�N����` */
#define A0_PINPOS	0		/* A0 */
#define A1_PINPOS	1		/* A1 */

#define GPIO_MODER_ANALOG	0x03	/* Analog input */

#define ADC_CR2_ADON			0x00000001
#define ADC_SQR1_L				0x00F00000	/* �V�[�P���X�񐔂̈ʒu */
#define ADC_SQR1_L_1_CONVERSION	0x0			/* �V�[�P���X�� */
#define ADC_SQR3_SQ1			0x0000001F	/* �V�[�P���X�ԍ�1 */
#define ADC_CH0					0x0
#define ADC_CR2_SWSTART			0x40000000
#define ADC_SR_EOC				0x00000002
#define INTNO_ADC				18			/* ADC1, ADC2 and ADC3 global Interrupt */
#define ADC_STB_DELAY_US		3			/* ADON��̑҂����� */
#define ADC_CR1_EOCIE			0x20		/* EOC interrupt enable */

/* ���W�X�^�l�\���i�f�o�b�O�p�j */
void print_register(char *register_name, uint32_t address) 
{
	uint32_t temp;

	temp = sil_rew_mem((uint32_t *)(address));
	sys_printf(register_name);
	sys_printf("=");
	sys_puthex(temp);
	sys_printf("\n");
}

/* NVIC�֘A�}�N����` */
#define TADR_NVIC_ISER      0xE000E100	/* NVIC INT SET ENABLE���W�X�^ */
/* NVIC INT SET ENABLE���W�X�^ */
#define TREG_NVIC_ISER      ((volatile unsigned long *)(TADR_NVIC_ISER))		

/* �����ݏ�����(NVIC) */
void int_init(void)
{
	/* �D��x�͋K��̂܂܎g�p */
	
	/* NVIC ISER setup */
	/* �����ݔԍ���32�Ŋ���i0x05�j���Ƃ�ISERx�̔ԍ����v�Z */
	/* �����ݔԍ���0X1F�̘_���ς�bit�ʒu���v�Z */
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

	/* �����݋��� */
	sil_orw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_CR1),
			ADC_CR1_EOCIE);
	print_register("ADC_CR1", TADR_ADC1_BASE + TOFF_ADC_CR1);

}

/* �����݃n���h����main�֐��Ŏg�p����O���[�o���ϐ��̐錾 */
volatile uint8_t adc_eoc_flag;	/* �����ݔ����ʒm�p�t���O */
volatile uint16_t ad_data;		/* AD�ϊ��l */

/* ADC�O���[�o�������� */
void ADC_IRQHandler(void)
{
	uint32_t temp;

	sys_printf("Interrupt occured!\n");

	/* �X�e�[�^�X���W�X�^���m�F���AADC1��EOC��1���m�F */
	temp = sil_rew_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_SR));
	if ((temp & ADC_SR_EOC) != 0) {
		/* IDR����f�[�^���擾 */
		ad_data = sil_reh_mem((uint16_t *)(TADR_ADC1_BASE + TOFF_ADC_DR));
		sys_printf("To set a flag\n");
		adc_eoc_flag = 1;	/* �����ݔ�����ʒm */
	}

	/* IDR����f�[�^���擾�����EOC�͎����ŃN���A */
	
}

void _main(void)
{
	uint16_t data;

	print_register("RCC_CR", TADR_RCC_BASE + TOFF_RCC_CR);
	print_register("RCC_CFGR", TADR_RCC_BASE + TOFF_RCC_CFGR);
	print_register("RCC_PLLCFGR", TADR_RCC_BASE + TOFF_RCC_PLLCFGR);
	print_register("RCC_AHB1ENR", TADR_RCC_BASE + TOFF_RCC_AHB1ENR);

	int_init();		/* �����ݐݒ� */
	print_register("NVIC_ISER0", TADR_NVIC_ISER);

	adc_init();		/* ADC�ݒ�Ɗ����݋��� */

	for(;;) {
		/* ADC start */
		sil_orw_mem((uint32_t *)(TADR_ADC1_BASE + TOFF_ADC_CR2), ADC_CR2_SWSTART);

		while (adc_eoc_flag == 0); /* EOC�����ݔ����҂� */

		data = ad_data;		/* �O���[�o���ϐ��̒l���R�s�[ */
		adc_eoc_flag = 0;	/* �����ݒʒm�t���O���N���A */

		/* �f�[�^���E�ɃA���C�����g����Ă���̂ŁA�s�v�ȃr�b�g���폜 */
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

