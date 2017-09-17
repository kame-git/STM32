
#ifndef _SYS_DEFS_H_
#define _SYS_DEFS_H_

/*	Values	*/

#define TARGET_CPU_STR		"STM32F4-Discovery"

/* endian */
#define SIL_ENDIAN      0		/* LITTLE ENDIAN */

/*	Address		*/
#define RAM_BASE        0x20000000
#define RAM_SIZE        (128*1024)
#define VECTOR_SIZE		(4*8)
#define FIQ_DATA_SIZE   256
#define STACKINT        (RAM_BASE+RAM_SIZE-FIQ_DATA_SIZE)	/* �����ߥ����å��ν���� */
#define STACKTOP        (STACKINT-256)						/* �桼���������å��ν���� */

/*
 *  ���������Ԥ��Τ�������
 *  �������ϥ���å����ȿ��˰�¸���롣
 */
#define SIL_DLY_TIM1    200
#if ROM_EXEC == 1
#define SIL_DLY_TIM2    24
#else
#define SIL_DLY_TIM2    54
#endif

/*	Default Values	*/

#define MAX_INT_NUM     82
#define DEF_DUMP_LENGTH	256
#define DEF_MON_SERIAL	0
#define NOT_INITDECT
#define NOINT_UARTTX
/* #define SYSTEMCORECLOCK 168000000 */
#define SYSTEMCORECLOCK 180000000

#ifndef TOPPERS_MACRO_ONLY
/*
 *  ����ѥ����¸�Υǡ����������
 */
#define	_int8_		char		/* 8�ӥåȤ�������	*/
#define	_int16_		short		/* 16�ӥåȤ�������	*/
#define	_int32_		int			/* 32�ӥåȤ�������	*/

/*
 *  ����ѥ���γ�ĥ��ǽ�Τ���Υޥ������
 */
#define	Inline          static inline

extern void sil_dly_nse(unsigned int);

#endif	/* TOPPERS_MACRO_ONLY */

/*	Serial Channels		*/

#define MAX_SERIAL		2

#include "stm32f4xx.h"

/* serial definitions */

#define DEFAULT_SPEED		115200

#ifndef TOPPERS_MACRO_ONLY

#if DEBUG == 0
#define	sys_puthex(val)
#define sys_printf(s)
#else
extern void sys_puthex(unsigned int val);
extern void sys_printf(const char *s);
#endif


#endif	/* TOPPERS_MACRO_ONLY */

#endif	/* SYS_DEFS_H_ */

