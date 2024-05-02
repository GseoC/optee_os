/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2024, STMicroelectronics
 */

#ifndef _DT_BINDINGS_SOC_STM32MP13_ETZPC_H
#define _DT_BINDINGS_SOC_STM32MP13_ETZPC_H

/*  define DECPROT modes */
#define DECPROT_S_RW		0x0
#define DECPROT_NS_R_S_W	0x1
#define DECPROT_NS_RW		0x3

/*  define DECPROT lock */
#define DECPROT_UNLOCK		0x0
#define DECPROT_LOCK		0x1

/* define TZMA IDs*/
#define ETZPC_TZMA0_ID			1000
#define ETZPC_TZMA1_ID			1001

/* define ETZPC ID */
#define STM32MP1_ETZPC_VREFBUF_ID	0
#define STM32MP1_ETZPC_LPTIM2_ID	1
#define STM32MP1_ETZPC_LPTIM3_ID	2
#define STM32MP1_ETZPC_LTDC_ID		3
#define STM32MP1_ETZPC_DCMIPP_ID	4
#define STM32MP1_ETZPC_USBPHYCTRL_ID	5
#define STM32MP1_ETZPC_DDRCTRLPHY_ID	6
/* 7-11 Reserved */
#define STM32MP1_ETZPC_IWDG1_ID		12
#define STM32MP1_ETZPC_STGENC_ID	13
/* 14-15 Reserved */
#define STM32MP1_ETZPC_USART1_ID	16
#define STM32MP1_ETZPC_USART2_ID	17
#define STM32MP1_ETZPC_SPI4_ID		18
#define STM32MP1_ETZPC_SPI5_ID		19
#define STM32MP1_ETZPC_I2C3_ID		20
#define STM32MP1_ETZPC_I2C4_ID		21
#define STM32MP1_ETZPC_I2C5_ID		22
#define STM32MP1_ETZPC_TIM12_ID		23
#define STM32MP1_ETZPC_TIM13_ID		24
#define STM32MP1_ETZPC_TIM14_ID		25
#define STM32MP1_ETZPC_TIM15_ID		26
#define STM32MP1_ETZPC_TIM16_ID		27
#define STM32MP1_ETZPC_TIM17_ID		28
/* 29-31 Reserved */
#define STM32MP1_ETZPC_ADC1_ID		32
#define STM32MP1_ETZPC_ADC2_ID		33
#define STM32MP1_ETZPC_OTG_ID		34
#define STM32MP1_ETZPC_TSC_ID		37
/* 38-39 Reserved */
#define STM32MP1_ETZPC_RNG_ID		40
#define STM32MP1_ETZPC_HASH_ID		41
#define STM32MP1_ETZPC_CRYP_ID		42
#define STM32MP1_ETZPC_SAES_ID		43
#define STM32MP1_ETZPC_PKA_ID		44
#define STM32MP1_ETZPC_BKPSRAM_ID	45
/* 46-47 Reserved */
#define STM32MP1_ETZPC_ETH1_ID		48
#define STM32MP1_ETZPC_ETH2_ID		49
#define STM32MP1_ETZPC_SDMMC1_ID	50
#define STM32MP1_ETZPC_SDMMC2_ID	51
/* 52 Reserved */
#define STM32MP1_ETZPC_MCE_ID		53
#define STM32MP1_ETZPC_FMC_ID		54
#define STM32MP1_ETZPC_QSPI_ID		55
/* 56-59 Reserved */
#define STM32MP1_ETZPC_SRAM1_ID		60
#define STM32MP1_ETZPC_SRAM2_ID		61
#define STM32MP1_ETZPC_SRAM3_ID		62
/* 63 Reserved */

#define STM32MP1_ETZPC_MAX_ID		64

#define DECPROT(id, mode, lock)		(((id) << 16) | ((mode) << 8) | (lock))

#endif /* _DT_BINDINGS_SOC_STM32MP13_ETZPC_H */
