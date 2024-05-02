/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2018-2019, STMicroelectronics
 */

#ifndef __DRIVERS_STM32_ETZPC_H
#define __DRIVERS_STM32_ETZPC_H

#include <util.h>
#include <types_ext.h>

struct etzpc_device;

enum etzpc_decprot_attributes {
	ETZPC_DECPROT_S_RW = 0,
	ETZPC_DECPROT_NS_R_S_W = 1,
	ETZPC_DECPROT_MCU_ISOLATION = 2,
	ETZPC_DECPROT_NS_RW = 3,
	ETZPC_DECPROT_MAX = 4,
};

struct etzpc_device *stm32_get_etzpc_device(void);

void etzpc_do_configure_tzma(struct etzpc_device *etzpc_dev, uint32_t tzma_id,
			     uint16_t tzma_value);

#endif /*__DRIVERS_STM32_ETZPC_H*/
