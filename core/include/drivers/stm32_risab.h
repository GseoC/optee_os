/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2022-2024, STMicroelectronics
 */

#ifndef __DRIVERS_STM32_RISAB_H__
#define __DRIVERS_STM32_RISAB_H__

/*
 * stm32_risab_clear_illegal_access_flags() - Clears flags raised when an
 * illegal access occurs on a memory region
 */
void stm32_risab_clear_illegal_access_flags(void);

/*
 * stm32_risab_print_erroneous_data() - Prints the data associated to an illegal
 * access occurring on a memory protected by a RISAB : faulty address and
 * firewall attributes of the master causing the illegal access.
 */
#ifdef CFG_TEE_CORE_DEBUG
void stm32_risab_print_erroneous_data(void);
#else /* CFG_TEE_CORE_DEBUG */
static inline void stm32_risab_print_erroneous_data(void)
{
}
#endif /* CFG_TEE_CORE_DEBUG */

#endif /*__DRIVERS_STM32_RISAB_H__*/
