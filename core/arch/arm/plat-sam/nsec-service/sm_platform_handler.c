// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2021, Microchip
 */

#include <console.h>
#include <drivers/pm/sam/atmel_pm.h>
#include <drivers/scmi-msg.h>
#include <io.h>
#include <kernel/tee_misc.h>
#include <mm/core_memprot.h>
#include <sam_sfr.h>
#include <sam_pl310.h>
#include <sm/optee_smc.h>
#include <sm/sm.h>
#include <smc_ids.h>

static enum sm_handler_ret sam_sip_handler(struct thread_smc_args *args)
{
	/*
	 * As all sama5 SoCs are single-core ones, check the code compiled for a
	 * single core. No serializations done to protect against concurrency.
	 */
	static_assert(CFG_TEE_CORE_NB_CORE == 1);

	switch (OPTEE_SMC_FUNC_NUM(args->a0)) {
#ifdef CFG_PL310_SIP_PROTOCOL
	case SAM_SMC_SIP_PL310_ENABLE:
		args->a0 = pl310_enable();
		break;
	case SAM_SMC_SIP_PL310_DISABLE:
		args->a0 = pl310_disable();
		break;
	case SAM_SMC_SIP_PL310_EN_WRITEBACK:
		args->a0 = pl310_enable_writeback();
		break;
	case SAM_SMC_SIP_PL310_DIS_WRITEBACK:
		args->a0 = pl310_disable_writeback();
		break;
#endif
	case SAMA5_SMC_SIP_SFR_SET_USB_SUSPEND:
		atmel_sfr_set_usb_suspend(args->a1);
		args->a0 = SAMA5_SMC_SIP_RETURN_SUCCESS;
		break;
	case SAMA5_SMC_SIP_SET_SUSPEND_MODE:
		return at91_pm_set_suspend_mode(args);
	case SAMA5_SMC_SIP_GET_SUSPEND_MODE:
		return at91_pm_get_suspend_mode(args);
	case SAMA5_SMC_SIP_SCMI_CALL_ID:
		scmi_smt_fastcall_smc_entry(0);
		args->a0 = SAMA5_SMC_SIP_RETURN_SUCCESS;
		break;
	default:
		return SM_HANDLER_PENDING_SMC;
	}

	return SM_HANDLER_SMC_HANDLED;
}

enum sm_handler_ret sm_platform_handler(struct sm_ctx *ctx)
{
	uint32_t *nsec_r0 = (uint32_t *)(&ctx->nsec.r0);
	uint16_t smc_owner = OPTEE_SMC_OWNER_NUM(*nsec_r0);

	switch (smc_owner) {
	case OPTEE_SMC_OWNER_SIP:
		return sam_sip_handler((struct thread_smc_args *)nsec_r0);
	default:
		return SM_HANDLER_PENDING_SMC;
	}
}

