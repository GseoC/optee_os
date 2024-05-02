incdirs-$(CFG_VERSAL_SHA3_384) += crypto/versal/include

srcs-$(CFG_CDNS_UART) += cdns_uart.c
srcs-$(CFG_PL011) += pl011.c
srcs-$(CFG_TZC400) += tzc400.c
srcs-$(CFG_TZC380) += tzc380.c
srcs-$(CFG_GIC) += gic.c
srcs-$(CFG_CORE_HAFNIUM_INTC) += hfic.c
srcs-$(CFG_PL061) += pl061_gpio.c
srcs-$(CFG_PL022) += pl022_spi.c
srcs-$(CFG_SP805_WDT) += sp805_wdt.c
srcs-$(CFG_8250_UART) += serial8250_uart.c
srcs-$(CFG_16550_UART) += ns16550.c
srcs-$(CFG_IMX_SNVS) += imx_snvs.c
srcs-$(CFG_IMX_UART) += imx_uart.c
srcs-$(CFG_IMX_I2C) += imx_i2c.c
srcs-$(CFG_IMX_LPUART) += imx_lpuart.c
srcs-$(CFG_IMX_WDOG) += imx_wdog.c
srcs-$(CFG_SPRD_UART) += sprd_uart.c
srcs-$(CFG_HI16XX_UART) += hi16xx_uart.c
srcs-$(CFG_HI16XX_RNG) += hi16xx_rng.c
srcs-$(CFG_LPC_UART) += lpc_uart.c
srcs-$(CFG_SCIF) += scif.c
srcs-$(CFG_DRA7_RNG) += dra7_rng.c
srcs-$(CFG_STIH_UART) += stih_asc.c
srcs-$(CFG_ATMEL_UART) += atmel_uart.c
srcs-$(CFG_ATMEL_TRNG) += atmel_trng.c
srcs-$(CFG_ATMEL_RSTC) += atmel_rstc.c
srcs-$(CFG_ATMEL_SHDWC) += atmel_shdwc.c atmel_shdwc_a32.S
srcs-$(CFG_ATMEL_SAIC) += atmel_saic.c
srcs-$(CFG_ATMEL_WDT) += atmel_wdt.c
srcs-$(CFG_ATMEL_RTC) += atmel_rtc.c
srcs-$(CFG_ATMEL_PIOBU) += atmel_piobu.c
srcs-$(CFG_ATMEL_TCB) += atmel_tcb.c
srcs-$(CFG_AMLOGIC_UART) += amlogic_uart.c
srcs-$(CFG_MVEBU_UART) += mvebu_uart.c
srcs-$(CFG_STM32_BSEC) += stm32_bsec.c
srcs-$(CFG_STM32_ETZPC) += stm32_etzpc.c
srcs-$(CFG_STM32_FMC) += stm32_fmc.c
srcs-$(CFG_STM32_GPIO) += stm32_gpio.c
srcs-$(CFG_STM32_HPDMA) += stm32_hpdma.c
srcs-$(CFG_STM32_HSEM) += stm32_hsem.c
srcs-$(CFG_STM32_IWDG) += stm32_iwdg.c
srcs-$(CFG_STM32_IPCC) += stm32_ipcc.c
srcs-$(CFG_STM32_I2C) += stm32_i2c.c
srcs-$(CFG_STM32_RNG) += stm32_rng.c
srcs-$(CFG_STM32_SHARED_IO) += stm32_shared_io.c
srcs-$(CFG_STM32_TAMP) += stm32_tamp.c
srcs-$(CFG_STM32_UART) += stm32_uart.c
srcs-$(CFG_STPMIC1) += stpmic1.c
srcs-$(CFG_BCM_HWRNG) += bcm_hwrng.c
srcs-$(CFG_BCM_SOTP) += bcm_sotp.c
srcs-$(CFG_BCM_GPIO) += bcm_gpio.c
srcs-$(CFG_LS_I2C) += ls_i2c.c
srcs-$(CFG_LS_GPIO) += ls_gpio.c
srcs-$(CFG_LS_DSPI) += ls_dspi.c
srcs-$(CFG_LS_SEC_MON) += ls_sec_mon.c
srcs-$(CFG_LS_SFP) += ls_sfp.c
srcs-$(CFG_IMX_RNGB) += imx_rngb.c
srcs-$(CFG_IMX_OCOTP) += imx_ocotp.c
srcs-$(CFG_IMX_CAAM) += imx_caam.c
srcs-$(CFG_IMX_SCU) += imx_scu.c
srcs-$(CFG_IMX_CSU) += imx_csu.c
srcs-$(CFG_XIPHERA_TRNG) += xiphera_trng.c
srcs-$(CFG_IMX_SC) += imx_sc_api.c
srcs-$(CFG_IMX_ELE) += imx_ele.c
srcs-$(CFG_ZYNQMP_CSU_PUF) += zynqmp_csu_puf.c
srcs-$(CFG_ZYNQMP_CSUDMA) += zynqmp_csudma.c
srcs-$(CFG_ZYNQMP_CSU_AES) += zynqmp_csu_aes.c
srcs-$(CFG_ZYNQMP_PM) += zynqmp_pm.c
srcs-$(CFG_ZYNQMP_HUK) += zynqmp_huk.c
srcs-$(CFG_ARM_SMCCC_TRNG) += smccc_trng.c
srcs-$(CFG_VERSAL_GPIO) += versal_gpio.c
srcs-$(CFG_VERSAL_MBOX) += versal_mbox.c
srcs-$(CFG_VERSAL_PM) += versal_pm.c
srcs-$(CFG_STM32MP15_HUK) += stm32mp15_huk.c
srcs-$(CFG_VERSAL_RNG_DRV) += versal_trng.c
srcs-$(CFG_VERSAL_NVM) += versal_nvm.c
srcs-$(CFG_VERSAL_SHA3_384) += versal_sha3_384.c
srcs-$(CFG_VERSAL_PUF) += versal_puf.c
srcs-$(CFG_VERSAL_HUK) += versal_huk.c
srcs-$(CFG_CBMEM_CONSOLE) += cbmem_console.c
srcs-$(CFG_RISCV_PLIC) += plic.c
srcs-$(CFG_RISCV_ZKR_RNG) += riscv_zkr_rng.c
srcs-$(CFG_HISILICON_CRYPTO_DRIVER) += hisi_trng.c
srcs-$(CFG_WIDEVINE_HUK) += widevine_huk.c
srcs-$(CFG_SEMIHOSTING_CONSOLE) += semihosting_console.c

subdirs-y += crypto
subdirs-$(CFG_BNXT_FW) += bnxt
subdirs-$(CFG_DRIVERS_CLK) += clk
subdirs-$(CFG_DRIVERS_FIREWALL) += firewall
subdirs-$(CFG_DRIVERS_GPIO) += gpio
subdirs-$(CFG_DRIVERS_I2C) += i2c
subdirs-$(CFG_DRIVERS_NVMEM) += nvmem
subdirs-$(CFG_DRIVERS_PINCTRL) += pinctrl
subdirs-$(CFG_DRIVERS_REGULATOR) += regulator
subdirs-$(CFG_DRIVERS_RSTCTRL) += rstctrl
subdirs-$(CFG_DRIVERS_REMOTEPROC) += remoteproc
subdirs-$(CFG_SCMI_MSG_DRIVERS) += scmi-msg
subdirs-y += imx
subdirs-y += pm
subdirs-y += wdt
subdirs-y += rtc
subdirs-y += firewall
