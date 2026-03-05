/*
 * Copyright (c) 2026 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief CH32L103 Power Management Implementation
 *
 * This file provides Power Management support for the CH32L103 which has
 * different PM capabilities than other WCH RISC-V variants.
 *
 * The CH32L103 supports:
 * - Sleep Mode: CPU sleeps, peripherals can run
 * - Stop Mode: CPU and most clocks stopped, RAM retained
 * - Standby Mode: Deepest sleep, RAM can be optionally retained
 *
 * IMPORTANT: Before entering Standby Mode, the application must configure
 * a wakeup source (WKUP pin, RTC alarm, etc.) via the PWR_CSR_EWUP bits.
 * Without a wakeup source configured, the device will remain in standby
 * until a reset occurs.
 *
 * Based on vendor EVT examples.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ch32l103_clock.h>
#include <zephyr/logging/log.h>
#include <hal_ch32fun.h>

#include "power_ch32l103.h"

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/* SystemCoreClock is typically defined in the HAL/ch32fun library */
extern uint32_t SystemCoreClock;

/* PWR register definitions are defined in HAL ch32l103hw.h
 * PWR_TypeDef *PWR provides register access via PWR->CTLR and PWR->CSR
 * Bit definitions like PWR_CTLR_CWUF, PWR_CTLR_CSBF, PWR_CTLR_PDDS are in HAL
 */

/* Flash low power mode bit position and mask - CH32L103 specific */
#define PWR_CTLR_FLASH_LP_Pos 9
#define PWR_CTLR_FLASH_LP_Msk (0x7U << PWR_CTLR_FLASH_LP_Pos)

static inline void flash_enter_low_power(void)
{
	/* Set flash low power mode bits (111b at bits 9-11) */
	PWR->CTLR |= PWR_CTLR_FLASH_LP_Msk;
}

static inline void flash_exit_low_power(void)
{
	/* Clear flash low power mode bit 9 (keep bits 10-11 as needed) */
	PWR->CTLR &= ~(1U << PWR_CTLR_FLASH_LP_Pos);
}

static inline int rcc_enable_pwr_clock(void)
{
	const struct device *rcc = DEVICE_DT_GET(DT_NODELABEL(rcc));

	if (!device_is_ready(rcc)) {
		LOG_ERR("RCC device not ready");
		return -ENODEV;
	}

	/* Enable PWR clock via the RCC clock control driver */
	return clock_control_on(rcc, (clock_control_subsys_t)RCC_APB1_PWR);
}

/* Power Management Hooks */

/* Save/restore context for sleep modes */
static volatile uint32_t saved_ahb_pcenr;
static volatile uint32_t saved_apb1_pcenr;
static volatile uint32_t saved_apb2_pcenr;

/* Track if we just woke up from stop mode (need clock reinit) */
static volatile bool stop_mode_wakeup;

/*
 * Enter Stop Mode with Low Power regulator
 * Based on vendor EVT Stop_Mode example
 */
static void enter_stop_mode(uint32_t regulator)
{
	uint32_t tmpreg;

	rcc_enable_pwr_clock();

	/* Enter flash low power mode for additional power savings */
	flash_enter_low_power();

	tmpreg = PWR->CTLR;
	/* Clear DS bits, set low power mode if requested */
	tmpreg &= ~0x03; /* Clear LPDS and PDDS */
	tmpreg |= regulator;

	/* If low power regulator, configure LDO low power */
	if (regulator == PWR_REGULATOR_LOW_POWER) {
		tmpreg &= ~(3 << 10);
		tmpreg |= (1 << 11); /* LDO in low power mode */
	}
	PWR->CTLR = tmpreg;

	/* Clear wakeup flags */
	PWR->CTLR |= PWR_CTLR_CWUF;

	/* Disable interrupts briefly for WFI */
	__asm__ volatile("csrci mstatus, 0x8");

	/* Enter stop mode via WFI */
	__asm__ volatile("wfi");

	/* Re-enable interrupts */
	__asm__ volatile("csrsi mstatus, 0x8");

	/* Mark that we woke up from stop mode */
	stop_mode_wakeup = true;
}

/*
 * Enter Standby Mode
 * Based on vendor EVT Standby_Mode example
 */
static void enter_standby_mode(void)
{
	uint32_t tmpreg;

	rcc_enable_pwr_clock();

	tmpreg = PWR->CTLR;

	/* Configure flash low power mode */
	tmpreg &= ~(3 << 10);
	tmpreg |= (1 << 11);

	/* Clear wakeup and standby flags */
	tmpreg |= PWR_CTLR_CWUF;
	tmpreg |= PWR_CTLR_CSBF;

	/* Set Power Down Deepsleep */
	tmpreg |= PWR_CTLR_PDDS;

	PWR->CTLR = tmpreg;

	/* Disable interrupts for WFI */
	__asm__ volatile("csrci mstatus, 0x8");

	/* Enter standby - this will cause a reset on wakeup */
	__asm__ volatile("wfi");

	/* We should never reach here due to reset on wakeup */
}

/*
 * Enter Standby Mode with RAM Retention
 * Based on vendor EVT Standby_RAM_Mode example
 */
static void enter_standby_mode_ram(void)
{
	uint32_t tmpreg;

	rcc_enable_pwr_clock();

	tmpreg = PWR->CTLR;

	/* Configure flash low power mode */
	tmpreg &= ~(3 << 10);
	tmpreg |= (1 << 11);

	/* Clear wakeup and standby flags */
	tmpreg |= PWR_CTLR_CWUF;
	tmpreg |= PWR_CTLR_CSBF;

	/* Set Power Down Deepsleep */
	tmpreg |= PWR_CTLR_PDDS;

	/* Enable 2K + 18K RAM retention (bits 16, 17) */
	tmpreg |= (BIT(16) | BIT(17));

	PWR->CTLR = tmpreg;

	/* Disable interrupts for WFI */
	__asm__ volatile("csrci mstatus, 0x8");

	/* Enter standby with RAM - continues execution after wakeup */
	__asm__ volatile("wfi");

	/* Re-enable interrupts */
	__asm__ volatile("csrsi mstatus, 0x8");
}

/*
 * Check if wakeup was from standby
 */
static bool is_wakeup_from_standby(void)
{
	rcc_enable_pwr_clock();
	return (PWR->CSR & PWR_CSR_SBF) != 0;
}

/*
 * Clear standby/wakeup flags
 * Note: PWR_CTLR_CWUF (bit 2) and PWR_CTLR_CSBF (bit 3) are write-1-to-clear
 */
static void clear_pwr_flags(void)
{
	PWR->CTLR |= PWR_CTLR_CWUF; /* Clear wakeup flag */
	PWR->CTLR |= PWR_CTLR_CSBF; /* Clear standby flag */
}

/*
 * Update SystemCoreClock after wake from stop mode
 * This reads the current clock configuration and updates the global
 */
void SystemCoreClockUpdate(void)
{
	uint32_t tmp;
	uint32_t pllmul = 0;
	uint32_t presc;

	/* Get system clock source */
	tmp = RCC->CFGR0 & RCC_SW;

	if (tmp == RCC_SW_HSI) {
		/* HSI used as system clock source */
		SystemCoreClock = HSI_VALUE;
	} else if (tmp == RCC_SW_HSE) {
		/* HSE used as system clock source */
		SystemCoreClock = HSE_VALUE;
	} else if (tmp == RCC_SW_PLL) {
		/* PLL used as system clock source */
		/* Get PLL source */
		if (RCC->CFGR0 & RCC_PLLSRC) {
			/* HSE used as PLL source */
			tmp = HSE_VALUE;
		} else {
			/* HSI used as PLL source */
			tmp = HSI_VALUE;
		}

		/* Get PLL multiplier - CH32L103 uses 5 bits (bit 22 + bits 18-21) */
		pllmul = (RCC->CFGR0 >> 18) & 0x0F;
		pllmul |= ((RCC->CFGR0 >> (22 - 4)) & 0x10);

		/* PLL multiplier is index + 2 for CH32L103 */
		pllmul += 2;
		SystemCoreClock = tmp * pllmul;
	} else {
		/* Unknown clock source - default to HSI */
		SystemCoreClock = HSI_VALUE;
	}

	/* Get AHB prescaler */
	tmp = RCC->CFGR0 & RCC_HPRE;
	if (tmp & RCC_HPRE_3) {
		/* prescaler is 2^((tmp >> 4) + 1) */
		presc = 1 << (((tmp >> 4) & 0x7) + 1);
		SystemCoreClock >>= presc;
	}
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	stop_mode_wakeup = false;

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/*
		 * Sleep mode: WFI puts the CPU in sleep until interrupt
		 * This is the lightest sleep mode - CPU core sleeps but
		 * clocks continue running
		 */
		LOG_DBG("Entering Sleep mode (SUSPEND_TO_IDLE)");

		/* Save clock states before sleep */
		saved_ahb_pcenr = RCC->HBPCENR;
		saved_apb1_pcenr = RCC->PB1PCENR;
		saved_apb2_pcenr = RCC->PB2PCENR;

		/* Enter sleep mode via WFI */
		__asm__ volatile("wfi");
		break;

	case PM_STATE_SUSPEND_TO_RAM:
		/*
		 * Stop mode: CPU and most clocks stopped, RAM retained
		 * Uses low-power regulator for maximum power savings
		 * Wakeup requires clock reinitialization
		 */
		LOG_DBG("Entering Stop mode (SUSPEND_TO_RAM)");

		/* Save clock states before stop */
		saved_ahb_pcenr = RCC->HBPCENR;
		saved_apb1_pcenr = RCC->PB1PCENR;
		saved_apb2_pcenr = RCC->PB2PCENR;

		/* Enter stop mode with low power regulator */
		enter_stop_mode(PWR_REGULATOR_LOW_POWER);
		break;

	case PM_STATE_SUSPEND_TO_DISK:
		/*
		 * Standby mode: Deepest sleep
		 * substate_id can specify:
		 *   0: Standby (full reset on wakeup)
		 *   1: Standby with RAM retention (continues execution)
		 */
		LOG_DBG("Entering Standby mode (SUSPEND_TO_DISK), substate %u", substate_id);

		if (substate_id == 1) {
			/* Standby with RAM retention */
			enter_standby_mode_ram();
		} else {
			/* Regular standby - full reset on wakeup */
			enter_standby_mode();
		}
		break;

	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/*
		 * Sleep mode exit - restore clocks
		 * Note: clocks should be intact, but restore anyway
		 */
		RCC->HBPCENR = saved_ahb_pcenr;
		RCC->PB1PCENR = saved_apb1_pcenr;
		RCC->PB2PCENR = saved_apb2_pcenr;
		break;

	case PM_STATE_SUSPEND_TO_RAM:
		/*
		 * Stop mode exit - restore flash and clocks
		 */
		if (stop_mode_wakeup) {
			/* Exit flash low power mode */
			flash_exit_low_power();

			/* Update SystemCoreClock variable based on current RCC config */
			SystemCoreClockUpdate();
			stop_mode_wakeup = false;
		}

		/* Restore clock registers */
		RCC->HBPCENR = saved_ahb_pcenr;
		RCC->PB1PCENR = saved_apb1_pcenr;
		RCC->PB2PCENR = saved_apb2_pcenr;
		break;

	case PM_STATE_SUSPEND_TO_DISK:
		/*
		 * Standby mode exit
		 * If we got here with RAM retention, clear flags
		 * (otherwise we reset and don't reach here)
		 */
		if (is_wakeup_from_standby()) {
			clear_pwr_flags();
		}
		break;

	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	/*
	 * System is now ready to continue normal operation.
	 * Interrupts remain disabled - the PM subsystem will re-enable them
	 * at the appropriate time.
	 */
}
