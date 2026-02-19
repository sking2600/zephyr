/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/drivers/clock_control.h>
#include "ch32l103.h"

/* PWR peripheral base address (APB1 + 0x7000) */
#define PWR_BASE        0x40007000UL

/* PWR register layout */
typedef struct {
	volatile uint32_t CTLR;
	volatile uint32_t CSR;
} PWR_TypeDef;

#define PWR             ((PWR_TypeDef *)PWR_BASE)

/* PWR_CTLR bit definitions */
#define PWR_CTLR_PDDS   BIT(1)  /* Power Down Deepsleep: 0=stop, 1=standby */
#define PWR_CTLR_CWUF   BIT(2)  /* Clear Wakeup Flag */
#define PWR_CTLR_CSBF   BIT(3)  /* Clear Standby Flag */

/* PWR_CSR bit definitions */
#define PWR_CSR_WUF     BIT(0)  /* Wakeup Flag */
#define PWR_CSR_SBF     BIT(1)  /* Standby Flag - set when waking from standby */

/* RCC reset status register (RSTSCKR) bit definitions */
#define RCC_RMVF        BIT(24) /* Remove reset flag */
#define RCC_PINRSTF     BIT(26) /* PIN reset flag */
#define RCC_PORRSTF     BIT(27) /* POR/PDR reset flag */
#define RCC_SFTRSTF     BIT(28) /* Software reset flag */
#define RCC_IWDGRSTF    BIT(29) /* Independent watchdog reset flag */
#define RCC_WWDGRSTF    BIT(30) /* Window watchdog reset flag */
#define RCC_LPWRRSTF    BIT(31) /* Low-power reset flag */

#if defined(CONFIG_SOC_CH32L103)
static void clock_init(void)
{
	/* HCLK = SYSCLK / 1 */
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;

	/* Enable DMA, AFIO and GPIO clocks */
	RCC->HBPCENR |= BIT(0); /* DMAEN */
	RCC->PB2PCENR |= BIT(0) | BIT(2) | BIT(3) | BIT(4) | BIT(5); /* AFIO, GPIOA-D */
}

static int wch_ch32l103_init(void)
{
    clock_init();
    return 0;
}

SYS_INIT(wch_ch32l103_init, PRE_KERNEL_1, 0);
#endif

/* Power Management Hooks */

/* Save/restore context for sleep modes */
static volatile uint32_t saved_ahb_pcenr;
static volatile uint32_t saved_apb1_pcenr;
static volatile uint32_t saved_apb2_pcenr;

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/* Sleep mode: WFI puts the CPU in sleep until interrupt */
		/* Save clock states before sleep */
		saved_ahb_pcenr = RCC->HBPCENR;
		saved_apb1_pcenr = RCC->APB1PCENR;
		saved_apb2_pcenr = RCC->PB2PCENR;

		/* Enter sleep mode via WFI */
		arch_cpu_idle();
		break;

	case PM_STATE_STANDBY:
		/*
		 * Standby mode: sets PDDS bit in PWR_CTLR so that the next
		 * WFI/WFE enters standby (power-down deepsleep) rather than
		 * stop mode.  On CH32L103 the MCU resets on wakeup from true
		 * standby, so execution resumes from the reset vector.
		 *
		 * We save the clock register state so that pm_state_exit_post_ops
		 * can detect whether we returned via a normal interrupt (stop-mode
		 * fallback) or via a full reset (standby).  In the reset path the
		 * saved values are never used because the reset handler runs first.
		 */

		/* Save critical clock registers (used only if standby is not entered) */
		saved_ahb_pcenr = RCC->HBPCENR;
		saved_apb1_pcenr = RCC->APB1PCENR;
		saved_apb2_pcenr = RCC->PB2PCENR;

		/* Clear any pending wakeup / standby flags */
		PWR->CTLR |= PWR_CTLR_CWUF | PWR_CTLR_CSBF;

		/* Select standby (power-down deepsleep) mode */
		PWR->CTLR |= PWR_CTLR_PDDS;

		/* Enter standby - MCU will reset on wakeup */
		__asm__ volatile("wfi");

		/*
		 * If we reach here the WFI returned without a full reset
		 * (e.g. an interrupt fired before standby was entered).
		 * Clear PDDS so subsequent sleeps use stop mode by default.
		 */
		PWR->CTLR &= ~PWR_CTLR_PDDS;
		break;

	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/* Restore clock registers that might have been affected */
		/* LPTIM and other wakeup sources should still be enabled */

		/* Re-enable critical clocks if they were disabled */
		if ((RCC->HBPCENR & BIT(0)) == 0) {
			RCC->HBPCENR |= BIT(0); /* DMA */
		}

		/* Restore full clock states */
		RCC->HBPCENR = saved_ahb_pcenr;
		RCC->APB1PCENR = saved_apb1_pcenr;
		RCC->PB2PCENR = saved_apb2_pcenr;
		break;

	case PM_STATE_STANDBY:
		/*
		 * Post-standby exit.
		 *
		 * If the MCU entered true standby it resets on wakeup and
		 * execution never reaches this point - the reset handler
		 * re-initialises everything from scratch.
		 *
		 * If we reach here it means standby was not entered (an
		 * interrupt fired before the WFI could commit to standby).
		 * In that case the saved clock registers are still valid and
		 * we restore them to undo any partial clock gating.
		 *
		 * We can distinguish the two cases by checking PWR_CSR_SBF:
		 * the standby flag is set by hardware when the MCU wakes from
		 * standby.  If it is clear we know standby was not entered.
		 */
		if (PWR->CSR & PWR_CSR_SBF) {
			/*
			 * Standby was entered and the MCU reset on wakeup.
			 * This path should not normally be reached because the
			 * reset vector runs first.  Clear the flag and return.
			 */
			PWR->CTLR |= PWR_CTLR_CSBF;
		} else {
			/*
			 * Standby was not entered (interrupt fired first).
			 * Restore clock registers to their pre-sleep state.
			 */
			RCC->HBPCENR = saved_ahb_pcenr;
			RCC->APB1PCENR = saved_apb1_pcenr;
			RCC->PB2PCENR = saved_apb2_pcenr;
		}
		break;

	default:
		break;
	}

	/*
	 * System is now ready to continue normal operation.
	 * Interrupts will be re-enabled by the PM subsystem.
	 */
}
