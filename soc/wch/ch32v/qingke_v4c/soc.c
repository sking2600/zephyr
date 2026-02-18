/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/arch/riscv/irq.h>
#include "ch32l103.h"

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

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/* WFI puts the CPU in sleep mode until an interrupt occurs */
		arch_cpu_idle();
		break;
	case PM_STATE_STANDBY:
		/* Enter Standby mode */
		/* Set SLEEPDEEP bit in System Control Register (handled by arch if configured?)
		 * For now, only WFI is safe without more complex context save/restore
		 */
		 __asm__ volatile("wfi");
		break;
	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}
