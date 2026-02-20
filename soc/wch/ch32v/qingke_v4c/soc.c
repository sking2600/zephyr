/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/toolchain.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/console/sdi_console.h>
#include <zephyr/dt-bindings/clock/ch32l103_clock.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/*
 * CH32L103 Power Management Implementation
 *
 * Based on vendor EVT examples:
 * - Sleep Mode: CPU sleeps, peripherals can run
 * - Stop Mode: CPU and most clocks stopped, RAM retained
 * - Standby Mode: Deepest sleep, RAM can be optionally retained
 */

/*
 * PWR Register Definitions for CH32L103
 * PWR Base: 0x40007000 (APB1 peripheral)
 */
#define PWR_BASE_ADDR			0x40007000U

/* PWR_CTLR bits */
#define PWR_CTLR_LPDS			BIT(0)  /* Low Power Deepsleep */
#define PWR_CTLR_PDDS			BIT(1)  /* Power Down Deepsleep */
#define PWR_CTLR_CWUF			BIT(2)  /* Clear Wakeup Flag */
#define PWR_CTLR_CSBF			BIT(3)  /* Clear Standby Flag */
#define PWR_CTLR_PVDEN			BIT(4)  /* PVD Enable */
#define PWR_CTLR PLS_Pos		5
#define PWR_CTLR_PLS_Msk		(0x7 << PWR_CTLR_PLS_Pos)
#define PWR_CTLR_FLP			BIT(9)   /* Flash Low Power mode */
#define PWR_CTLR_AUTO_LDO_LP		BIT(12) /* Auto LDO Low Power in Stop */
#define PWR_CTLR_LDO_LP		BIT(13) /* LDO Low Power */
/* RAM retention bits */
#define PWR_CTLR_RAM1			BIT(16) /* 2K RAM retention in standby */
#define PWR_CTLR_RAM2			BIT(17) /* 18K RAM retention in standby */
#define PWR_CTLR_RAM1_VBAT		BIT(18) /* 2K RAM retention via VBAT */
#define PWR_CTLR_RAM2_VBAT		BIT(19) /* 18K RAM retention via VBAT */
#define PWR_CTLR_RAM_LV		BIT(20) /* Low Voltage RAM retention */

/* PWR_CSR bits */
#define PWR_CSR_WUF			BIT(0)  /* Wakeup Flag */
#define PWR_CSR_SBF			BIT(1)  /* Standby Flag */
#define PWR_CSR_PVDO			BIT(2)  /* PVD Output */
#define PWR_CSR_EWUP			BIT(8)  /* Enable Wakeup Pin */

/* PWR Regulator definitions */
#define PWR_REGULATOR_ON		0x00000000U
#define PWR_REGULATOR_LOW_POWER	0x00000001U

/* Stop mode entry */
#define PWR_STOP_ENTRY_WFI		0x01U
#define PWR_STOP_ENTRY_WFE		0x02U

/* PWR Flag definitions */
#define PWR_FLAG_WU			0x00000001U
#define PWR_FLAG_SB			0x00000002U
#define PWR_FLAG_PVDO			0x00000004U

/* Inline PWR register access */
static inline volatile uint32_t *get_pwr_ctlr(void)
{
	return (volatile uint32_t *)(PWR_BASE_ADDR + 0x00);
}

static inline volatile uint32_t *get_pwr_csr(void)
{
	return (volatile uint32_t *)(PWR_BASE_ADDR + 0x04);
}

#undef PWR_CTLR
#define PWR_CTLR		(*get_pwr_ctlr())
#define PWR_CSR		(*get_pwr_csr())

/*
 * RCC Register Definitions for CH32L103
 * RCC Base: 0x40021000
 */
#define RCC_BASE_ADDR			0x40021000U

/* Inline RCC register access */
#define RCC		(*(volatile struct rcc_regs *)RCC_BASE_ADDR)

struct rcc_regs {
	volatile uint32_t CTLR;
	volatile uint32_t CFGR0;
	volatile uint32_t INTR;
	volatile uint32_t PB2PRSTR;
	volatile uint32_t PB1PRSTR;
	volatile uint32_t RESERVED1[2];
	volatile uint32_t PB2APB1RSTR;
	volatile uint32_t PB1APB2RSTR;
	volatile uint32_t RESERVED2[2];
	volatile uint32_t AHBPRSTR;
	volatile uint32_t RESERVED3[3];
	volatile uint32_t PB2PCENR;
	volatile uint32_t PB1PCENR;
	volatile uint32_t RESERVED4[2];
	volatile uint32_t HBPCENR;
	volatile uint32_t RESERVED5[3];
	volatile uint32_t HBPRSTR;
	volatile uint32_t RESERVED6[3];
	volatile uint32_t PB2PCENR1;
	volatile uint32_t RESERVED7[3];
	volatile uint32_t RESERVED8[0x30];
	volatile uint32_t RSTSCKR;
};

/* PWR clock enable bit */
#define RCC_PB1_PERIPH_PWR		BIT(28)

static inline void rcc_enable_pwr_clock(void)
{
	const struct device *rcc = DEVICE_DT_GET(DT_NODELABEL(rcc));

	/* Enable PWR clock via the RCC clock control driver */
	clock_control_on(rcc, (clock_control_subsys_t)RCC_APB1_PWR);
}

/* RCC reset status register (RSTSCKR) bit definitions */
#define RCC_RMVF		BIT(24) /* Remove reset flag */
#define RCC_PINRSTF		BIT(26) /* PIN reset flag */
#define RCC_PORRSTF		BIT(27) /* POR/PDR reset flag */
#define RCC_SFTRSTF		BIT(28) /* Software reset flag */
#define RCC_IWDGRSTF		BIT(29) /* Independent watchdog reset flag */
#define RCC_WWDGRSTF		BIT(30) /* Window watchdog reset flag */
#define RCC_LPWRRSTF		BIT(31) /* Low-power reset flag */

/* Power Management Hooks */

/* Save/restore context for sleep modes */
static volatile uint32_t saved_ahb_pcenr;
static volatile uint32_t saved_apb1_pcenr;
static volatile uint32_t saved_apb2_pcenr;

/* Track if we just woke up from stop mode (need clock reinit) */
static volatile bool stop_mode_wakeup;

/* External declaration for system clock reinit (provided by clock driver) */
extern void SystemCoreClockUpdate(void);


/*
 * Enter Stop Mode with Low Power regulator
 * Based on vendor EVT Stop_Mode example
 */
static void enter_stop_mode(uint32_t regulator)
{
	uint32_t tmpreg;

	rcc_enable_pwr_clock();

	tmpreg = PWR_CTLR;
	/* Clear DS bits, set low power mode if requested */
	tmpreg &= ~0x03;  /* Clear LPDS and PDDS */
	tmpreg |= regulator;

	/* If low power regulator, configure LDO low power */
	if (regulator == PWR_REGULATOR_LOW_POWER) {
		tmpreg &= ~(3 << 10);
		tmpreg |= (1 << 11);  /* LDO in low power mode */
	}
	PWR_CTLR = tmpreg;

	/* Clear wakeup flags */
	PWR_CTLR |= PWR_CTLR_CWUF;

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

	tmpreg = PWR_CTLR;

	/* Configure flash low power mode */
	tmpreg &= ~(3 << 10);
	tmpreg |= (1 << 11);

	/* Clear wakeup and standby flags */
	tmpreg |= PWR_CTLR_CWUF;
	tmpreg |= PWR_CTLR_CSBF;

	/* Set Power Down Deepsleep */
	tmpreg |= PWR_CTLR_PDDS;

	PWR_CTLR = tmpreg;

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

	tmpreg = PWR_CTLR;

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

	PWR_CTLR = tmpreg;

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
	return (PWR_CSR & PWR_CSR_SBF) != 0;
}

/*
 * Clear standby/wakeup flags
 */
static void clear_pwr_flags(void)
{
	PWR_CTLR |= (PWR_FLAG_WU << 2);  /* Clear wakeup flag */
	PWR_CTLR |= (PWR_FLAG_SB << 2);  /* Clear standby flag */
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
		saved_ahb_pcenr = RCC.HBPCENR;
		saved_apb1_pcenr = RCC.PB1PCENR;
		saved_apb2_pcenr = RCC.PB2PCENR;

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
		saved_ahb_pcenr = RCC.HBPCENR;
		saved_apb1_pcenr = RCC.PB1PCENR;
		saved_apb2_pcenr = RCC.PB2PCENR;

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
		RCC.HBPCENR = saved_ahb_pcenr;
		RCC.PB1PCENR = saved_apb1_pcenr;
		RCC.PB2PCENR = saved_apb2_pcenr;
		break;

	case PM_STATE_SUSPEND_TO_RAM:
		/*
		 * Stop mode exit - need to reinitialize clocks
		 * The EVT examples call SystemInit() after waking up
		 * from stop mode with low-power regulator
		 */
		if (stop_mode_wakeup) {
			/* Reinitialize system clocks */
			SystemCoreClockUpdate();
			stop_mode_wakeup = false;
		}

		/* Restore clock registers */
		RCC.HBPCENR = saved_ahb_pcenr;
		RCC.PB1PCENR = saved_apb1_pcenr;
		RCC.PB2PCENR = saved_apb2_pcenr;
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
	 * Interrupts will be re-enabled by the PM subsystem.
	 */
	irq_unlock(0);
}
