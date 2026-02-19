/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_lptim

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/pm/pm.h>
#include <hal_ch32fun.h>

#define LPTIM ((LPTIM_TypeDef *)DT_INST_REG_ADDR(0))

/* LPTIM register bit definitions */
#define LPTIM_CR_ENABLE     BIT(0)
#define LPTIM_CR_SNGSTRT    BIT(1)
#define LPTIM_CR_CNTSTRT    BIT(2)
#define LPTIM_CR_COUNTRST   BIT(3)
#define LPTIM_CR_RSTARE     BIT(4)

#define LPTIM_IER_CMPMIE    BIT(0)
#define LPTIM_IER_ARRMIE    BIT(1)
#define LPTIM_IER_EXTTRIGIE BIT(2)
#define LPTIM_IER_CMPOKIE   BIT(3)
#define LPTIM_IER_ARROKIE   BIT(4)
#define LPTIM_IER_UPOKIE    BIT(5)

#define LPTIM_ISR_CMPM      BIT(0)
#define LPTIM_ISR_ARRM      BIT(1)
#define LPTIM_ISR_EXTTRIG   BIT(2)
#define LPTIM_ISR_CMPOK     BIT(3)
#define LPTIM_ISR_ARROK     BIT(4)
#define LPTIM_ISR_UPOK      BIT(5)

#define LPTIM_ICR_CMPMCF    BIT(0)
#define LPTIM_ICR_ARRMCF    BIT(1)
#define LPTIM_ICR_EXTTRIGCF BIT(2)
#define LPTIM_ICR_CMPOKCF   BIT(3)
#define LPTIM_ICR_ARROKCF   BIT(4)
#define LPTIM_ICR_UPOKCF    BIT(5)

#define LPTIM_CFGR_TIMOUT   BIT(19)

/* Clock source selection bits (25-26) */
#define LPTIM_CFGR_CLKSEL_Pos   25
#define LPTIM_CFGR_CLKSEL_Msk   (0x3U << LPTIM_CFGR_CLKSEL_Pos)

/* Prescaler bits (9-11) */
#define LPTIM_CFGR_PRESC_Pos    9
#define LPTIM_CFGR_PRESC_Msk    (0x7U << LPTIM_CFGR_PRESC_Pos)

/* Minimum nb of clock cycles to set autoreload register correctly.
 * Configurable via CONFIG_WCH_LPTIM_GUARD_VALUE (default 5).
 */
#define LPTIM_GUARD_VALUE   CONFIG_WCH_LPTIM_GUARD_VALUE

/* LPTIM has 16-bit counter */
#define LPTIM_MAX_VALUE     0xFFFFU

static struct k_spinlock lock;
static uint32_t accumulated_cycles;
static uint32_t last_announcement_cycles;
static uint32_t lptim_clock_freq;
static uint32_t lptim_time_base;

/* Prescaler from DT (1, 2, 4, 8, 16, 32, 64, 128) */
static const uint32_t lptim_clock_presc = DT_INST_PROP_OR(0, wch_prescaler, 1);

/* Clock source from DT (0=PCLK, 1=HSI, 2=LSE, 3=LSI) */
static const uint32_t lptim_clock_source = DT_INST_PROP_OR(0, wch_clock_source, 3);

static const struct device *const clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));

/**
 * @brief Get current LPTIM counter value with synchronization
 *
 * Reads the counter value twice to ensure a stable read, handling
 * potential wraparound.
 */
static uint32_t lptim_get_counter(void)
{
	uint32_t cnt;
	uint32_t prev;

	/* Double read for sync */
	cnt = LPTIM->CNT;
	do {
		prev = cnt;
		cnt = LPTIM->CNT;
	} while (cnt != prev);

	return cnt;
}

/**
 * @brief Check if ARRM (Auto-Reload Match) flag is active
 */
static inline bool lptim_is_arrm_active(void)
{
	return (LPTIM->ISR & LPTIM_ISR_ARRM) != 0;
}

/**
 * @brief Set autoreload value with proper synchronization
 *
 * Waits for ARROK flag after setting ARR to ensure the update
 * was synchronized by hardware.
 */
static int lptim_set_autoreload(uint32_t value)
{
	/* Clear ARROK flag */
	LPTIM->ICR |= LPTIM_ICR_ARROKCF;

	/* Set new ARR value (16-bit) */
	LPTIM->ARR = (uint16_t)(value & LPTIM_MAX_VALUE);

	/* Wait for ARROK with timeout */
	uint32_t timeout = 100000;
	while (!(LPTIM->ISR & LPTIM_ISR_ARROK) && timeout--) {
		/* Wait for sync - yield to allow other threads */
		k_yield();
	}

	if (timeout == 0) {
		return -ETIMEDOUT;
	}

	/* Clear ARROK flag */
	LPTIM->ICR |= LPTIM_ICR_ARROKCF;

	return 0;
}

static void lptim_irq_handler(const void *unused)
{
	ARG_UNUSED(unused);

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t isr = LPTIM->ISR;

	if (isr & LPTIM_ISR_ARRM) {
		/* Clear the flag */
		LPTIM->ICR |= LPTIM_ICR_ARRMCF;

		/* Add ARR+1 to accumulated cycles (counter resets after match) */
		accumulated_cycles += (LPTIM->ARR + 1);

		/* Calculate elapsed ticks */
		uint32_t elapsed_cycles = accumulated_cycles - last_announcement_cycles;
		uint32_t cycles_per_tick = lptim_clock_freq / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		uint32_t elapsed_ticks = elapsed_cycles / cycles_per_tick;

		if (elapsed_ticks > 0) {
			last_announcement_cycles += elapsed_ticks * cycles_per_tick;
			k_spin_unlock(&lock, key);
			sys_clock_announce(elapsed_ticks);
			return;
		}
	}

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t cnt = lptim_get_counter();
	uint32_t total = accumulated_cycles + cnt;

	k_spin_unlock(&lock, key);
	return total;
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t cnt = lptim_get_counter();

	/* Check if ARRM is active (counter has wrapped but IRQ not processed yet) */
	if (lptim_is_arrm_active()) {
		cnt = LPTIM->ARR + 1;
		cnt += lptim_get_counter();
	}

	uint32_t total_cycles = accumulated_cycles + cnt;
	uint32_t elapsed_cycles = total_cycles - last_announcement_cycles;
	uint32_t cycles_per_tick = lptim_clock_freq / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	k_spin_unlock(&lock, key);

	return elapsed_cycles / cycles_per_tick;
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	/*
	 * When CONFIG_SYSTEM_CLOCK_SLOPPY_IDLE = y, ticks equals to -1
	 * is treated as LPTIM off (never waking up, LPTIM not clocked)
	 */
	if (ticks == K_TICKS_FOREVER) {
		/* Disable ARRM interrupt to prevent wakeups */
		LPTIM->IER &= ~LPTIM_IER_ARRMIE;
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	/* Read current counter value - read ARR first to minimize race window */
	uint32_t autoreload = LPTIM->ARR;
	uint32_t lp_time = lptim_get_counter();

	/*
	 * Check if ARRM is active or if we're too close to the next match.
	 * If so, we can't safely update ARR - interrupt will fire soon anyway.
	 * Note: We read ARR first, then counter. If counter wraps between reads,
	 * we'll see a smaller delta (more conservative), so this is safe.
	 */
	if (lptim_is_arrm_active() ||
	    ((autoreload >= lp_time) && ((autoreload - lp_time) < LPTIM_GUARD_VALUE))) {
		k_spin_unlock(&lock, key);
		return;
	}

	/*
	 * Calculate the next ARR value:
	 * - ticks == 0 or negative: schedule next tick as soon as possible
	 * - ticks >= 1: schedule requested number of ticks
	 *
	 * First, align to next tick boundary from current counter position
	 */
	uint32_t cycles_per_tick = lptim_clock_freq / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	/* Defensive check to prevent division by zero */
	if (cycles_per_tick == 0) {
		cycles_per_tick = 1;
	}

	uint32_t next_arr;

	/* Calculate next tick boundary */
	uint32_t current_tick_cycles = (lp_time / cycles_per_tick) * cycles_per_tick;
	next_arr = current_tick_cycles + cycles_per_tick;

	/* Add requested ticks (clamping to valid range) */
	if (ticks <= 0) {
		ticks = 1;
	}

	/* Limit ticks to time base maximum */
	if (ticks > lptim_time_base) {
		ticks = lptim_time_base;
	}

	next_arr += (uint32_t)(ticks - 1) * cycles_per_tick;

	/* Clamp to maximum time base */
	if (next_arr > (uint32_t)lptim_time_base) {
		next_arr = lptim_time_base;
	}

	/* Ensure minimum guard value from current counter */
	if (next_arr < (lp_time + LPTIM_GUARD_VALUE)) {
		next_arr = lp_time + LPTIM_GUARD_VALUE;
	}

	/* Convert to ARR value (counter counts from 0 to ARR, so ARR = target - 1) */
	if (next_arr > 0) {
		next_arr--;
	}

	/* Clamp to 16-bit maximum */
	if (next_arr > LPTIM_MAX_VALUE) {
		next_arr = LPTIM_MAX_VALUE;
	}

	/* Update autoreload register */
	lptim_set_autoreload(next_arr);

	/* Re-enable ARRM interrupt in case it was disabled by K_TICKS_FOREVER */
	LPTIM->IER |= LPTIM_IER_ARRMIE;

	k_spin_unlock(&lock, key);
}

static int sys_clock_driver_init(void)
{
	int err;

	/* Validate clock device is ready */
	if (!device_is_ready(clk_dev)) {
		return -ENODEV;
	}

	/* Enable LPTIM clock via clock control API */
	err = clock_control_on(clk_dev, (clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(0, id));
	if (err < 0) {
		return err;
	}

	/* Get LPTIM clock frequency */
	err = clock_control_get_rate(clk_dev,
				    (clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(0, id),
				    &lptim_clock_freq);
	if (err < 0) {
		return err;
	}

	/* Apply prescaler to get actual timer clock frequency */
	lptim_clock_freq = lptim_clock_freq / lptim_clock_presc;

	/* Validate frequency vs ticks per second ratio */
	if (lptim_clock_freq < (uint32_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
		return -EINVAL;
	}

	/* Calculate time base (max timeout in cycles) */
	uint32_t timeout_sec = DT_INST_PROP_OR(0, wch_timeout, 1);
	lptim_time_base = lptim_clock_freq * timeout_sec;
	if (lptim_time_base > LPTIM_MAX_VALUE) {
		lptim_time_base = LPTIM_MAX_VALUE;
	}

	/* NVIC configuration */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), lptim_irq_handler, NULL, 0);
	irq_enable(DT_INST_IRQN(0));

	/* Configure LPTIM */
	LPTIM->CR = 0; /* Disable */

	/* Calculate prescaler bits (0-7 for div 1,2,4,8,16,32,64,128) */
	uint32_t presc_val = 0;
	uint32_t p = lptim_clock_presc;
	while (p > 1) {
		p >>= 1;
		presc_val++;
	}

	/* Configure: clock source, prescaler, timeout enable */
	uint32_t clksel = (lptim_clock_source & 0x3) << LPTIM_CFGR_CLKSEL_Pos;
	uint32_t presc_bits = (presc_val & 0x7) << LPTIM_CFGR_PRESC_Pos;

	LPTIM->CFGR = LPTIM_CFGR_TIMOUT | clksel | presc_bits;

	/* Enable LPTIM */
	LPTIM->CR |= LPTIM_CR_ENABLE;

	/* Set initial ARR for periodic tick */
	uint32_t cycles_per_tick = lptim_clock_freq / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	uint32_t initial_arr = cycles_per_tick - 1;
	if (initial_arr > LPTIM_MAX_VALUE) {
		initial_arr = LPTIM_MAX_VALUE;
	}

	err = lptim_set_autoreload(initial_arr);
	if (err < 0) {
		return err;
	}

	/* Enable ARRM interrupt */
	LPTIM->IER |= LPTIM_IER_ARRMIE;

	/* Start in continuous mode */
	LPTIM->CR |= LPTIM_CR_CNTSTRT;

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
