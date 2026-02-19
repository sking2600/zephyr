/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_lptim_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/spinlock.h>
#include <soc.h>

LOG_MODULE_REGISTER(counter_lptim, CONFIG_COUNTER_LOG_LEVEL);

struct counter_lptim_config {
	struct counter_config_info info;
	LPTIM_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t clock_id;
	uint32_t clock_source;
	uint32_t prescaler;
	void (*irq_config)(const struct device *dev);
};

struct counter_lptim_data {
	counter_alarm_callback_t alarm_cb;
	uint32_t top_value;
	void *user_data;
	struct k_spinlock lock;
	uint32_t freq;  /* Actual frequency, calculated at runtime */
};

/* LPTIM register bit definitions */
#define LPTIM_CR_ENABLE     BIT(0)
#define LPTIM_CR_SNGSTRT    BIT(1)
#define LPTIM_CR_CNTSTRT    BIT(2)
#define LPTIM_IER_CMPMIE    BIT(0)
#define LPTIM_IER_ARRMIE    BIT(1)
#define LPTIM_ISR_CMPM      BIT(0)
#define LPTIM_ISR_ARRM      BIT(1)
#define LPTIM_ISR_CMPOK     BIT(3)
#define LPTIM_ISR_ARROK     BIT(4)
#define LPTIM_ICR_CMPMCF    BIT(0)
#define LPTIM_ICR_ARRMCF    BIT(1)
#define LPTIM_ICR_CMPOKCF   BIT(3)
#define LPTIM_ICR_ARROKCF   BIT(4)
#define LPTIM_CFGR_TIMOUT   BIT(19)

/* Clock source selection bits (25-26) */
#define LPTIM_CFGR_CLKSEL_Pos   25
#define LPTIM_CFGR_CLKSEL_Msk   (0x3U << LPTIM_CFGR_CLKSEL_Pos)

/* Prescaler bits (9-11) */
#define LPTIM_CFGR_PRESC_Pos    9
#define LPTIM_CFGR_PRESC_Msk    (0x7U << LPTIM_CFGR_PRESC_Pos)

/**
 * @brief Wait for LPTIM synchronization flag with timeout
 *
 * Waits for specified flag to be set in ISR register, with a timeout
 * to prevent infinite loops.
 *
 * @param regs LPTIM register base
 * @param bit Flag bit to wait for
 * @return 0 on success, -ETIMEDOUT on timeout
 */
static int lptim_sync_wait(LPTIM_TypeDef *regs, uint32_t bit)
{
	uint32_t retries = 100000;

	while (!(regs->ISR & bit) && retries--) {
		/* Wait for sync - yield to allow other threads */
		k_yield();
	}

	if (retries == 0) {
		LOG_ERR("LPTIM sync timeout waiting for bit 0x%08x", bit);
		return -ETIMEDOUT;
	}

	return 0;
}

static int counter_lptim_start(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;

	config->regs->CR |= LPTIM_CR_ENABLE;
	config->regs->CR |= LPTIM_CR_CNTSTRT;
	return 0;
}

static int counter_lptim_stop(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;

	config->regs->CR &= ~LPTIM_CR_ENABLE;
	return 0;
}

static int counter_lptim_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Double read for synchronization to get consistent value */
	uint32_t cnt;
	uint32_t prev;

	cnt = config->regs->CNT;
	do {
		prev = cnt;
		cnt = config->regs->CNT;
	} while (cnt != prev);

	*ticks = cnt;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int counter_lptim_set_alarm(const struct device *dev, uint8_t chan_id,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int err = 0;

	if (chan_id != 0) {
		k_spin_unlock(&data->lock, key);
		return -ENOTSUP;
	}

	data->alarm_cb = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;

	/* Set compare value */
	config->regs->CMP = alarm_cfg->ticks;

	/* Wait for CMPOK with timeout */
	err = lptim_sync_wait(config->regs, LPTIM_ISR_CMPOK);
	if (err < 0) {
		LOG_ERR("Failed to set compare value");
		data->alarm_cb = NULL;
		k_spin_unlock(&data->lock, key);
		return err;
	}

	/* Clear CMPOK flag */
	config->regs->ICR |= LPTIM_ICR_CMPOKCF;

	/* Enable compare match interrupt */
	config->regs->IER |= LPTIM_IER_CMPMIE;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int counter_lptim_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (chan_id != 0) {
		k_spin_unlock(&data->lock, key);
		return -ENOTSUP;
	}

	config->regs->IER &= ~LPTIM_IER_CMPMIE;
	data->alarm_cb = NULL;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int counter_lptim_set_top_value(const struct device *dev,
				       const struct counter_top_cfg *cfg)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int err = 0;

	if (cfg->ticks > 0xFFFF) {
		k_spin_unlock(&data->lock, key);
		return -EINVAL;
	}

	data->top_value = cfg->ticks;
	config->regs->ARR = cfg->ticks;

	/* Wait for ARROK with timeout */
	err = lptim_sync_wait(config->regs, LPTIM_ISR_ARROK);
	if (err < 0) {
		LOG_ERR("Failed to set top value");
		k_spin_unlock(&data->lock, key);
		return err;
	}

	/* Clear ARROK flag */
	config->regs->ICR |= LPTIM_ICR_ARROKCF;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static uint32_t counter_lptim_get_top_value(const struct device *dev)
{
	struct counter_lptim_data *data = dev->data;

	return data->top_value;
}

static uint32_t counter_lptim_get_freq(const struct device *dev)
{
	struct counter_lptim_data *data = dev->data;

	return data->freq;
}

static void counter_lptim_isr(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t isr = config->regs->ISR;

	if (isr & LPTIM_ISR_CMPM) {
		/* Clear flag */
		config->regs->ICR |= LPTIM_ICR_CMPMCF;

		/* Disable interrupt for single-shot */
		config->regs->IER &= ~LPTIM_IER_CMPMIE;

		counter_alarm_callback_t cb = data->alarm_cb;
		void *user_data = data->user_data;

		data->alarm_cb = NULL;

		if (cb) {
			k_spin_unlock(&data->lock, key);
			cb(dev, 0, config->regs->CNT, user_data);
			return;
		}
	}

	if (isr & LPTIM_ISR_ARRM) {
		/* Clear flag */
		config->regs->ICR |= LPTIM_ICR_ARRMCF;
	}

	k_spin_unlock(&data->lock, key);
}

static const struct counter_driver_api counter_lptim_api = {
	.start = counter_lptim_start,
	.stop = counter_lptim_stop,
	.get_value = counter_lptim_get_value,
	.set_alarm = counter_lptim_set_alarm,
	.cancel_alarm = counter_lptim_cancel_alarm,
	.set_top_value = counter_lptim_set_top_value,
	.get_top_value = counter_lptim_get_top_value,
	.get_freq = counter_lptim_get_freq,
};

#ifdef CONFIG_PM_DEVICE
static int counter_lptim_pm_action(const struct device *dev,
				    enum pm_device_action action)
{
	const struct counter_lptim_config *config = dev->config;
	int err = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Disable LPTIM and turn off clock */
		config->regs->CR &= ~LPTIM_CR_ENABLE;
		err = clock_control_off(config->clock_dev,
				(clock_control_subsys_t)(uintptr_t)config->clock_id);
		if (err < 0) {
			LOG_ERR("Failed to disable clock: %d", err);
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Enable clock and reconfigure LPTIM */
		err = clock_control_on(config->clock_dev,
				(clock_control_subsys_t)(uintptr_t)config->clock_id);
		if (err < 0) {
			LOG_ERR("Failed to enable clock: %d", err);
			return err;
		}
		/* Re-enable LPTIM - configuration is preserved */
		config->regs->CR |= LPTIM_CR_ENABLE;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

PM_DEVICE_DEFINE(counter_lptim_pm, counter_lptim_pm_action);
#endif /* CONFIG_PM_DEVICE */

static int counter_lptim_init(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	int err;

	err = clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);
	if (err < 0) {
		LOG_ERR("Failed to enable clock: %d", err);
		return err;
	}

	/* WCH LPTIM Clock Selection:
	 * 0: PCLK1 (Bus clock)
	 * 1: HSI (8MHz)
	 * 2: LSE (32.768kHz)
	 * 3: LSI (40kHz)
	 */
	uint32_t base_freq = 0;

	switch (config->clock_source) {
	case 1: /* HSI */
		base_freq = 8000000;
		break;
	case 2: /* LSE */
		base_freq = 32768;
		break;
	case 3: /* LSI */
		base_freq = 40000;
		break;
	default: /* PCLK1 */
		clock_control_get_rate(config->clock_dev,
				       (clock_control_subsys_t)(uintptr_t)config->clock_id,
				       &base_freq);
		break;
	}

	uint32_t clksel = (config->clock_source & 0x3) << LPTIM_CFGR_CLKSEL_Pos;

	/* Prescaler mapping:
	 * 1, 2, 4, 8, 16, 32, 64, 128
	 * Bits 9-11: 000, 001, 010, 011, 100, 101, 110, 111
	 */
	uint32_t presc_val = 0;
	uint32_t p = config->prescaler;
	while (p > 1) {
		p >>= 1;
		presc_val++;
	}

	/* Configure LPTIM */
	config->regs->CR &= ~LPTIM_CR_ENABLE;
	/* Reference code sets TIMOUT (Bit 19) and InClockSource (LSI/etc) */
	config->regs->CFGR = clksel | ((presc_val & 0x7) << LPTIM_CFGR_PRESC_Pos) | LPTIM_CFGR_TIMOUT;

	/* Enable LPTIM before writing ARR */
	config->regs->CR |= LPTIM_CR_ENABLE;

	/* Calculate and store actual frequency in data (not const config) */
	data->freq = base_freq / config->prescaler;

	/* Default top value */
	data->top_value = 0xFFFF;
	config->regs->ARR = 0xFFFF;

	/* Wait for ARROK with timeout handling */
	err = lptim_sync_wait(config->regs, LPTIM_ISR_ARROK);
	if (err < 0) {
		LOG_ERR("Failed to initialize ARR");
		return err;
	}

	config->regs->ICR |= LPTIM_ICR_ARROKCF;

	config->irq_config(dev);

	return 0;
}

#define LPTIM_DEVICE_INIT(inst)                                                                    \
	static void counter_lptim_irq_config_##inst(const struct device *dev)                          \
	{                                                                                              \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                               \
			    counter_lptim_isr, DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_enable(DT_INST_IRQN(inst));                                                            \
	}                                                                                              \
                                                                                                   \
	static struct counter_lptim_data counter_lptim_data_##inst = {                                 \
		.top_value = 0xFFFF,                                                                       \
		.freq = 0, /* Set at runtime */                                                            \
	};                                                                                             \
                                                                                                   \
	static struct counter_lptim_config counter_lptim_config_##inst = {                             \
		.info = {                                                                                  \
			.max_top_value = 0xFFFF,                                                               \
			.freq = 0, /* Retrieved from data at runtime */                                        \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,                                                  \
			.channels = 1,                                                                          \
		},                                                                                         \
		.regs = (LPTIM_TypeDef *)DT_INST_REG_ADDR(inst),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                                     \
		.clock_id = DT_INST_CLOCKS_CELL(inst, id),                                                 \
		.clock_source = DT_INST_PROP_OR(inst, wch_clock_source, 0),                                \
		.prescaler = DT_INST_PROP_OR(inst, wch_prescaler, 1),                                      \
		.irq_config = counter_lptim_irq_config_##inst,                                             \
	};                                                                                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, counter_lptim_init, PM_DEVICE_DT_INST_GET(inst),                   \
			      &counter_lptim_data_##inst, &counter_lptim_config_##inst, POST_KERNEL,             \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_lptim_api);

DT_INST_FOREACH_STATUS_OKAY(LPTIM_DEVICE_INIT)
