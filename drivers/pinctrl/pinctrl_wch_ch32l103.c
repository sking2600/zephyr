/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32l103_afio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/dt-bindings/pinctrl/ch32l103-pinctrl.h>

#include <hal_ch32fun.h>

/* This driver only supports CH32L103 */
#if !defined(CONFIG_SOC_CH32L103)
#error "pinctrl_wch_ch32l103.c only supports CH32L103"
#endif

static GPIO_TypeDef *const wch_afio_pinctrl_regs[] = {
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpioa)),
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpiob)),
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpioc)),
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpiod)),
#if DT_NODE_EXISTS(DT_NODELABEL(gpioe))
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpioe)),
#endif
};

static uint8_t pinctrl_build_cfg(const pinctrl_soc_pin_t *pin)
{
	uint8_t cfg = 0;

	if (pin->output_high || pin->output_low) {
		cfg |= (pin->slew_rate + 1);
		if (pin->drive_open_drain) {
			cfg |= BIT(2);
		}
		/* Select the alternate function */
		cfg |= BIT(3);
	} else if (pin->bias_pull_up || pin->bias_pull_down) {
		cfg |= BIT(3);
	}

	return cfg;
}

static void pinctrl_set_gpio_cfg(GPIO_TypeDef *regs, uint8_t pin, uint8_t cfg)
{
	if (pin < 8) {
		regs->CFGLR = (regs->CFGLR & ~(0x0F << (pin * 4))) | (cfg << (pin * 4));
	} else {
		regs->CFGHR = (regs->CFGHR & ~(0x0F << ((pin - 8) * 4))) | (cfg << ((pin - 8) * 4));
	}
}

static void pinctrl_set_pin_state(GPIO_TypeDef *regs, const pinctrl_soc_pin_t *pin, uint8_t pin_num)
{
	if (pin->output_high) {
		regs->BSHR = BIT(pin_num);
	} else if (pin->output_low) {
		regs->BCR = BIT(pin_num);
	} else {
		if (pin->bias_pull_up) {
			regs->BSHR = BIT(pin_num);
		}
		if (pin->bias_pull_down) {
			regs->BCR = BIT(pin_num);
		}
	}
}

static void pinctrl_apply_remap(uint8_t pcfr_id, uint8_t remap, uint8_t bit0)
{
	if (remap == 0) {
		return;
	}

	RCC->APB2PCENR |= RCC_AFIOEN;

	if (pcfr_id == 0) {
		AFIO->PCFR1 |= (uint32_t)remap << bit0;
	} else {
		AFIO->PCFR2 |= (uint32_t)remap << bit0;
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (int i = 0; i < pin_cnt; i++, pins++) {
		uint8_t port = FIELD_GET(CH32L103_PINCTRL_PORT_MASK, pins->config);
		uint8_t pin = FIELD_GET(CH32L103_PINCTRL_PIN_MASK, pins->config);
		uint8_t bit0 = FIELD_GET(CH32L103_PINCTRL_RM_BASE_MASK, pins->config);
		uint8_t pcfr_id = FIELD_GET(CH32L103_PINCTRL_PCFR_ID_MASK, pins->config);
		uint8_t remap = FIELD_GET(CH32L103_PINCTRL_RM_MASK, pins->config);

		GPIO_TypeDef *regs = wch_afio_pinctrl_regs[port];
		uint8_t cfg = pinctrl_build_cfg(pins);

		pinctrl_set_gpio_cfg(regs, pin, cfg);
		pinctrl_set_pin_state(regs, pins, pin);
		pinctrl_apply_remap(pcfr_id, remap, bit0);
	}

	return 0;
}

static int pinctrl_clock_init(void)
{
	const struct device *clock_dev;
	uint8_t clock_id;

#if DT_HAS_COMPAT_STATUS_OKAY(wch_ch32l103_afio)
	clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_COMPAT_GET_ANY_STATUS_OKAY(wch_ch32l103_afio)));
	clock_id = DT_CLOCKS_CELL(DT_COMPAT_GET_ANY_STATUS_OKAY(wch_ch32l103_afio), id);
#else
#error "No supported AFIO compatible found for CH32L103"
#endif

	return clock_control_on(clock_dev, (clock_control_subsys_t *)(uintptr_t)clock_id);
}

SYS_INIT(pinctrl_clock_init, PRE_KERNEL_1, 0);
