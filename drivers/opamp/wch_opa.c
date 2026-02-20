/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_opa

#include <zephyr/drivers/opamp.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <hal_ch32fun.h>

LOG_MODULE_REGISTER(opamp_wch, CONFIG_OPAMP_LOG_LEVEL);

/*
 * CH32L103 has only 1 OPA (OPA1).
 * CTLR1 register layout for OPA1:
 *   Bits 0: EN1
 *   Bits 1-3: MODE1
 *   Bits 4-6: PSEL1
 *   Bit 7: FBEN1
 *   Bits 8-11: NSEL1
 *   Bit 12: LP1
 * There is no OPA2 in this hardware.
 */

struct wch_opa_config {
	struct wch_opacmp_regs *regs;
	uint8_t psel;
	uint8_t nsel;
	uint8_t mode;
};

static int wch_opa_set_gain(const struct device *dev, enum opamp_gain gain)
{
	const struct wch_opa_config *config = dev->config;
	struct wch_opacmp_regs *regs = config->regs;
	uint32_t val = 0;
	uint32_t mask = 0;

	switch (gain) {
	case OPAMP_GAIN_1:
		val = OPA_NSEL_CHN_PGA_1x;
		break;
	case OPAMP_GAIN_2:
		val = OPA_NSEL_CHN_PGA_2x;
		break;
	case OPAMP_GAIN_4:
		val = OPA_NSEL_CHN_PGA_4x;
		break;
	case OPAMP_GAIN_8:
		val = OPA_NSEL_CHN_PGA_8x;
		break;
	case OPAMP_GAIN_16:
		val = OPA_NSEL_CHN_PGA_16x;
		break;
	case OPAMP_GAIN_32:
		val = OPA_NSEL_CHN_PGA_32x;
		break;
	case OPAMP_GAIN_64:
		val = OPA_NSEL_CHN_PGA_64x;
		break;
	default:
		return -ENOTSUP;
	}

	/* OPA1 uses bits 8-11 for NSEL in CTLR1 */
	mask = OPA_CTLR1_NSEL1_MASK;
	regs->CTLR1 = (regs->CTLR1 & ~mask) | (val << OPA_CTLR1_NSEL1_SHIFT);

	return 0;
}

static const struct opamp_driver_api wch_opa_api = {
	.set_gain = wch_opa_set_gain,
};

static int wch_opa_init(const struct device *dev)
{
	const struct wch_opa_config *config = dev->config;
	struct wch_opacmp_regs *regs = config->regs;

	/* OPA1 control bits in CTLR1 */
	uint32_t val = (1 << 0) | /* EN1 */
		       (config->mode << OPA_CTLR1_MODE1_SHIFT) |
		       (config->psel << OPA_CTLR1_PSEL1_SHIFT) |
		       (config->nsel << OPA_CTLR1_NSEL1_SHIFT);

	uint32_t mask = OPA_CTLR1_EN1_MASK | OPA_CTLR1_MODE1_MASK |
		       OPA_CTLR1_PSEL1_MASK | OPA_CTLR1_NSEL1_MASK;

	regs->CTLR1 = (regs->CTLR1 & ~mask) | val;

	return 0;
}

#define WCH_OPA_INIT(n)								\
	static const struct wch_opa_config wch_opa_config_##n = {			\
		.regs = (struct wch_opacmp_regs *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(n))), \
		.psel = DT_INST_PROP_OR(n, wch_psel, 0),				\
		.nsel = DT_INST_PROP_OR(n, wch_nsel, 0),				\
		.mode = DT_INST_PROP_OR(n, wch_mode, 0),				\
	};										\
	DEVICE_DT_INST_DEFINE(n, wch_opa_init, NULL, NULL,				\
			      &wch_opa_config_##n, POST_KERNEL,			\
			      CONFIG_OPAMP_INIT_PRIORITY, &wch_opa_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_OPA_INIT)
