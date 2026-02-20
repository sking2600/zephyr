/*
 * Copyright (c) 2024 Dhiru Kholia
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NOTE: See this table for IC family reference,
 * in conjunction with Page 5 of the reference manual:
 * https://www.wch-ic.com/products/productsCenter/mcuInterface?categoryId=70
 */

#ifndef _CH32FUN_H
#define _CH32FUN_H


#if defined(CONFIG_SOC_CH32V003)
#define CH32V003 1
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_CH32V003) */

#if defined(CONFIG_SOC_SERIES_CH32V00X)
#define CH32V00x 1
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_CH32V00X) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V4B)
#define CH32V20x    1
#define CH32V20x_D6 1
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V4B) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V4C)
#if defined(CONFIG_SOC_CH32L103)
#define CH32L103 1
#elif defined(CONFIG_SOC_CH32V208)
#define CH32V20x 1
#define CH32V20x_D8W 1
#else
#error "Unsupported CH32V4C variant. Please define SOC_CH32L103 or SOC_CH32V208."
#endif
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V4C) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V4F)
#define CH32V30x 1
#if defined(CONFIG_SOC_CH32V303)
#define CH32V30x_D8 1
#elif defined(CONFIG_SOC_CH32V307)
#define CH32V30x_D8C 1
#endif
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V4F) */

/* Standard WCH ADC bits */
#ifndef ADC_ADON
#define ADC_ADON                BIT(0)
#endif

#ifndef ADC_TSVREFE
#define ADC_TSVREFE             BIT(23)
#endif

#ifndef ADC_RSWSTART
#define ADC_RSWSTART            BIT(22)
#endif

#ifndef ADC_CTLR1_SCAN
#define ADC_CTLR1_SCAN          BIT(8)
#endif

#ifndef ADC_CTLR2_DMA
#define ADC_CTLR2_DMA           BIT(8)
#endif

#ifndef ADC_RSTCAL
#define ADC_RSTCAL              BIT(3)
#endif

#ifndef ADC_CAL
#define ADC_CAL                 BIT(2)
#endif

#ifndef ADC_L_0
#define ADC_L_0                 BIT(20)
#endif

#ifndef WCH_ADC_PGA_1X
#define WCH_ADC_PGA_1X 0
#endif

#ifndef WCH_ADC_PGA_4X
#define WCH_ADC_PGA_4X BIT(27)
#endif

#ifndef WCH_ADC_PGA_16X
#define WCH_ADC_PGA_16X BIT(28)
#endif

#ifndef WCH_ADC_PGA_64X
#define WCH_ADC_PGA_64X (BIT(27) | BIT(28))
#endif

#ifndef WCH_ADC_PGA_MASK
#define WCH_ADC_PGA_MASK (BIT(27) | BIT(28))
#endif

/* WCH OPA/CMP Register definitions - CH32L103 and similar */
#ifndef _WCH_OPACMP_REGS_H
#define _WCH_OPACMP_REGS_H

struct wch_opacmp_regs {
	volatile uint16_t CFGR1;
	volatile uint16_t CFGR2;
	volatile uint32_t CTLR1;
	volatile uint32_t CTLR2;
	volatile uint32_t RESERVED0;
	volatile uint32_t RESERVED1;
	volatile uint32_t OPCMKEY;
};

/* OPCM Key Values */
#define WCH_OPCM_KEY1 0x45670123
#define WCH_OPCM_KEY2 0xCDEF89AB

/* OPA Control Register 1 (CTLR1) Bits */
#define OPA_CTLR1_EN1_MASK      BIT(0)
#define OPA_CTLR1_MODE1_MASK    (BIT(1) | BIT(2) | BIT(3))
#define OPA_CTLR1_PSEL1_MASK    (BIT(4) | BIT(5) | BIT(6))
#define OPA_CTLR1_FBEN1_MASK    BIT(7)
#define OPA_CTLR1_NSEL1_MASK    (BIT(8) | BIT(9) | BIT(10) | BIT(11))
#define OPA_CTLR1_LP1_MASK      BIT(12)

/* Shift values for CTLR1 */
#define OPA_CTLR1_MODE1_SHIFT   1
#define OPA_CTLR1_PSEL1_SHIFT   4
#define OPA_CTLR1_FBEN1_SHIFT   7
#define OPA_CTLR1_NSEL1_SHIFT   8

/* NSEL (Negative Input/Gain) Enumerations */
#define OPA_NSEL_CHN_PGA_1x     0
#define OPA_NSEL_CHN_PGA_2x     5
#define OPA_NSEL_CHN_PGA_4x     6
#define OPA_NSEL_CHN_PGA_8x     7
#define OPA_NSEL_CHN_PGA_16x    8
#define OPA_NSEL_CHN_PGA_32x    9
#define OPA_NSEL_CHN_PGA_64x    10

/* PSEL (Positive Input) Enumerations */
#define OPA_PSEL_CHP0           0
#define OPA_PSEL_CHP1           1

/* CMP Control Register 2 (CTLR2) Bits */
#define CMP_CTLR2_EN_MASK       BIT(0)
#define CMP_CTLR2_MODE_MASK     (BIT(1) | BIT(2))
#define CMP_CTLR2_NSEL_MASK     BIT(3)
#define CMP_CTLR2_PSEL_MASK     BIT(4)
#define CMP_CTLR2_HYEN_MASK     BIT(5)
#define CMP_CTLR2_LP_MASK       BIT(6)
#define CMP_CTLR2_ALL_MASK      0x7F

/* Global CTLR2 bits */
#define CMP_CTLR2_WAKEUP_MASK   (BIT(24) | BIT(25))
#define CMP_CTLR2_WAKEUP_SHIFT  24
#define CMP_CTLR2_LOCK          BIT(31)

#endif /* _WCH_OPACMP_REGS_H */

#endif
