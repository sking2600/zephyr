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
#define CH32V20x     1
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
#define ADC_ADON BIT(0)
#endif

#ifndef ADC_TSVREFE
#define ADC_TSVREFE BIT(23)
#endif

#ifndef ADC_RSWSTART
#define ADC_RSWSTART BIT(22)
#endif

#ifndef ADC_CTLR1_SCAN
#define ADC_CTLR1_SCAN BIT(8)
#endif

#ifndef ADC_CTLR2_DMA
#define ADC_CTLR2_DMA BIT(8)
#endif

#ifndef ADC_RSTCAL
#define ADC_RSTCAL BIT(3)
#endif

#ifndef ADC_CAL
#define ADC_CAL BIT(2)
#endif

#ifndef ADC_L_0
#define ADC_L_0 BIT(20)
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

/*
 * CH32L103 PWR Register Definitions
 * PWR Base: 0x40007000 (APB1 peripheral)
 */
#if defined(CONFIG_SOC_CH32L103)

#define PWR_BASE_ADDR 0x40007000U

/* PWR_CTLR bits */
#ifndef PWR_CTLR_LPDS
#define PWR_CTLR_LPDS BIT(0) /* Low Power Deepsleep */
#endif
#ifndef PWR_CTLR_PDDS
#define PWR_CTLR_PDDS BIT(1) /* Power Down Deepsleep */
#endif
#ifndef PWR_CTLR_CWUF
#define PWR_CTLR_CWUF BIT(2) /* Clear Wakeup Flag */
#endif
#ifndef PWR_CTLR_CSBF
#define PWR_CTLR_CSBF BIT(3) /* Clear Standby Flag */
#endif
#define PWR_CTLR_PVDEN       BIT(4) /* PVD Enable */
#define PWR_CTLR_PLS_Pos     5
#define PWR_CTLR_PLS_Msk     (0x7 << PWR_CTLR_PLS_Pos)
#define PWR_CTLR_FLP         BIT(9)  /* Flash Low Power mode */
#define PWR_CTLR_AUTO_LDO_LP BIT(12) /* Auto LDO Low Power in Stop */
#define PWR_CTLR_LDO_LP      BIT(13) /* LDO Low Power */
/* RAM retention bits */
#define PWR_CTLR_RAM1        BIT(16) /* 2K RAM retention in standby */
#define PWR_CTLR_RAM2        BIT(17) /* 18K RAM retention in standby */
#define PWR_CTLR_RAM1_VBAT   BIT(18) /* 2K RAM retention via VBAT */
#define PWR_CTLR_RAM2_VBAT   BIT(19) /* 18K RAM retention via VBAT */
#define PWR_CTLR_RAM_LV      BIT(20) /* Low Voltage RAM retention */

/* PWR_CSR bits */
#ifndef PWR_CSR_WUF
#define PWR_CSR_WUF BIT(0) /* Wakeup Flag */
#endif
#ifndef PWR_CSR_SBF
#define PWR_CSR_SBF BIT(1) /* Standby Flag */
#endif
#ifndef PWR_CSR_PVDO
#define PWR_CSR_PVDO BIT(2) /* PVD Output */
#endif
#ifndef PWR_CSR_EWUP
#define PWR_CSR_EWUP BIT(8) /* Enable Wakeup Pin */
#endif

/* PWR Regulator definitions */
#define PWR_REGULATOR_ON        0x00000000U
#define PWR_REGULATOR_LOW_POWER 0x00000001U

/* Stop mode entry */
#define PWR_STOP_ENTRY_WFI 0x01U
#define PWR_STOP_ENTRY_WFE 0x02U

/* PWR Flag definitions */
#ifndef PWR_FLAG_WU
#define PWR_FLAG_WU 0x00000001U
#endif
#ifndef PWR_FLAG_SB
#define PWR_FLAG_SB 0x00000002U
#endif
#ifndef PWR_FLAG_PVDO
#define PWR_FLAG_PVDO 0x00000004U
#endif

#endif /* CONFIG_SOC_CH32L103 */

/*
 * CH32L103 RCC Register Definitions
 * RCC Base: 0x40021000
 */
#if defined(CONFIG_SOC_CH32L103)

#define RCC_BASE_ADDR 0x40021000U

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

/* RCC reset status register (RSTSCKR) bit definitions */
#ifndef RCC_RMVF
#define RCC_RMVF BIT(24) /* Remove reset flag */
#endif
#ifndef RCC_PINRSTF
#define RCC_PINRSTF BIT(26) /* PIN reset flag */
#endif
#ifndef RCC_PORRSTF
#define RCC_PORRSTF BIT(27) /* POR/PDR reset flag */
#endif
#ifndef RCC_SFTRSTF
#define RCC_SFTRSTF BIT(28) /* Software reset flag */
#endif
#ifndef RCC_IWDGRSTF
#define RCC_IWDGRSTF BIT(29) /* Independent watchdog reset flag */
#endif
#ifndef RCC_WWDGRSTF
#define RCC_WWDGRSTF BIT(30) /* Window watchdog reset flag */
#endif
#ifndef RCC_LPWRRSTF
#define RCC_LPWRRSTF BIT(31) /* Low-power reset flag */
#endif

/* PWR clock enable bit */
#define RCC_PB1_PERIPH_PWR BIT(28)

#endif /* CONFIG_SOC_CH32L103 */

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
#define OPA_CTLR1_EN1_MASK   BIT(0)
#define OPA_CTLR1_MODE1_MASK (BIT(1) | BIT(2) | BIT(3))
#define OPA_CTLR1_PSEL1_MASK (BIT(4) | BIT(5) | BIT(6))
#define OPA_CTLR1_FBEN1_MASK BIT(7)
#define OPA_CTLR1_NSEL1_MASK (BIT(8) | BIT(9) | BIT(10) | BIT(11))
#define OPA_CTLR1_LP1_MASK   BIT(12)

/* Shift values for CTLR1 */
#define OPA_CTLR1_MODE1_SHIFT 1
#define OPA_CTLR1_PSEL1_SHIFT 4
#define OPA_CTLR1_FBEN1_SHIFT 7
#define OPA_CTLR1_NSEL1_SHIFT 8

/* NSEL (Negative Input/Gain) Enumerations */
#define OPA_NSEL_CHN_PGA_1x  0
#define OPA_NSEL_CHN_PGA_2x  5
#define OPA_NSEL_CHN_PGA_4x  6
#define OPA_NSEL_CHN_PGA_8x  7
#define OPA_NSEL_CHN_PGA_16x 8
#define OPA_NSEL_CHN_PGA_32x 9
#define OPA_NSEL_CHN_PGA_64x 10

/* PSEL (Positive Input) Enumerations */
#define OPA_PSEL_CHP0 0
#define OPA_PSEL_CHP1 1

/* CMP Control Register 2 (CTLR2) Bits */
#define CMP_CTLR2_EN_MASK   BIT(0)
#define CMP_CTLR2_MODE_MASK (BIT(1) | BIT(2))
#define CMP_CTLR2_NSEL_MASK BIT(3)
#define CMP_CTLR2_PSEL_MASK BIT(4)
#define CMP_CTLR2_HYEN_MASK BIT(5)
#define CMP_CTLR2_LP_MASK   BIT(6)
#define CMP_CTLR2_ALL_MASK  0x7F

/* Global CTLR2 bits */
#define CMP_CTLR2_WAKEUP_MASK  (BIT(24) | BIT(25))
#define CMP_CTLR2_WAKEUP_SHIFT 24
#define CMP_CTLR2_LOCK         BIT(31)

#endif /* _WCH_OPACMP_REGS_H */

#endif
