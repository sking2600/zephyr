/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief DWCH CH32L103 Pin Multiplexing Definitions
 */

#ifndef __CH32L103_PINCTRL_H__
#define __CH32L103_PINCTRL_H__

#include <zephyr/dt-bindings/dt-util.h>

/** @brief GPIO Port A identifier */
#define CH32L103_PINMUX_PORT_PA 0
/** @brief GPIO Port B identifier */
#define CH32L103_PINMUX_PORT_PB 1
/** @brief GPIO Port C identifier */
#define CH32L103_PINMUX_PORT_PC 2
/** @brief GPIO Port D identifier */
#define CH32L103_PINMUX_PORT_PD 3

/** @brief ADC1 remap - no remap available */
#define CH32L103_PINMUX_ADC1_RM    0 /* No remap for ADC */
/** @brief SPI1 remap - no remap */
#define CH32L103_PINMUX_SPI1_RM    0
/** @brief I2C1 remap - remap option 1 */
#define CH32L103_PINMUX_I2C1_RM    1
/** @brief USART1 remap - remap option 2 */
#define CH32L103_PINMUX_USART1_RM  2
/** @brief USART2 remap - remap option 3 */
#define CH32L103_PINMUX_USART2_RM  3
/** @brief USART3 remap - remap option 4 */
#define CH32L103_PINMUX_USART3_RM  4
/** @brief TIM1 remap - remap option 6 */
#define CH32L103_PINMUX_TIM1_RM    6
/** @brief TIM2 remap - remap option 8 */
#define CH32L103_PINMUX_TIM2_RM    8
/** @brief TIM3 remap - remap option 10 */
#define CH32L103_PINMUX_TIM3_RM    10
/** @brief TIM4 remap - remap option 12 */
#define CH32L103_PINMUX_TIM4_RM    12
/** @brief CAN1 remap - remap option 13 */
#define CH32L103_PINMUX_CAN1_RM    13
/** @brief PD01 remap - remap option 15 */
#define CH32L103_PINMUX_PD01_RM    15
/** @brief TIM5 CH4 remap - remap option 16 */
#define CH32L103_PINMUX_TIM5CH4_RM 16

/* Port number with 0-4 */
/** @brief Port field shift */
#define CH32L103_PINCTRL_PORT_SHIFT    0
/** @brief Port field mask */
#define CH32L103_PINCTRL_PORT_MASK     GENMASK(2, 0)
/* Pin number 0-15 */
/** @brief Pin field shift */
#define CH32L103_PINCTRL_PIN_SHIFT     3
/** @brief Pin field mask */
#define CH32L103_PINCTRL_PIN_MASK      GENMASK(6, 3)
/* Base remap bit 0-31 */
/** @brief Remap base field shift */
#define CH32L103_PINCTRL_RM_BASE_SHIFT 7
/** @brief Remap base field mask */
#define CH32L103_PINCTRL_RM_BASE_MASK  GENMASK(11, 7)
/* Remap Register ID */
/** @brief Remap register ID field shift */
#define CH32L103_PINCTRL_PCFR_ID_SHIFT 12
/** @brief Remap register ID field mask */
#define CH32L103_PINCTRL_PCFR_ID_MASK  GENMASK(12, 12)
/* Function remapping ID 0-3 */
/** @brief Function remap field shift */
#define CH32L103_PINCTRL_RM_SHIFT      13
/** @brief Function remap field mask */
#define CH32L103_PINCTRL_RM_MASK       GENMASK(14, 13)

/**
 * @brief Helper macro to construct pin mux configuration
 *
 * @param port GPIO port (PA, PB, PC, or PD)
 * @param pin Pin number (0-15)
 * @param rm Peripheral remap identifier
 * @param remapping Remap option (0-3)
 */
#define CH32L103_PINMUX_DEFINE(port, pin, rm, remapping)                                           \
	((CH32L103_PINMUX_PORT_##port << CH32L103_PINCTRL_PORT_SHIFT) |                            \
	 (pin << CH32L103_PINCTRL_PIN_SHIFT) |                                                     \
	 (CH32L103_PINMUX_##rm##_RM << CH32L103_PINCTRL_RM_BASE_SHIFT) |                           \
	 (remapping << CH32L103_PINCTRL_RM_SHIFT))

/* Pin swaps for USART1 */
/** @brief USART1 clock pin on PA8, remap 0 */
#define USART1_CK_PA8_0   CH32L103_PINMUX_DEFINE(PA, 8, USART1, 0)
/** @brief USART1 clock pin on PA8, remap 1 */
#define USART1_CK_PA8_1   CH32L103_PINMUX_DEFINE(PA, 8, USART1, 1)
/** @brief USART1 clock pin on PA10, remap 2 */
#define USART1_CK_PA10_2  CH32L103_PINMUX_DEFINE(PA, 10, USART1, 2)
/** @brief USART1 clock pin on PA5, remap 3 */
#define USART1_CK_PA5_3   CH32L103_PINMUX_DEFINE(PA, 5, USART1, 3)
/** @brief USART1 TX pin on PA9, remap 0 */
#define USART1_TX_PA9_0   CH32L103_PINMUX_DEFINE(PA, 9, USART1, 0)
/** @brief USART1 TX pin on PB6, remap 1 */
#define USART1_TX_PB6_1   CH32L103_PINMUX_DEFINE(PB, 6, USART1, 1)
/** @brief USART1 TX pin on PB15, remap 2 */
#define USART1_TX_PB15_2  CH32L103_PINMUX_DEFINE(PB, 15, USART1, 2)
/** @brief USART1 TX pin on PA6, remap 3 */
#define USART1_TX_PA6_3   CH32L103_PINMUX_DEFINE(PA, 6, USART1, 3)
/** @brief USART1 RX pin on PA10, remap 0 */
#define USART1_RX_PA10_0  CH32L103_PINMUX_DEFINE(PA, 10, USART1, 0)
/** @brief USART1 RX pin on PB7, remap 1 */
#define USART1_RX_PB7_1   CH32L103_PINMUX_DEFINE(PB, 7, USART1, 1)
/** @brief USART1 RX pin on PA8, remap 2 */
#define USART1_RX_PA8_2   CH32L103_PINMUX_DEFINE(PA, 8, USART1, 2)
/** @brief USART1 RX pin on PA7, remap 3 */
#define USART1_RX_PA7_3   CH32L103_PINMUX_DEFINE(PA, 7, USART1, 3)
/** @brief USART1 CTS pin on PA11, remap 0 */
#define USART1_CTS_PA11_0 CH32L103_PINMUX_DEFINE(PA, 11, USART1, 0)
/** @brief USART1 CTS pin on PA11, remap 1 */
#define USART1_CTS_PA11_1 CH32L103_PINMUX_DEFINE(PA, 11, USART1, 1)
/** @brief USART1 CTS pin on PA5, remap 2 */
#define USART1_CTS_PA5_2  CH32L103_PINMUX_DEFINE(PA, 5, USART1, 2)
/** @brief USART1 CTS pin on PC4, remap 3 */
#define USART1_CTS_PC4_3  CH32L103_PINMUX_DEFINE(PC, 4, USART1, 3)
/** @brief USART1 RTS pin on PA12, remap 0 */
#define USART1_RTS_PA12_0 CH32L103_PINMUX_DEFINE(PA, 12, USART1, 0)
/** @brief USART1 RTS pin on PA12, remap 1 */
#define USART1_RTS_PA12_1 CH32L103_PINMUX_DEFINE(PA, 12, USART1, 1)
/** @brief USART1 RTS pin on PA9, remap 2 */
#define USART1_RTS_PA9_2  CH32L103_PINMUX_DEFINE(PA, 9, USART1, 2)
/** @brief USART1 RTS pin on PC5, remap 3 */
#define USART1_RTS_PC5_3  CH32L103_PINMUX_DEFINE(PC, 5, USART1, 3)

/* Pin swaps for USART2 */
/** @brief USART2 clock pin on PA4, remap 0 */
#define USART2_CK_PA4_0  CH32L103_PINMUX_DEFINE(PA, 4, USART2, 0)
/** @brief USART2 clock pin on PD7, remap 1 */
#define USART2_CK_PD7_1  CH32L103_PINMUX_DEFINE(PD, 7, USART2, 1)
/** @brief USART2 TX pin on PA2, remap 0 */
#define USART2_TX_PA2_0  CH32L103_PINMUX_DEFINE(PA, 2, USART2, 0)
/** @brief USART2 TX pin on PD5, remap 1 */
#define USART2_TX_PD5_1  CH32L103_PINMUX_DEFINE(PD, 5, USART2, 1)
/** @brief USART2 RX pin on PA3, remap 0 */
#define USART2_RX_PA3_0  CH32L103_PINMUX_DEFINE(PA, 3, USART2, 0)
/** @brief USART2 RX pin on PD6, remap 1 */
#define USART2_RX_PD6_1  CH32L103_PINMUX_DEFINE(PD, 6, USART2, 1)
/** @brief USART2 CTS pin on PA0, remap 0 */
#define USART2_CTS_PA0_0 CH32L103_PINMUX_DEFINE(PA, 0, USART2, 0)
/** @brief USART2 CTS pin on PD3, remap 1 */
#define USART2_CTS_PD3_1 CH32L103_PINMUX_DEFINE(PD, 3, USART2, 1)
/** @brief USART2 RTS pin on PA1, remap 0 */
#define USART2_RTS_PA1_0 CH32L103_PINMUX_DEFINE(PA, 1, USART2, 0)
/** @brief USART2 RTS pin on PD4, remap 1 */
#define USART2_RTS_PD4_1 CH32L103_PINMUX_DEFINE(PD, 4, USART2, 1)

/* Pin swaps for USART3 */
/** @brief USART3 clock pin on PB12, remap 0 */
#define USART3_CK_PB12_0  CH32L103_PINMUX_DEFINE(PB, 12, USART3, 0)
/** @brief USART3 clock pin on PC12, remap 1 */
#define USART3_CK_PC12_1  CH32L103_PINMUX_DEFINE(PC, 12, USART3, 1)
/** @brief USART3 clock pin on PD10, remap 2 */
#define USART3_CK_PD10_2  CH32L103_PINMUX_DEFINE(PD, 10, USART3, 2)
/** @brief USART3 clock pin on PD10, remap 3 */
#define USART3_CK_PD10_3  CH32L103_PINMUX_DEFINE(PD, 10, USART3, 3)
/** @brief USART3 TX pin on PB10, remap 0 */
#define USART3_TX_PB10_0  CH32L103_PINMUX_DEFINE(PB, 10, USART3, 0)
/** @brief USART3 TX pin on PC10, remap 1 */
#define USART3_TX_PC10_1  CH32L103_PINMUX_DEFINE(PC, 10, USART3, 1)
/** @brief USART3 TX pin on PA13, remap 2 */
#define USART3_TX_PA13_2  CH32L103_PINMUX_DEFINE(PA, 13, USART3, 2)
/** @brief USART3 TX pin on PD8, remap 3 */
#define USART3_TX_PD8_3   CH32L103_PINMUX_DEFINE(PD, 8, USART3, 3)
/** @brief USART3 RX pin on PB11, remap 0 */
#define USART3_RX_PB11_0  CH32L103_PINMUX_DEFINE(PB, 11, USART3, 0)
/** @brief USART3 RX pin on PC11, remap 1 */
#define USART3_RX_PC11_1  CH32L103_PINMUX_DEFINE(PC, 11, USART3, 1)
/** @brief USART3 RX pin on PA14, remap 2 */
#define USART3_RX_PA14_2  CH32L103_PINMUX_DEFINE(PA, 14, USART3, 2)
/** @brief USART3 RX pin on PD9, remap 3 */
#define USART3_RX_PD9_3   CH32L103_PINMUX_DEFINE(PD, 9, USART3, 3)
/** @brief USART3 CTS pin on PB13, remap 0 */
#define USART3_CTS_PB13_0 CH32L103_PINMUX_DEFINE(PB, 13, USART3, 0)
/** @brief USART3 CTS pin on PB13, remap 1 */
#define USART3_CTS_PB13_1 CH32L103_PINMUX_DEFINE(PB, 13, USART3, 1)
/** @brief USART3 CTS pin on PD11, remap 2 */
#define USART3_CTS_PD11_2 CH32L103_PINMUX_DEFINE(PD, 11, USART3, 2)
/** @brief USART3 CTS pin on PD11, remap 3 */
#define USART3_CTS_PD11_3 CH32L103_PINMUX_DEFINE(PD, 11, USART3, 3)
/** @brief USART3 RTS pin on PB14, remap 0 */
#define USART3_RTS_PB14_0 CH32L103_PINMUX_DEFINE(PB, 14, USART3, 0)
/** @brief USART3 RTS pin on PB14, remap 1 */
#define USART3_RTS_PB14_1 CH32L103_PINMUX_DEFINE(PB, 14, USART3, 1)
/** @brief USART3 RTS pin on PD12, remap 2 */
#define USART3_RTS_PD12_2 CH32L103_PINMUX_DEFINE(PD, 12, USART3, 2)
/** @brief USART3 RTS pin on PD12, remap 3 */
#define USART3_RTS_PD12_3 CH32L103_PINMUX_DEFINE(PD, 12, USART3, 3)

/* Pin swaps for SPI1 */
/** @brief SPI1 NSS pin on PA4, remap 0 */
#define SPI1_NSS_PA4_0  CH32L103_PINMUX_DEFINE(PA, 4, SPI1, 0)
/** @brief SPI1 NSS pin on PA15, remap 1 */
#define SPI1_NSS_PA15_1 CH32L103_PINMUX_DEFINE(PA, 15, SPI1, 1)
/** @brief SPI1 SCK pin on PA5, remap 0 */
#define SPI1_SCK_PA5_0  CH32L103_PINMUX_DEFINE(PA, 5, SPI1, 0)
/** @brief SPI1 SCK pin on PB3, remap 1 */
#define SPI1_SCK_PB3_1  CH32L103_PINMUX_DEFINE(PB, 3, SPI1, 1)
/** @brief SPI1 MISO pin on PA6, remap 0 */
#define SPI1_MISO_PA6_0 CH32L103_PINMUX_DEFINE(PA, 6, SPI1, 0)
/** @brief SPI1 MISO pin on PB4, remap 1 */
#define SPI1_MISO_PB4_1 CH32L103_PINMUX_DEFINE(PB, 4, SPI1, 1)
/** @brief SPI1 MOSI pin on PA7, remap 0 */
#define SPI1_MOSI_PA7_0 CH32L103_PINMUX_DEFINE(PA, 7, SPI1, 0)
/** @brief SPI1 MOSI pin on PB5, remap 1 */
#define SPI1_MOSI_PB5_1 CH32L103_PINMUX_DEFINE(PB, 5, SPI1, 1)

/* Pin swaps for I2C1 */
/** @brief I2C1 SCL pin on PB6, remap 0 */
#define I2C1_SCL_PB6_0 CH32L103_PINMUX_DEFINE(PB, 6, I2C1, 0)
/** @brief I2C1 SCL pin on PB8, remap 1 */
#define I2C1_SCL_PB8_1 CH32L103_PINMUX_DEFINE(PB, 8, I2C1, 1)
/** @brief I2C1 SDA pin on PB7, remap 0 */
#define I2C1_SDA_PB7_0 CH32L103_PINMUX_DEFINE(PB, 7, I2C1, 0)
/** @brief I2C1 SDA pin on PB9, remap 1 */
#define I2C1_SDA_PB9_1 CH32L103_PINMUX_DEFINE(PB, 9, I2C1, 1)

/* ADC pins */
/** @brief ADC1 channel 0 on PA0, remap 0 */
#define ADC1_IN0_PA0_0 CH32L103_PINMUX_DEFINE(PA, 0, ADC1, 0)

/* TIM1 pins */
/** @brief TIM1 channel 1 on PA8, remap 0 */
#define TIM1_CH1_PA8_0 CH32L103_PINMUX_DEFINE(PA, 8, TIM1, 0)

#endif /* __CH32L103_PINCTRL_H__ */
