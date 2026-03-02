/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief DWCH CH32L103 Clock Definitions
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32L103_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32L103_CLOCK_H_

/** @brief Offset for AHB peripheral clock enable register */
#define CH32L103_AHB_PCENR_OFFSET  0
/** @brief Offset for APB2 peripheral clock enable register */
#define CH32L103_APB2_PCENR_OFFSET 1
/** @brief Offset for APB1 peripheral clock enable register */
#define CH32L103_APB1_PCENR_OFFSET 2

/**
 * @brief Helper macro to construct clock configuration
 *
 * @param bus Bus type (AHB, APB1, or APB2)
 * @param bit Bit position in the peripheral clock enable register
 */
#define CH32L103_CLOCK_CONFIG(bus, bit) (((CH32L103_##bus##_PCENR_OFFSET) << 5) | (bit))

/* AHB Peripherals */
/** @brief DMA1 clock */
#define RCC_AHB_DMA1  CH32L103_CLOCK_CONFIG(AHB, 0)
/** @brief SRAM clock */
#define RCC_AHB_SRAM  CH32L103_CLOCK_CONFIG(AHB, 2)
/** @brief CRC clock */
#define RCC_AHB_CRC   CH32L103_CLOCK_CONFIG(AHB, 6)
/** @brief USBFS clock */
#define RCC_AHB_USBFS CH32L103_CLOCK_CONFIG(AHB, 12)
/** @brief USBPD clock */
#define RCC_AHB_USBPD CH32L103_CLOCK_CONFIG(AHB, 17)

/* APB2 Peripherals */
/** @brief AFIO clock */
#define RCC_APB2_AFIO   CH32L103_CLOCK_CONFIG(APB2, 0)
/** @brief GPIOA clock */
#define RCC_APB2_GPIOA  CH32L103_CLOCK_CONFIG(APB2, 2)
/** @brief GPIOB clock */
#define RCC_APB2_GPIOB  CH32L103_CLOCK_CONFIG(APB2, 3)
/** @brief GPIOC clock */
#define RCC_APB2_GPIOC  CH32L103_CLOCK_CONFIG(APB2, 4)
/** @brief GPIOD clock */
#define RCC_APB2_GPIOD  CH32L103_CLOCK_CONFIG(APB2, 5)
/** @brief ADC1 clock */
#define RCC_APB2_ADC1   CH32L103_CLOCK_CONFIG(APB2, 9)
/** @brief TIM1 clock */
#define RCC_APB2_TIM1   CH32L103_CLOCK_CONFIG(APB2, 11)
/** @brief SPI1 clock */
#define RCC_APB2_SPI1   CH32L103_CLOCK_CONFIG(APB2, 12)
/** @brief USART1 clock */
#define RCC_APB2_USART1 CH32L103_CLOCK_CONFIG(APB2, 14)

/* APB1 Peripherals */
/** @brief TIM2 clock */
#define RCC_APB1_TIM2   CH32L103_CLOCK_CONFIG(APB1, 0)
/** @brief TIM3 clock */
#define RCC_APB1_TIM3   CH32L103_CLOCK_CONFIG(APB1, 1)
/** @brief TIM4 clock */
#define RCC_APB1_TIM4   CH32L103_CLOCK_CONFIG(APB1, 2)
/** @brief WWDG clock */
#define RCC_APB1_WWDG   CH32L103_CLOCK_CONFIG(APB1, 11)
/** @brief SPI2 clock */
#define RCC_APB1_SPI2   CH32L103_CLOCK_CONFIG(APB1, 14)
/** @brief USART2 clock */
#define RCC_APB1_USART2 CH32L103_CLOCK_CONFIG(APB1, 17)
/** @brief USART3 clock */
#define RCC_APB1_USART3 CH32L103_CLOCK_CONFIG(APB1, 18)
/** @brief USART4 clock */
#define RCC_APB1_USART4 CH32L103_CLOCK_CONFIG(APB1, 19)
/** @brief I2C1 clock */
#define RCC_APB1_I2C1   CH32L103_CLOCK_CONFIG(APB1, 21)
/** @brief I2C2 clock */
#define RCC_APB1_I2C2   CH32L103_CLOCK_CONFIG(APB1, 22)
/** @brief CAN1 clock */
#define RCC_APB1_CAN1   CH32L103_CLOCK_CONFIG(APB1, 25)
/** @brief BKP clock */
#define RCC_APB1_BKP    CH32L103_CLOCK_CONFIG(APB1, 27)
/** @brief PWR clock */
#define RCC_APB1_PWR    CH32L103_CLOCK_CONFIG(APB1, 28)
/** @brief LPTIM clock */
#define RCC_APB1_LPTIM  CH32L103_CLOCK_CONFIG(APB1, 31)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32L103_CLOCK_H_ */
