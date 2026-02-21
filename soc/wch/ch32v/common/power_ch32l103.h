/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_WCH_CH32V_COMMON_POWER_CH32L103_H_
#define SOC_WCH_CH32V_COMMON_POWER_CH32L103_H_

#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Update system core clock after wake from stop mode.
 *
 * This function should be called after waking up from stop mode
 * to reinitialize the system clocks. It is typically provided by
 * the clock control driver or HAL.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* SOC_WCH_CH32V_COMMON_POWER_CH32L103_H_ */
