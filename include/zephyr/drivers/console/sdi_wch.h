/*
 * Copyright (c) 2026 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CONSOLE_SDI_WCH_H_
#define ZEPHYR_INCLUDE_DRIVERS_CONSOLE_SDI_WCH_H_

#include <stdint.h>
#include <zephyr/toolchain.h>

/**
 * @brief WCH SDI Console Driver
 *
 * This driver provides console output via the WCH Serial Debug Interface (SDI).
 * It uses the DMDATA0 and DMDATA1 debug registers for communication with the
 * WCH-Link debugger.
 *
 * Note: This is a WCH-specific extension for direct SDI access. Standard
 * console output is handled via printk/stdout hooks installed during init.
 */

/**
 * @brief Initialize the WCH SDI Console.
 *
 * This enables the necessary debug module registers if needed.
 */
void sdi_wch_init(void);

/**
 * @brief Write a string to the WCH SDI Console using DMDATA registers.
 *
 * @param str The null-terminated string to write.
 *
 * @note This is a WCH-specific extension. For standard console output,
 * use printk() or printf() which are automatically redirected.
 */
void sdi_wch_puts(const char *str);

/**
 * @brief Write formatted output to the WCH SDI Console.
 *
 * @param format The format string.
 * @param ...    Variable arguments for the format string.
 *
 * @note This is a WCH-specific extension. For standard console output,
 * use printk() or printf() which are automatically redirected.
 */
__printf_like(1, 2) void sdi_wch_printf(const char *format, ...);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CONSOLE_SDI_WCH_H_ */
