/*
 * Copyright (c) 2026 Scott King
 * SPDX-License-Identifier: Apache-2.0
 *
 * WCH SDI Console Driver
 *
 * This driver provides console output via the WCH Serial Debug Interface (SDI)
 * using the DMDATA0 and DMDATA1 debug registers for communication with the
 * WCH-Link debugger.
 */

#include <zephyr/drivers/console/sdi_wch.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk-hooks.h>
#include <zephyr/sys/libc-hooks.h>

LOG_MODULE_REGISTER(sdi_wch, CONFIG_SDI_WCH_LOG_LEVEL);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sdi), okay)
#define DMDATA0_ADDR DT_REG_ADDR(DT_NODELABEL(sdi))
#define DMDATA1_ADDR (DMDATA0_ADDR + 4)
#else
/* Fallback to V4 default if no node found */
#define DMDATA0_ADDR 0xe0000380
#define DMDATA1_ADDR 0xe0000384
#endif

#define DMDATA0 (*(volatile uint32_t *)DMDATA0_ADDR)
#define DMDATA1 (*(volatile uint32_t *)DMDATA1_ADDR)

#define SDI_TIMEOUT_NORMAL 0x80000
#define SDI_TIMEOUT_ISR    1000 /* Reduced timeout for ISR context */

/* Track if host is responding to avoid repeated timeouts */
static bool host_present = true;

void sdi_wch_init(void)
{
	DMDATA1 = 0x00;
	DMDATA0 = 0x80;
}

static int sdi_wch_write_chunk(const char *buf, int len)
{
	uint32_t timeout;

	if (len > 7) {
		len = 7;
	}

	/* Use reduced timeout in ISR context to avoid hanging */
	if (k_is_in_isr()) {
		timeout = SDI_TIMEOUT_ISR;
	} else {
		timeout = SDI_TIMEOUT_NORMAL;
	}

	while ((DMDATA0 & 0x80) && timeout--) {
		/* Wait for host to acknowledge previous data */
	}

	if (timeout == 0) {
		if (host_present) {
			LOG_WRN("SDI console timeout - host not responding");
			host_present = false;
		}
		return -ETIMEDOUT;
	}

	/* Mark host as present on successful write */
	host_present = true;

	uint32_t d0 = (len + 4) | 0x80;
	uint32_t d1 = 0;
	uint8_t *p0 = (uint8_t *)&d0;
	uint8_t *p1 = (uint8_t *)&d1;

	for (int i = 0; i < len; i++) {
		if (i < 3) {
			p0[i + 1] = buf[i];
		} else {
			p1[i - 3] = buf[i];
		}
	}

	DMDATA1 = d1;
	DMDATA0 = d0;
	return 0;
}

void sdi_wch_puts(const char *str)
{
	int len = strlen(str);
	int pos = 0;

	while (pos < len) {
		int chunk_len = len - pos;

		if (chunk_len > 7) {
			chunk_len = 7;
		}
		if (sdi_wch_write_chunk(str + pos, chunk_len) < 0) {
			/* Timeout - abort remaining output */
			break;
		}
		pos += chunk_len;
	}
}

#define SDI_PRINTF_BUF_SIZE 128

__printf_like(1, 2) void sdi_wch_printf(const char *format, ...)
{
	char buf[SDI_PRINTF_BUF_SIZE];
	va_list args;
	int len;

	va_start(args, format);
	len = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	if (len > 0) {
		if (len >= SDI_PRINTF_BUF_SIZE) {
			LOG_WRN("SDI console output truncated (got %d, max %d)", len,
				SDI_PRINTF_BUF_SIZE - 1);
		}
		sdi_wch_puts(buf);
	}
}

static int sdi_wch_console_out(int character)
{
	char c = (char)character;

	sdi_wch_write_chunk(&c, 1);
	return character;
}

static int sdi_wch_console_init(void)
{
	sdi_wch_init();

#ifdef CONFIG_PRINTK
	__printk_hook_install(sdi_wch_console_out);
#endif
	__stdout_hook_install(sdi_wch_console_out);

	return 0;
}

SYS_INIT(sdi_wch_console_init, PRE_KERNEL_1, CONFIG_CONSOLE_INIT_PRIORITY);
