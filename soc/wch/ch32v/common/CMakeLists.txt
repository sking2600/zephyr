# Copyright (c) 2025 James Bennion-Pedley
# Copyright (c) 2025 MASSDRIVER EI
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_CH32L103)
zephyr_library_sources_ifdef(CONFIG_PM power_ch32l103.c)
else()
zephyr_library_sources_ifdef(CONFIG_PM power.c)
endif()
zephyr_library_sources_ifdef(CONFIG_ARCH_HAS_CUSTOM_CPU_IDLE soc_idle.c)

zephyr_include_directories(.)
