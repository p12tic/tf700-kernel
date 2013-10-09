/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Iliyan Malchev <malchev@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_TEGRA_FIQ_H
#define __ASM_ARCH_TEGRA_FIQ_H

#include <mach/gpio-tegra.h>

/* enable/disable an interrupt that is an FIQ (safe from FIQ context?) */
void tegra_fiq_enable(int n);
void tegra_fiq_disable(int n);

/* SDCARD */
#define TEGRA_GPIO_SD1_CD		TEGRA_GPIO_PV5
#define TEGRA_GPIO_SD1_WP		TEGRA_GPIO_PH1
#define TEGRA_GPIO_SD1_POWER		TEGRA_GPIO_PV1

/* ULPI */
#define TEGRA_ULPI_RST			TEGRA_GPIO_PV0

/* WIFI */
#define TEGRA_WIFI_PWRN			TEGRA_GPIO_PK5
#define TEGRA_WIFI_RST			TEGRA_GPIO_PD1
#define TEGRA_WIFI_LED			TEGRA_GPIO_PD0

#endif
