/*
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include "gpio-names.h"
#include "wakeups.h"

static struct tegra_wake_info tegra_wake_event_data_t3[] = {
	// irq, gpio, polarity
	{-EINVAL, TEGRA_GPIO_PO5, POLARITY_NONE},	/* wake0 */
	{-EINVAL, TEGRA_GPIO_PV1, POLARITY_NONE},	/* wake1 */
	{-EINVAL, TEGRA_GPIO_PL1, POLARITY_NONE},	/* wake2 */
	{-EINVAL, TEGRA_GPIO_PB6, POLARITY_NONE},	/* wake3 */
	{-EINVAL, TEGRA_GPIO_PN7, POLARITY_NONE},	/* wake4 */
	{-EINVAL, TEGRA_GPIO_PBB6, POLARITY_NONE},	/* wake5 */
	{-EINVAL, TEGRA_GPIO_PU5, POLARITY_NONE},	/* wake6 */
	{-EINVAL, TEGRA_GPIO_PU6, POLARITY_NONE},	/* wake7 */
	{-EINVAL, TEGRA_GPIO_PC7, POLARITY_NONE},	/* wake8 */
	{-EINVAL, TEGRA_GPIO_PS2, POLARITY_NONE},	/* wake9 */
	{-EINVAL, TEGRA_GPIO_PAA1, POLARITY_NONE},	/* wake10 */
	{-EINVAL, TEGRA_GPIO_PW3, POLARITY_NONE},	/* wake11 */
	{-EINVAL, TEGRA_GPIO_PW2, POLARITY_NONE},	/* wake12 */
	{-EINVAL, TEGRA_GPIO_PY6, POLARITY_NONE},	/* wake13 */
	{-EINVAL, TEGRA_GPIO_PDD3, POLARITY_NONE},	/* wake14 */
	{-EINVAL, TEGRA_GPIO_PJ2, POLARITY_NONE},	/* wake15 */
	{INT_RTC, -EINVAL, POLARITY_NONE},				/* wake16 */
	{INT_KBC, -EINVAL, POLARITY_NONE},				/* wake17 */
	{INT_EXTERNAL_PMU, -EINVAL, POLARITY_NONE},			/* wake18 */
	{INT_USB, -EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB1_VBUS, */		/* wake19 */
	{-EINVAL, -EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB2_VBUS, */		/* wake20 */
	{INT_USB, -EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB1_ID, */		/* wake21 */
	{-EINVAL, -EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB2_ID, */		/* wake22 */
	{-EINVAL, TEGRA_GPIO_PI5, POLARITY_NONE},	/* wake23 */
	{-EINVAL, TEGRA_GPIO_PV0, POLARITY_NONE},	/* wake24 */
	{-EINVAL, TEGRA_GPIO_PS4, POLARITY_NONE},	/* wake25 */
	{-EINVAL, TEGRA_GPIO_PS5, POLARITY_NONE},	/* wake26 */
	{-EINVAL, TEGRA_GPIO_PS0, POLARITY_NONE},	/* wake27 */
	{-EINVAL, TEGRA_GPIO_PS6, POLARITY_NONE},	/* wake28 */
	{-EINVAL, TEGRA_GPIO_PS7, POLARITY_NONE},	/* wake29 */
	{-EINVAL, TEGRA_GPIO_PN2, POLARITY_NONE},	/* wake30 */
	{-EINVAL, -EINVAL, POLARITY_NONE}, /* not used */			/* wake31 */
	{-EINVAL, TEGRA_GPIO_PO4, POLARITY_NONE},	/* wake32 */
	{-EINVAL, TEGRA_GPIO_PJ0, POLARITY_NONE},	/* wake33 */
	{-EINVAL, TEGRA_GPIO_PK2, POLARITY_NONE},	/* wake34 */
	{-EINVAL, TEGRA_GPIO_PI6, POLARITY_NONE},	/* wake35 */
	{-EINVAL, TEGRA_GPIO_PBB1, POLARITY_NONE},	/* wake36 */
	{-EINVAL, -EINVAL, POLARITY_NONE}, /* TEGRA_USB3_VBUS, */		/* wake37 */
	{-EINVAL, -EINVAL, POLARITY_NONE}, /* TEGRA_USB3_ID, */		/* wake38 */
	{INT_USB, -EINVAL, POLARITY_LEVEL_HI}, /* TEGRA_USB1_UTMIP, */		/* wake39 */
	{INT_USB2, -EINVAL, POLARITY_LEVEL_HI}, /* TEGRA_USB2_UTMIP, */	/* wake40 */
	{-EINVAL, -EINVAL, POLARITY_NONE}, /* TEGRA_USB3_UTMIP, */	/* wake41 */
	{INT_USB2, -EINVAL, POLARITY_LEVEL_HI}, /* TEGRA_USB2_UHSIC, */	/* wake42 */
};

struct tegra_wake_info *tegra_wake_event_data = tegra_wake_event_data_t3;
unsigned int tegra_wake_event_data_size = ARRAY_SIZE(tegra_wake_event_data_t3);

