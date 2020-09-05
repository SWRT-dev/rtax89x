/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef IPQ807X_CLK_H
#define IPQ807X_CLK_H

/* UART clocks configuration */
void uart2_clock_config(unsigned int m,
			unsigned int n, unsigned int two_d);
void uart2_toggle_clock(void);
int uart2_trigger_update(void);
void uart2_set_rate_mnd(unsigned int m,
			unsigned int n, unsigned int two_d);
void uart2_configure_mux(void);

/* I2C clocks configuration */
#ifdef CONFIG_IPQ807x_I2C
void i2c_clock_config(void);
#endif

#endif /*IPQ807X_CLK_H*/
