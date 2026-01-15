/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_erpc_wifi_spi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <erpc_transport_setup.h>
#include <erpc_transport_setup.h>
#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log.h>
#include "erpc_wifi_transport.h"

struct erpc_wifi_spi_config {
	struct gpio_dt_spec n_int;
	struct spi_dt_spec bus;
};
static const struct gpio_dt_spec *g_slave_ready_gpio;
static const struct erpc_wifi_spi_config erpc_wifi_config_spi0 = {
	.n_int = GPIO_DT_SPEC_INST_GET(0, int_gpios),
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_WORD_SET(8), 0)
};

erpc_transport_t erpc_wifi_transport_init(void)
{
	const struct erpc_wifi_spi_config *cfg = &erpc_wifi_config_spi0;
	g_slave_ready_gpio = &cfg->n_int;
	if (!spi_is_ready_dt(&cfg->bus)) {
		//LOG_ERR("SPI bus is not ready");
		return NULL;
	};
	if (!device_is_ready(cfg->n_int.port)) {
		printf("Slave-ready GPIO controller not ready: %s", cfg->n_int.port->name);
		return NULL;
	} 

	return erpc_transport_zephyr_spi_master_init((void *)&cfg->bus, (void *)&cfg->n_int);
}

void erpc_wifi_transport_deinit(erpc_transport_t transport)
{
	erpc_transport_zephyr_spi_master_deinit(transport);
}
int erpc_wifi_transport_slave_ready(void)
{
	if (g_slave_ready_gpio == NULL) {
		return true;
	}
	if (!device_is_ready(g_slave_ready_gpio->port)) {
		printf("Slave-ready GPIO not ready at runtime: %s",
			g_slave_ready_gpio->port->name);
		return false;
	}

	int v = gpio_pin_get_dt(g_slave_ready_gpio);
	if (v < 0) {
		return false;
	}
	return (v == 0);
}