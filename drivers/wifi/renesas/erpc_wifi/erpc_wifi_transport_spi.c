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
		//printf("Slave-ready GPIO controller not ready: %s", cfg->n_int.port->name);
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
		return 0;
	}

	int v = gpio_pin_get_dt(g_slave_ready_gpio);
	if (v < 0) {
		return 0;
	}

	return (v != 0);
}

static struct gpio_callback srdy_cb_data;
static erpc_transport_srdy_cb_t g_srdy_cb = NULL;

static void srdy_gpio_callback_handler(const struct device *port,
				       struct gpio_callback *cb,
				       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (g_srdy_cb) {
		g_srdy_cb();
	}
}

void erpc_transport_register_srdy_cb(erpc_transport_srdy_cb_t cb)
{
	g_srdy_cb = cb;

	if (g_slave_ready_gpio && cb) {
		gpio_remove_callback(g_slave_ready_gpio->port, &srdy_cb_data);
		gpio_init_callback(&srdy_cb_data, srdy_gpio_callback_handler, BIT(g_slave_ready_gpio->pin));
		gpio_add_callback(g_slave_ready_gpio->port, &srdy_cb_data);

		/* If SRDY is already high, there may be no new edge; kick callback once. */
		if (gpio_pin_get_dt(g_slave_ready_gpio) > 0) {
			cb();
		}
	}
}