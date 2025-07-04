/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_erpc_wifi_uart

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_erpc_wifi_transport, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <erpc_transport_setup.h>

#include "erpc_wifi_transport.h"

erpc_transport_t erpc_wifi_transport_init(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_INST_BUS(0));

	if (!device_is_ready(dev)) {
		LOG_ERR("Bus device is not ready");
		return NULL;
	}

    return erpc_transport_zephyr_uart_init((void *)dev);
}
