/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_TRANSPORT_H_
#define ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_TRANSPORT_H_

#include <erpc_transport_setup.h>

#ifdef __cplusplus
extern "C" {
#endif

erpc_transport_t erpc_wifi_transport_init(void);

void erpc_wifi_transport_deinit(erpc_transport_t transport);
int erpc_wifi_transport_slave_ready(void);
typedef void (*erpc_transport_srdy_cb_t)(void);
void erpc_transport_register_srdy_cb(erpc_transport_srdy_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_TRANSPORT_H_ */


