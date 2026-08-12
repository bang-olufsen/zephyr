/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_SOCKET_OFFLOAD_H_
#define ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_SOCKET_OFFLOAD_H_

#include <zephyr/net/wifi_mgmt.h>

#ifdef __cplusplus
extern "C" {
#endif

int erpc_wifi_socket_offload_init(struct net_if *iface);
void erpc_wifi_dns_offload_init(void);
int erpc_wifi_wake_for_tx();
void erpc_wifi_socket_invalidate_active_job_cache(void);
void erpc_wifi_offload_srdy_callback(void);
void erpc_wifi_offload_clear_srdy_pending(void);
void erpc_wifi_offload_server_evt_query_begin(void);
void erpc_wifi_offload_server_evt_query_end(void);
void erpc_wifi_offload_host_erpc_begin(void);
void erpc_wifi_offload_host_erpc_end(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_SOCKET_OFFLOAD_H_ */
