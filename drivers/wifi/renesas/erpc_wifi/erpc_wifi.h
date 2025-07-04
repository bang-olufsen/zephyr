/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_
#define ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/drivers/spi.h>
#include <wifi_host_to_ra_common.h>
#include <erpc_server_setup.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ERPC_WIFI_MTU					1500
#define ERPC_WIFI_DRV_FW_VER_LEN_MAX	32

enum erpc_wifi_driver_state {
	ERPC_WIFI_DRIVER_INITIALIZING = 0,
	ERPC_WIFI_DRIVER_INITIALIZED,
};

struct erpc_wifi_data {
	struct net_if *net_iface;
	enum wifi_iface_state state;
	scan_result_cb_t scan_cb;
	uint16_t scan_max_bss_cnt;
	struct WIFINetworkParams_t drv_nwk_params;
	erpc_server_t erpc_server;
	enum erpc_wifi_driver_state driver_state;
	bool reset_msg_received;

	struct k_work_q workq;
	struct k_work scan_work;
	struct k_work connect_work;
	struct k_work disconnect_work;

	struct k_sem sem_if_ready;

	char fw_version_driver[ERPC_WIFI_DRV_FW_VER_LEN_MAX];
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_ */
