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
#include <wifi_host_to_ra_common.h>
#include <erpc_server_setup.h>
#include <erpc_client_setup.h>
#include <erpc_transport_setup.h>
#include <erpc_mbf_setup.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ERPC_WIFI_MTU                1500
#define ERPC_WIFI_DRV_FW_VER_LEN_MAX 32

#ifndef ERPC_PMGR_JOB_ID_SEND
#define ERPC_PMGR_JOB_ID_SEND (1U)
#endif

#ifndef ERPC_PMGR_JOB_ID_RECV
#define ERPC_PMGR_JOB_ID_RECV (2U)
#endif

enum erpc_wifi_driver_state {
	ERPC_WIFI_DRIVER_INITIALIZING = 0,
	ERPC_WIFI_DRIVER_INITIALIZED,
};

struct erpc_wifi_data {
	struct net_if *net_iface;
	struct in_addr addr;
	enum wifi_iface_state state;
	scan_result_cb_t scan_cb;
	uint16_t scan_max_bss_cnt;
	struct WIFINetworkParams_t drv_nwk_params;
	erpc_server_t erpc_server;
	erpc_client_t client_manager;
	erpc_transport_t transport;
	erpc_transport_t arbitrator;
	erpc_mbf_t mbf;
	void *service;
	k_tid_t server_thread_id;

	enum erpc_wifi_driver_state driver_state;
	bool reset_msg_received;

	struct k_work_q workq;
	struct k_work scan_work;
	struct k_work connect_work;
	struct k_work disconnect_work;
	struct k_work reinit_work;

    struct k_sem sem_cmd_process;
	struct k_sem sem_if_ready;

	struct gpio_dt_spec *n_int_gpio;
	struct gpio_callback n_int_cb;
	char fw_version_driver[ERPC_WIFI_DRV_FW_VER_LEN_MAX];
	
	bool wifi_params_read;
	bool ipv4_assigned;
#if defined(CONFIG_NET_IPV6)
	bool ipv6_assigned;
#endif
};

bool erpc_wifi_ps_is_module_awake(void);
bool erpc_wifi_ps_is_module_asleep(void);
void erpc_wifi_gpio_wakeup_pulse_fast(void);
void erpc_wifi_ps_notify_socket_connected(void);
void erpc_wifi_ps_notify_socket_connect_complete(void);
void erpc_wifi_ps_notify_socket_connect_failed(void);
void erpc_wifi_ps_notify_socket_connect_start(void);
int erpc_wifi_ps_notify_wakeup(void);
void erpc_wifi_ps_hold_during_recv(void);
bool erpc_wifi_ps_is_enabled(void);
bool erpc_wifi_ps_socket_connect_pending(void);
bool erpc_wifi_ps_sleep_is_sent(void);
void erpc_wifi_ps_confirm_sleep(void);
bool erpc_wifi_ps_sleep_is_confirmed(void);
bool erpc_wifi_ps_wait_for_srdy_low_get(void);
void erpc_wifi_ps_wait_for_srdy_low_clear(void);
int erpc_wifi_wake_for_tx(void);
int erpc_wifi_pmgr_ram_release(uint32_t job_id);
int erpc_wifi_otp_mac_read(uint8_t mac[WIFI_MAC_ADDR_LEN]);
int erpc_wifi_otp_mac_write(const uint8_t mac[WIFI_MAC_ADDR_LEN]);
int erpc_wifi_get_mac(uint8_t mac[WIFI_MAC_ADDR_LEN]);
int erpc_wifi_ps_hold_awake(const char *reason);
int erpc_wifi_ps_release_awake(const char *reason);
void erpc_wifi_ps_cancel_sleep_work(void);
void erpc_wifi_ps_schedule_sleep(const char *reason);
void erpc_wifi_ps_wait_awake_tx(void);
void erpc_wifi_ps_wait_awake_rx(void);
void erpc_wifi_ps_reset_state_awake(void);
void erpc_wifi_ps_wake_failed(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_ */
