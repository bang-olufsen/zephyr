#pragma once

//#include "wifi_ra_to_host_common.h"
#include "wifi_host_to_ra_common.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>

typedef int (*erpc_wifi_msg_handler_t)(void *data);
typedef int (*erpc_wifi_msg_cb_t)(void *data);

typedef enum {
	ERPC_WIFI_BIND_CMD,
	ERPC_WIFI_CONNECT_CMD,
	ERPC_WIFI_LISTEN_CMD,
	ERPC_WIFI_ACCEPT_CMD,

	ERPC_WIFI_RECV_CMD,
	ERPC_WIFI_RECVFROM_CMD,
	ERPC_WIFI_SEND_CMD,
	ERPC_WIFI_SENDTO_CMD,

	ERPC_WIFI_SOCKET_CREATE_CMD,
	ERPC_WIFI_SOCKET_CLOSE_CMD,

	ERPC_WIFI_SOCKET_GETOPT_CMD,
	ERPC_WIFI_SOCKET_SETOPT_CMD,

	EPRC_WIFI_GET_SOCKET_EVT_CMD,
	EPRC_WIFI_GET_SERVER_EVT_CMD,
	ERPC_WIFI_WAIT_RA_AWAKE_CMD,

	ERPC_WIFI_AP_CONNECT_CMD,
	ERPC_WIFI_AP_DISCONNECT_CMD,
	ERPC_WIFI_AP_SCAN_CMD,
	ERPC_WIFI_AP_GET_RSSI_CMD,
	ERPC_WIFI_AP_GET_CONNECTION_INFO_CMD,
	ERPC_WIFI_GET_DRIVER_VER_CMD,

	ERPC_WIFI_PS_SET_PARAM_CMD,
	ERPC_WIFI_PS_GET_PARAM_CMD,
	ERPC_WIFI_PS_SLEEP_ENABLE_CMD,

	ERPC_WIFI_DPM_TCP_PORT_FILTER_SET_CMD,
	ERPC_WIFI_DPM_TCP_PORT_DELETE_CMD,
	ERPC_WIFI_DPM_UDP_PORT_FILTER_SET_CMD,
	ERPC_WIFI_DPM_UDP_PORT_DELETE_CMD,

	ERPC_WIFI_DNS_GETADDRINFO_CMD,

	ERPC_WIFI_OTP_MAC_READ_CMD,
	ERPC_WIFI_OTP_MAC_WRITE_CMD,
	ERPC_WIFI_GET_MAC_CMD,

	ERPC_WIFI_PMGR_ADD_SLEEP_CONSTRAINT_CMD,
	ERPC_WIFI_PMGR_REMOVE_SLEEP_CONSTRAINT_CMD,
	ERPC_WIFI_PMGR_DPM_RCV_READY_SET_CMD,
	ERPC_WIFI_PMGR_DPM_IS_ENABLED_CMD,
	ERPC_WIFI_PMGR_DPM_WAKEUP_DONE_CMD,
	ERPC_WIFI_PMGR_DPM_JOB_NAME_SET_CMD,

	ERPC_WIFI_PS_APPLY_CMD,

	ERPC_WIFI_TEST_CMD,
	ERPC_WIFI_LAST_CMD,
} erpc_wifi_cmd_t;

typedef struct erpc_wifi_cmd_ctx {
	atomic_t ref_count;
	struct k_sem sem;
	int cmd_ret;
	atomic_t timed_out;
} erpc_wifi_cmd_ctx_t;

typedef struct {
	erpc_wifi_cmd_t cmd;
	erpc_wifi_cmd_ctx_t *ctx;
	void *data;
	erpc_wifi_msg_cb_t cb;
} erpc_wifi_msg_data_t;

typedef struct {
	int fd;
	struct ra_erpc_sockaddr addr_erpc_wifi;
} erpc_wifi_msg_bind_t;

typedef struct {
	int fd;
	struct ra_erpc_sockaddr addr_erpc_wifi;
} erpc_wifi_msg_connect_t;

typedef struct {
	int fd;
	int backlog;
} erpc_wifi_msg_listen_t;

typedef struct {
	int fd;
	struct ra_erpc_sockaddr *addr_erpc_wifi;
	uint32_t *addrlen;
} erpc_wifi_msg_accept_t;

typedef struct {
	int fd;
	uint32_t flags;
	uint8_t *buff;
	size_t buff_size;
} erpc_wifi_msg_recv_t;

typedef struct {
	int fd;
	uint32_t flags;
	uint8_t *buff;
	size_t buff_size;
	ra_erpc_sockaddr *src_addr;
	uint32_t *src_addrlen;
} erpc_wifi_msg_recvfrom_t;

typedef struct {
	int fd;
	uint32_t flags;
	uint8_t *buff;
	size_t buff_size;
} erpc_wifi_msg_send_t;

typedef struct {
	int fd;
	uint32_t flags;
	uint8_t *buff;
	size_t buff_size;
	struct ra_erpc_sockaddr *addr_erpc_wifi;
} erpc_wifi_msg_sendto_t;

typedef struct {
	int fd;
} erpc_wifi_msg_socket_event_t;

typedef struct {
	int32_t domain;
	int32_t type;
	int32_t protocol;
} erpc_wifi_msg_socket_create_t;

typedef struct {
	int fd;
	int32_t level;
	int32_t optname;
	uint32_t *optval;
	size_t *optlen;
} erpc_wifi_msg_sockgetopt_t;

typedef struct {
	int fd;
	int32_t level;
	int32_t optname;
	uint32_t *optval;
	size_t optlen;
} erpc_wifi_msg_socksetopt_t;

typedef struct {
	int8_t *rssi;
} erpc_wifi_get_rssi_t;

typedef struct {
	WIFIConnectionInfo_t *connection_info;
} erpc_wifi_get_connection_info_t;

typedef struct {
	struct erpc_wifi_data *dev_data;
	WIFIScanResult_t *scan_results;
	uint8_t max_bss_cnt;
} erpc_wifi_scan_t;

typedef struct {
	ra_wifi_ps_param_t param;
	uint32_t value;
} erpc_wifi_ps_t;

typedef struct {
	struct ra_erp_server_event_t *event;
} erpc_wifi_server_evt_t;

typedef struct {
	char *fw_version_driver;
	size_t fw_ver_drv_len;
	char *fw_erpc_version;
	size_t fw_erpc_len;
} erpc_wifi_driver_version_t;

typedef struct {
	uint16_t port;
} erpc_wifi_dpm_tcp_port_filter_set_t;

typedef struct {
	uint16_t port;
} erpc_wifi_dpm_tcp_port_delete_t;

typedef struct {
	uint16_t port;
} erpc_wifi_dpm_udp_port_filter_set_t;

typedef struct {
	uint16_t port;
} erpc_wifi_dpm_udp_port_delete_t;

typedef struct {
	const char *node;
	WIFIIPAddress_t *result;
	uint8_t max_count;
	uint8_t *actual_count;
} erpc_wifi_dns_getaddrinfo_t;

typedef struct {
	uint8_t *mac;
} erpc_wifi_mac_t;

typedef struct {
	uint32_t constraint;
} erpc_wifi_pmgr_constraint_t;

typedef struct {
	uint32_t job_id;
} erpc_wifi_pmgr_job_t;

typedef struct {
	uint32_t job_id;
	const char *job_name;
} erpc_wifi_pmgr_job_name_t;

/* Core message queue APIs */
int erpc_wifi_cmd_init(void);

int erpc_wifi_register_cmd_handler(erpc_wifi_cmd_t cmd, erpc_wifi_msg_handler_t h);
int erpc_wifi_unregister_cmd_handler(erpc_wifi_cmd_t cmd);

/* Primary API: send command through queue and wait for completion */
int erpc_wifi_send_cmd(erpc_wifi_cmd_t cmd, void *data, size_t size, int tout);
