#include "erpc_wifi_cmd_process_handlers.h"
#include "erpc_wifi_cmd.h"
#include "erpc_wifi.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "zephyr/net/socket.h"

/* Include WiFi API headers for proper type definitions */
#include <c_wifi_host_to_ra_client.h>

LOG_MODULE_REGISTER(erpc_wifi_cmd_ap_process, CONFIG_WIFI_LOG_LEVEL);

/* All declarations are provided by c_wifi_host_to_ra_client.h */

/* AP operation handlers - executed in queue handler thread */

/* Forward declare global driver data */
extern struct erpc_wifi_data erpc_wifi_driver_data;

static int erpc_wifi_ap_connect_msg_process(void *data)
{
	ARG_UNUSED(data);
	/* Use global driver data which contains the configured network parameters */
	return WIFI_ConnectAP(&erpc_wifi_driver_data.drv_nwk_params);
}

static int erpc_wifi_ap_disconnect_msg_process(void *data)
{
	ARG_UNUSED(data);
	return WIFI_Disconnect();
}

static int erpc_wifi_ap_scan_msg_process(void *data)
{
	erpc_wifi_scan_t *msg = (erpc_wifi_scan_t *)data;
	if (!msg) return -EINVAL;

	int ret = WIFI_Scan(msg->scan_results, msg->max_bss_cnt);
	return (ret == eWiFiSuccess) ? 0 : -1;
}

static int erpc_wifi_ap_getrssi_msg_process(void *data)
{
	erpc_wifi_get_rssi_t *msg = (erpc_wifi_get_rssi_t *)data;
	if (!msg) return -EINVAL;

	return WIFI_GetRSSI(msg->rssi);
}

static int erpc_wifi_ap_get_connection_info_msg_process(void *data)
{
	erpc_wifi_get_connection_info_t *msg = (erpc_wifi_get_connection_info_t *)data;
	if (!msg) return -EINVAL;

	return WIFI_GetConnectionInfo(msg->connection_info);
}

static int erpc_wifi_get_driver_ver_msg_process(void *data)
{
	erpc_wifi_driver_version_t *msg = (erpc_wifi_driver_version_t *)data;
	if (!msg) return -EINVAL;

	/* actual API: void fw_version_get_driver_ver(char *version_s, uint8_t len_max) */
	fw_version_get_driver_ver(msg->fw_version_driver, (uint8_t)msg->fw_ver_drv_len);
	return 0;
}

static int erpc_wifi_get_server_evt_msg_process(void *data)
{
	erpc_wifi_server_evt_t *msg = (erpc_wifi_server_evt_t *)data;
	if (!msg) return -EINVAL;

	erpc_get_server_event(msg->event);
	return 0;
}

/* Power Save (PS) parameter handlers */

static int erpc_wifi_ps_set_param_msg_process(void *data)
{
	erpc_wifi_ps_t *msg = (erpc_wifi_ps_t *)data;
	if (!msg) return -EINVAL;

	/* actual API: int32_t ra6w1_wifi_ps_set_param(ra_wifi_ps_param_t param, uint32_t value) */
	return ra6w1_wifi_ps_set_param(msg->param, msg->value);
}

static int erpc_wifi_ps_get_param_msg_process(void *data)
{
	erpc_wifi_ps_t *msg = (erpc_wifi_ps_t *)data;
	if (!msg) return -EINVAL;

	/* actual API: int32_t ra6w1_wifi_ps_get_param(ra_wifi_ps_param_t param, uint32_t *value) */
	return ra6w1_wifi_ps_get_param(msg->param, &msg->value);
}

static int erpc_wifi_ps_sleep_enable_msg_process(void *data)
{
	/* Placeholder - PS sleep enable handled separately */
	ARG_UNUSED(data);
	return 0;
}

/* DPM (Dynamic Power Management) TCP/UDP port filter handlers */

static int erpc_wifi_dpm_tcp_port_filter_set_msg_process(void *data)
{
	erpc_wifi_dpm_tcp_port_filter_set_t *msg = (erpc_wifi_dpm_tcp_port_filter_set_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_wifi_dpm_tcp_port_filter_set(msg->port);
}

static int erpc_wifi_dpm_tcp_port_delete_msg_process(void *data)
{
	erpc_wifi_dpm_tcp_port_delete_t *msg = (erpc_wifi_dpm_tcp_port_delete_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_wifi_dpm_tcp_port_delete(msg->port);
}

static int erpc_wifi_dpm_udp_port_filter_set_msg_process(void *data)
{
	/* UDP DPM port filter not yet implemented in eRPC service */
	ARG_UNUSED(data);
	return -ENOTSUP;
}

static int erpc_wifi_dpm_udp_port_delete_msg_process(void *data)
{
	/* UDP DPM port delete not yet implemented in eRPC service */
	ARG_UNUSED(data);
	return -ENOTSUP;
}

/* Handler registration function */
int erpc_wifi_cmd_ap_handlers_init(void)
{
	/* AP operations */
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_CONNECT_CMD, erpc_wifi_ap_connect_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_DISCONNECT_CMD, erpc_wifi_ap_disconnect_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_SCAN_CMD, erpc_wifi_ap_scan_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_GET_RSSI_CMD, erpc_wifi_ap_getrssi_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_GET_CONNECTION_INFO_CMD, 
								   erpc_wifi_ap_get_connection_info_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_GET_DRIVER_VER_CMD, erpc_wifi_get_driver_ver_msg_process);
	erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SERVER_EVT_CMD, erpc_wifi_get_server_evt_msg_process);

	/* Power Save operations */
	erpc_wifi_register_cmd_handler(ERPC_WIFI_PS_SET_PARAM_CMD, erpc_wifi_ps_set_param_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_PS_GET_PARAM_CMD, erpc_wifi_ps_get_param_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_PS_SLEEP_ENABLE_CMD, erpc_wifi_ps_sleep_enable_msg_process);

	/* DPM TCP/UDP port filter operations */
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DPM_TCP_PORT_FILTER_SET_CMD, 
								   erpc_wifi_dpm_tcp_port_filter_set_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DPM_TCP_PORT_DELETE_CMD, erpc_wifi_dpm_tcp_port_delete_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DPM_UDP_PORT_FILTER_SET_CMD, 
								   erpc_wifi_dpm_udp_port_filter_set_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DPM_UDP_PORT_DELETE_CMD, erpc_wifi_dpm_udp_port_delete_msg_process);

	LOG_INF("AP/DPM/PS handlers initialized");

	return 0;
}
