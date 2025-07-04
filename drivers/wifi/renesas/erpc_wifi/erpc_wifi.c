/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/kernel.h"

#if defined(CONFIG_ERPC_TRANSPORT_UART)
#define DT_DRV_COMPAT renesas_erpc_wifi_uart
#else
#define DT_DRV_COMPAT renesas_erpc_wifi_spi
#endif

#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_erpc_wifi, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/drivers/gpio.h>
#include <erpc_client_setup.h>
#include <erpc_transport_setup.h>
#include <erpc_mbf_setup.h>
#include <c_wifi_host_to_ra_client.h>
#ifdef CONFIG_ERPC_TRANSPORT_UART
#include <erpc_arbitrated_client_setup.h>
#include <erpc_server_setup.h>
#include <c_wifi_ra_to_host_server.h>
#endif

#include "erpc_wifi.h"
#include "erpc_wifi_socket_offload.h"
#include "erpc_wifi_transport.h"

#ifdef CONFIG_ERPC_TRANSPORT_UART
K_THREAD_STACK_DEFINE(erpc_wifi_server_thread_stack,
			  CONFIG_WIFI_ERPC_WIFI_SERVER_THREAD_STACK_SIZE);
static struct k_thread erpc_wifi_server_thread_data;
#endif

K_KERNEL_STACK_DEFINE(erpc_wifi_workq_stack,
		      CONFIG_WIFI_ERPC_WIFI_WORKQ_STACK_SIZE);

// TODO can this be static?
struct erpc_wifi_data erpc_wifi_driver_data;

static uint8_t wifi_chan_to_band(uint16_t chan)
{
	uint8_t band;

	if (wifi_utils_validate_chan_2g(chan)) {
		band = WIFI_FREQ_BAND_2_4_GHZ;
	} else if (wifi_utils_validate_chan_5g(chan)) {
		band = WIFI_FREQ_BAND_5_GHZ;
	} else {
		band = WIFI_FREQ_BAND_UNKNOWN;
	}

	return band;
}

static inline enum wifi_security_type drv_to_wifi_mgmt_sec(int drv_security_type)
{
	switch (drv_security_type) {
	case eWiFiSecurityOpen:
		return WIFI_SECURITY_TYPE_NONE;
	case eWiFiSecurityWEP:
		return WIFI_SECURITY_TYPE_WEP;
	case eWiFiSecurityWPA:
		return WIFI_SECURITY_TYPE_WPA_PSK;
	case eWiFiSecurityWPA2:
		return WIFI_SECURITY_TYPE_PSK;
	case eWiFiSecurityWPA2_ent:
		return WIFI_SECURITY_TYPE_PSK_SHA256;
	case eWiFiSecurityWPA3:
		return WIFI_SECURITY_TYPE_SAE;
	default:
		return WIFI_SECURITY_TYPE_UNKNOWN;
	}
}

static inline enum WIFISecurity_t wifi_mgmt_to_drv_sec(int wifi_mgmt_security_type)
{
	switch (wifi_mgmt_security_type) {
	case WIFI_SECURITY_TYPE_NONE:
		return eWiFiSecurityOpen;
	case WIFI_SECURITY_TYPE_WEP:
		return eWiFiSecurityWEP;
	case WIFI_SECURITY_TYPE_WPA_PSK:
		return eWiFiSecurityWPA;
	case WIFI_SECURITY_TYPE_PSK:
		return eWiFiSecurityWPA2;
	case WIFI_SECURITY_TYPE_PSK_SHA256:
		return eWiFiSecurityWPA2_ent;
	case  WIFI_SECURITY_TYPE_SAE:
		return eWiFiSecurityWPA3;
	default:
		return eWiFiSecurityNotSupported;
	}
}

/*
	Parameters passed to this function via wifi_scan_params:

	enum wifi_scan_type scan_type	
	uint8_t bands
	uint16_t dwell_time_active
	uint16_t dwell_time_passive
	const char *ssids[WIFI_MGMT_SCAN_SSID_FILT_MAX]
	uint16_t max_bss_cnt
	struct wifi_band_channel band_chan[WIFI_MGMT_SCAN_CHAN_MAX_MANUAL]
	
	This function results in the following function being called which
	takes the following parameters:

	WIFI_Scan
	WIFIScanResult_t * pxBuffer
	uint8_t ucNumNetworks

	which means we use the passed parameters as follows:

	scan_type			- NOT USED
	bands				- NOT USED
	dwell_time_active	- NOT USED
	dwell_time_passive	- NOT USED
	ssids				- NOT USED
	max_bss_cnt			- ucNumNetworks
	band_chan			- NOT USED

	TODO - We should use the extended scan function which will allow
	more of the passed parameters to be used.

	WIFI_ScanExtended
	WIFIScanResult_t * pxBuffer
	uint8_t ucNumNetworks
	const WIFIScanExtendedConfig_t * pxScanConfigExtended

	struct WIFIScanExtendedConfig_t

    WIFIScanConfig_t pxScanConfig;
    WIFIBand_t ucBand;

	struct WIFIScanConfig_t

    uint8_t ucSSID[32]
    uint8_t ucSSIDLength
    uint8_t ucChannel
*/
static int erpc_wifi_mgmt_scan(const struct device *dev,
	struct wifi_scan_params *params,
	scan_result_cb_t cb)
{
	struct erpc_wifi_data *data = dev->data;

	LOG_DBG("erpc_wifi_mgmt_scan");
	LOG_DBG("type: %d cb: 0x%x", params->scan_type, (int)data->scan_cb);

	if (data->scan_cb != NULL) {
		return -EINPROGRESS;
	}

	if (!net_if_is_carrier_ok(data->net_iface)) {
		return -EIO;
	}

	data->scan_cb = cb;
	data->scan_max_bss_cnt = params->max_bss_cnt;
	data->state = WIFI_STATE_SCANNING;

	k_work_submit_to_queue(&data->workq, &data->scan_work);

	return 0;
}

/*
	Parameters (wifi_scan_result) returned by the callback executed by this
	function:

	struct wifi_scan_result
	uint8_t ssid[WIFI_SSID_MAX_LEN + 1]
	uint8_t ssid_length
	uint8_t band
	uint8_t channel
	enum wifi_security_type security
	enum wifi_wpa3_enterprise_type wpa3_ent_type
	enum wifi_mfp_options mfp
	int8_t rssi
	uint8_t mac[WIFI_MAC_ADDR_LEN]
	uint8_t mac_length

	Scan results arrive via the WIFIScanResult_t parameter:

	struct WIFIScanResult_t
	uint8_t ucSSID[32]
	uint8_t ucSSIDLength
	uint8_t ucBSSID[6]
	WIFISecurity_t xSecurity
	int8_t cRSSI
	uint8_t ucChannel

	which results in the following be passed back:

	ssid			- ucSSID
	ssid_length		- ucSSIDLength
	band			- derived from ucChannel
	channel			- ucChannel
	security		- xSecurity
	wpa3_ent_type	- NOT POPULATED
	mfp				- NOT POPULATED
	rssi			- cRSSI
	mac				- ucBSSID
	mac_length		- WIFI_MAC_ADDR_LEN
*/
static void erpc_wifi_mgmt_scan_work(struct k_work *work)
{
	struct erpc_wifi_data *dev;
	struct wifi_scan_result entry;
	WIFIScanResult_t *results;
	WIFIReturnCode_t ret;
	int i;

	LOG_DBG("erpc_wifi_mgmt_scan_work");

	dev = CONTAINER_OF(work, struct erpc_wifi_data, scan_work);

	if (dev->scan_max_bss_cnt == 0) {
		dev->scan_max_bss_cnt = CONFIG_WIFI_ERPC_WIFI_MAX_BBS_COUNT;
	}

	results = malloc(dev->scan_max_bss_cnt * sizeof(WIFIScanResult_t));
	if (results != NULL) {
		memset(results, 0, dev->scan_max_bss_cnt * sizeof(WIFIScanResult_t));

		ret = WIFI_Scan(results, dev->scan_max_bss_cnt);

		LOG_DBG("WIFI_Scan: %d", ret);

		if ((ret == eWiFiSuccess) || (ret == eWiFiTimeout)) {
			for (i = 0; (results[i].ucSSIDLength != 0) && (i < dev->scan_max_bss_cnt); i++) {
				memset(&entry, 0, sizeof(struct wifi_scan_result));

				if (results[i].ucSSIDLength <= WIFI_SSID_MAX_LEN) {
					/* Tab character signifies hidden SSID */
					if (results[i].ucSSID[0] != '\t') {
						entry.ssid_length = results[i].ucSSIDLength;
						memcpy(entry.ssid, results[i].ucSSID, entry.ssid_length);
					}
				}

				entry.security = drv_to_wifi_mgmt_sec(results[i].xSecurity);
				entry.channel = results[i].ucChannel;
				entry.rssi = results[i].cRSSI;
				entry.mac_length = WIFI_MAC_ADDR_LEN;
				memcpy(entry.mac, results[i].ucBSSID, entry.mac_length);
				entry.band = wifi_chan_to_band(entry.channel);

				dev->scan_cb(dev->net_iface, 0, &entry);
				k_yield();
			}
		} else {
			// TODO - pass back an enumerated error code?
			dev->scan_cb(dev->net_iface, -1, NULL);
		}
		free(results);
	}

	dev->scan_cb(dev->net_iface, 0, NULL);
	dev->scan_cb = NULL;
	dev->state = WIFI_STATE_DISCONNECTED;
}

/*
	Parameters passed to this function via wifi_connect_req_params:

	const uint8_t *ssid;
	uint8_t ssid_length;
	const uint8_t *psk;
	uint8_t psk_length;
	const uint8_t *sae_password;
	uint8_t sae_password_length;
	uint8_t band;
	uint8_t channel;
	enum wifi_security_type security;
	enum wifi_mfp_options mfp;
	uint8_t bssid[WIFI_MAC_ADDR_LEN];
	int timeout;
	const uint8_t *anon_id;
	uint8_t aid_length;
	const uint8_t *key_passwd;
	uint8_t key_passwd_length;
	const uint8_t *key2_passwd;
	uint8_t key2_passwd_length;
	enum wifi_wpa3_enterprise_type wpa3_ent_mode;
	uint8_t TLS_cipher;
	int eap_ver;
	const uint8_t *eap_identity;
	uint8_t eap_id_length;
	const uint8_t *eap_password;
	uint8_t eap_passwd_length;
	bool verify_peer_cert;
	bool ft_used;
	int nusers;
	uint8_t passwds;
	const uint8_t *identities[WIFI_ENT_IDENTITY_MAX_USERS];
	const uint8_t *passwords[WIFI_ENT_IDENTITY_MAX_USERS];
	uint8_t ignore_broadcast_ssid;
	enum wifi_frequency_bandwidths bandwidth;

	This function results in the following function being called which
	takes the following parameters:

	WIFI_ConnectAP
	WIFINetworkParams_t * pxNetworkParams

	struct WIFINetworkParams_t
    uint8_t ucSSID[32];
    uint8_t ucSSIDLength;
    WIFISecurity_t xSecurity;
    union
    {
        WIFIWEPKey_t xWEP[4];
        WIFIWPAPassphrase_t xWPA;
    } xPassword;
    uint8_t ucDefaultWEPKeyIndex;
    uint8_t ucChannel;

	which means we use the passed parameters as follows:

	ssid					- ucSSID
	ssid_length				- ucSSIDLength
	psk						- xPassword.xWPA.cPassphrase
	psk_length				- xPassword.xWPA.ucLength
	sae_password			- NOT USED
	sae_password_length		- NOT USED
	band					- NOT USED
	channel					- ucChannel
	security				- xSecurity
	mfp						- NOT USED
	bssid					- NOT USED
	timeout					- NOT USED
	anon_id					- NOT USED
	aid_length				- NOT USED
	key_passwd				- NOT USED
	key_passwd_length		- NOT USED
	key2_passwd				- NOT USED
	key2_passwd_length		- NOT USED
	wpa3_ent_mode			- NOT USED
	TLS_cipher				- NOT USED
	eap_ver					- NOT USED
	eap_identity			- NOT USED
	eap_id_length			- NOT USED
	eap_password			- NOT USED
	eap_passwd_length		- NOT USED
	verify_peer_cert		- NOT USED
	ft_used					- NOT USED
	nusers					- NOT USED
	passwds					- NOT USED
	identities				- NOT USED
	passwords				- NOT USED
	ignore_broadcast_ssid	- NOT USED
	bandwidth				- NOT USED

	TODO - We should use the extended connect function which will allow
	more of the passed parameters to be used.

	WIFI_ConnectAPExt(const WIFINetworkParamsExt_t * pxNetworkParamsExt);

	struct WIFINetworkParamsExt_t

    WIFINetworkParams_t xNetworkParams;
    WIFIEnterpriseNetParams_t xEntNetParams;
    WIFIBand_t ucBand;
    e_ra_eprc_wifi_phy_mode_ext_t ucWiFi_mode;
    bool hidden_ssid;
    WIFIPmf_t pmf;
    uint8_t sae_groups[20];
*/
static int erpc_wifi_mgmt_connect(const struct device *dev,
	struct wifi_connect_req_params *params)
{
	struct erpc_wifi_data *data = dev->data;

	LOG_DBG("erpc_wifi_mgmt_connect");

	memset(&data->drv_nwk_params, 0, sizeof(sizeof(WIFINetworkParams_t)));

	data->drv_nwk_params.ucSSIDLength = MIN(params->ssid_length,
		sizeof(data->drv_nwk_params.ucSSID));
	memcpy(data->drv_nwk_params.ucSSID, params->ssid,
		data->drv_nwk_params.ucSSIDLength);
	data->drv_nwk_params.xSecurity = wifi_mgmt_to_drv_sec(params->security);
	// TODO - copy parameters based on security type
	data->drv_nwk_params.xPassword.xWPA.ucLength = MIN(params->psk_length,
		sizeof(data->drv_nwk_params.xPassword.xWPA.cPassphrase));
	memcpy(data->drv_nwk_params.xPassword.xWPA.cPassphrase, params->psk,
		data->drv_nwk_params.xPassword.xWPA.ucLength);
	data->drv_nwk_params.ucChannel = params->channel;

	k_work_submit_to_queue(&data->workq, &data->connect_work);

	return 0;
}

static void erpc_wifi_mgmt_connect_work(struct k_work *work)
{
	struct erpc_wifi_data *dev;
	WIFIReturnCode_t ret;
	int status;

	dev = CONTAINER_OF(work, struct erpc_wifi_data, connect_work);

	LOG_DBG("erpc_wifi_mgmt_connect_work");
	LOG_DBG("ucSSID: %s", dev->drv_nwk_params.ucSSID);
	LOG_DBG("ucSSIDLength: %d", dev->drv_nwk_params.ucSSIDLength);
	LOG_DBG("xSecurity: %d", dev->drv_nwk_params.xSecurity);
	LOG_DBG("xWPA.ucLength: %d", dev->drv_nwk_params.xPassword.xWPA.ucLength);
	LOG_DBG("ucChannel: %d", dev->drv_nwk_params.ucChannel);

	ret = WIFI_ConnectAP(&dev->drv_nwk_params);

	LOG_DBG("WIFI_ConnectAP: %d", ret);

	if (ret == eWiFiSuccess) {
		dev->state = WIFI_STATE_COMPLETED;
		status = WIFI_STATUS_CONN_SUCCESS;
		net_if_dormant_off(dev->net_iface);
	} else {
		status = WIFI_STATUS_CONN_FAIL;
	}

	wifi_mgmt_raise_connect_result_event(dev->net_iface, status);
}

static int erpc_wifi_mgmt_disconnect(const struct device *dev)
{
	struct erpc_wifi_data *data = dev->data;

	LOG_DBG("erpc_wifi_mgmt_disconnect");

	k_work_submit_to_queue(&data->workq, &data->disconnect_work);

	return 0;
}

static void erpc_wifi_mgmt_disconnect_work(struct k_work *work)
{
	int status = 0;
	struct erpc_wifi_data *dev;
	WIFIReturnCode_t ret;

	LOG_DBG("erpc_wifi_mgmt_disconnect_work");

	dev = CONTAINER_OF(work, struct erpc_wifi_data, disconnect_work);

	ret = WIFI_Disconnect();

	LOG_DBG("WIFI_Disconnect: %d", ret);

	if (ret != eWiFiSuccess) {
		// TODO - there an enumerated error code that can be used here?
		status = -1;
	}

	dev->state = WIFI_STATE_DISCONNECTED;

	wifi_mgmt_raise_disconnect_result_event(dev->net_iface, status);
	net_if_dormant_on(dev->net_iface);
}
/*
	Parameters returned from this function via wifi_iface_status:

	int state (see enum wifi_iface_state)
	unsigned int ssid_len
	char ssid[WIFI_SSID_MAX_LEN + 1]
	char bssid[WIFI_MAC_ADDR_LEN]
	enum wifi_frequency_bands band
	unsigned int channel
	enum wifi_iface_mode iface_mode
	enum wifi_link_mode link_mode
	enum wifi_wpa3_enterprise_type wpa3_ent_type
	enum wifi_security_type security
	enum wifi_mfp_options mfp
	int rssi
	unsigned char dtim_period
	unsigned short beacon_interval
	bool twt_capable
	int current_phy_tx_rate

	state				- data->state
	ssid_len			- data->drv_nwk_params.ucSSIDLength
	ssid				- data->drv_nwk_params.ucSSID
	bssid				- NOT POPULATED
	band				- wifi_chan_to_band(data->drv_nwk_params.ucChannel)
	channel				- dev->drv_nwk_params.ucChannel
	iface_mode			- NOT POPULATED
	link_mode			- NOT POPULATED
	wpa3_ent_type		- NOT POPULATED
	security			- drv_nwk_params.xSecurity
	mfp					- NOT POPULATED
	rssi				- NOT POPULATED
	dtim_period			- NOT POPULATED
	beacon_interval		- NOT POPULATED
	twt_capable			- NOT POPULATED
	current_phy_tx_rate	- NOT POPULATED
*/
int erpc_wifi_mgmt_iface_status(const struct device *dev,
			     struct wifi_iface_status *status)
{
	struct erpc_wifi_data *data = dev->data;
	
	status->state = data->state;
	status->ssid_len = data->drv_nwk_params.ucSSIDLength;
	memcpy(status->ssid, data->drv_nwk_params.ucSSID, 
		MIN(sizeof(status->ssid), status->ssid_len));
	status->channel = data->drv_nwk_params.ucChannel;
	status->security = drv_to_wifi_mgmt_sec(data->drv_nwk_params.xSecurity);
	status->band = wifi_chan_to_band(data->drv_nwk_params.ucChannel);

	return 0;
}

int erpc_wifi_mgmt_get_version(const struct device *dev,
				 struct wifi_version *params)
{
	struct erpc_wifi_data *data = dev->data;

	fw_version_get_driver_ver(data->fw_version_driver,
			 sizeof(data->fw_version_driver));

	params->drv_version = data->fw_version_driver;
	params->fw_version = NULL;

	return 0;
}

static enum offloaded_net_if_types erpc_wifi_offload_get_type(void)
{
	return L2_OFFLOADED_NET_IF_TYPE_WIFI;
}

static void erpc_wifi_iface_init(struct net_if *iface)
{
	erpc_wifi_socket_offload_init(iface);

	/* Not currently connected to a network */
	net_if_dormant_on(iface);
}

static void erpc_wifi_client_error_handler(erpc_status_t err, uint32_t func_id)
{
	if (err > 0) {
		/* See wifi_interface.hpp for list of function id's */
		LOG_ERR("eRPC client error - err: %d func_id: %d", err, func_id);
	}
}

#ifdef CONFIG_ERPC_TRANSPORT_UART
static void erpc_wifi_server_thread(void *arg1, void *unused1, void *unused2) 
{
	erpc_status_t err;
	struct erpc_wifi_data *data = (struct erpc_wifi_data *)arg1;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	while (1) {
		// TODO should we use erpc_server_run() instead? 
		err = erpc_server_poll(data->erpc_server);
		if (err == kErpcStatus_Success) {
			if (data->driver_state == ERPC_WIFI_DRIVER_INITIALIZING) {
				if (data->reset_msg_received) {
					data->reset_msg_received = false;
					k_sem_give(&erpc_wifi_driver_data.sem_if_ready);
				}
			}
		} else {
			LOG_ERR("erpc_server_poll - err: %d", err);
		}
	}
}

// TODO- rename this erpc_wifi_server_event_handler and regenerate service files
void ra_erpc_server_event_handler(const ra_erp_server_event_t * event)
{
	struct erpc_wifi_data *data = &erpc_wifi_driver_data;
	struct in_addr ip_addr;
	struct in_addr gw_addr;
	struct in_addr netmask;

	switch (event->event_id) {
		case eDeviceReset:
		{
			LOG_DBG("eDeviceReset");

			data->reset_msg_received = true;
			break;
		}
		case eNetworkInterfaceIPAssigned:
		{
			LOG_DBG("eNetworkInterfaceIPAssigned");

			ip_addr.s_addr = event->event_data.xConfig.xIPAddress.ulAddress[0];
			gw_addr.s_addr = event->event_data.xConfig.xGateway.ulAddress[0];
			netmask.s_addr = event->event_data.xConfig.xNetMask.ulAddress[0];

			net_if_ipv4_addr_add(data->net_iface, &ip_addr, NET_ADDR_DHCP, 0);
			net_if_ipv4_set_gw(data->net_iface, &gw_addr);
			net_if_ipv4_set_netmask_by_addr(data->net_iface, &ip_addr, &netmask);
			break;
		}
		case eNetworkInterfaceDown:
		{
			LOG_DBG("eNetworkInterfaceDown");

			data->state = WIFI_STATE_DISCONNECTED;

			wifi_mgmt_raise_disconnect_result_event(data->net_iface, 0);
			net_if_dormant_on(data->net_iface);
			break;
		}
		default:
			LOG_DBG("event_id: %d", event->event_id);
			break;
	}
}
#endif

static const struct wifi_mgmt_ops erpc_wifi_mgmt_ops = {
	.scan		   		= erpc_wifi_mgmt_scan,
	.connect	   		= erpc_wifi_mgmt_connect,
	.disconnect	   		= erpc_wifi_mgmt_disconnect,
	.iface_status		= erpc_wifi_mgmt_iface_status,
	.get_version        = erpc_wifi_mgmt_get_version,
#ifdef CONFIG_WIFI_ERPC_WIFI_SOFTAP_SUPPORT
	.ap_enable     		= NULL,
	.ap_disable    		= NULL,
	.ap_sta_disconnect 	= NULL,
	.ap_config_params   = NULL,
#endif
};

static const struct net_wifi_mgmt_offload erpc_wifi_api = {
	.wifi_iface.iface_api.init = erpc_wifi_iface_init,
	.wifi_iface.get_type = erpc_wifi_offload_get_type,
	.wifi_mgmt_api = &erpc_wifi_mgmt_ops,
};

static int erpc_wifi_init(const struct device *dev);

NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, erpc_wifi_init, NULL,
	&erpc_wifi_driver_data, NULL,
	CONFIG_WIFI_INIT_PRIORITY, &erpc_wifi_api,
	ERPC_WIFI_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));

static int erpc_wifi_init(const struct device *dev)
{
	erpc_transport_t transport;
	erpc_mbf_t mbf;
	erpc_client_t client_manager;
#ifdef CONFIG_ERPC_TRANSPORT_UART
	erpc_service_t service;
	erpc_transport_t arbitrator;
#endif

	struct erpc_wifi_data *data = dev->data;

	LOG_DBG("initializing...");

	data->driver_state = ERPC_WIFI_DRIVER_INITIALIZING;
	data->reset_msg_received = false;

	k_work_init(&data->scan_work, erpc_wifi_mgmt_scan_work);
	k_work_init(&data->connect_work, erpc_wifi_mgmt_connect_work);
	k_work_init(&data->disconnect_work, erpc_wifi_mgmt_disconnect_work);

	k_sem_init(&data->sem_if_ready, 0, 1);

	/* Initialize the work queue */
	k_work_queue_start(&data->workq, erpc_wifi_workq_stack,
			   K_KERNEL_STACK_SIZEOF(erpc_wifi_workq_stack),
			   K_PRIO_COOP(CONFIG_WIFI_ERPC_WIFI_WORKQ_THREAD_PRIORITY),
			   NULL);
	k_thread_name_set(&data->workq.thread, "erpc_wifi_workq");

	/* Initialize the eRPC client infrastructure */	
	transport = erpc_wifi_transport_init();
	if (transport == NULL) {
		LOG_ERR("Failed to initialize eRPC transport");
		return -ENODEV;
	}

	mbf = erpc_mbf_dynamic_init();
	if (mbf == NULL) {
		LOG_ERR("Failed to initialize eRPC message buffer factory");
		return -ENODEV;
	}

#ifdef CONFIG_ERPC_TRANSPORT_UART
	client_manager = erpc_arbitrated_client_init(transport, mbf, &arbitrator);
#else
	client_manager = erpc_client_init(transport, mbf);
#endif

	if (client_manager == NULL) {
		LOG_ERR("Failed to initialize eRPC client");
		return -ENODEV;
	}

	/* Initialize eRPC client interface */
	erpc_client_set_error_handler(client_manager, erpc_wifi_client_error_handler);
	initwifi_client(client_manager);

#ifdef CONFIG_ERPC_TRANSPORT_UART
	/* Initialize eRPC server interface */
	data->erpc_server = erpc_server_init(arbitrator, mbf);
	service = create_wifi_async_service();

	/* Add custom service implementation to the server */
	erpc_add_service_to_server(data->erpc_server, service);

	k_thread_create(&erpc_wifi_server_thread_data,
					erpc_wifi_server_thread_stack,
					CONFIG_WIFI_ERPC_WIFI_SERVER_THREAD_STACK_SIZE,
					erpc_wifi_server_thread,
					data, NULL, NULL,
					CONFIG_WIFI_ERPC_WIFI_SERVER_THREAD_PRIORITY, 0,
					K_NO_WAIT);
#endif

	data->net_iface = NET_IF_GET(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)), 0);

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	int err = 0;
	struct gpio_dt_spec wifi_reset = GPIO_DT_SPEC_GET(DT_DRV_INST(0), reset_gpios);

	if (!gpio_is_ready_dt(&wifi_reset)) {
		LOG_ERR("Error: failed to configure wifi_reset %s pin %d", wifi_reset.port->name,
			wifi_reset.pin);
		return -EIO;
	}

	/* Set wifi_reset as output and activate reset */
	err = gpio_pin_configure_dt(&wifi_reset, GPIO_OUTPUT_ACTIVE);
	if (err) {
		LOG_ERR("Error %d: failed to configure wifi_reset %s pin %d", err,
			wifi_reset.port->name, wifi_reset.pin);
		return err;
	}

	k_sleep(K_MSEC(DT_INST_PROP_OR(0, reset_assert_duration_ms, 0)));

	/* Release the device from reset */
	err = gpio_pin_configure_dt(&wifi_reset, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}

	/* We can either wait a fixed amount of time for the RA6Wx device to 
	   finish booting or we can wait for it to send us the reset complete
	   eveent. At present, the SPI interface does not support sending the
	   reset complete event so we have to just wait for boot to complete. */
	#if DT_INST_NODE_HAS_PROP(0, boot_duration_ms)
		k_sleep(K_MSEC(DT_INST_PROP_OR(0, boot_duration_ms, 0)));
	#else
		/* 
	   	   While we are waiting for this sempahore the erpc_wifi_server_thread
	   	   is running and calling erpc_server_poll to check for any incoming
	   	   message. When a valid message is received the ra_erpc_server_event_handler
	   	   function is called. The eRPC middleware sets the nestingDetection flag
	   	   to true just before calling the hanlder and sets it to false when execution
	   	   of the handler is complete and it has returned. 
		   
	   	   When the handler receives the eDeviceReset event it gives the 
	   	   sem_if_ready sempaphore. This causes the thread running the handler
	   	   to immediately suspend and the RTOS starts running this init function
	   	   once again. It continues through this init function and starts running 
	   	   main. In main the application calls a driver function, however this call
	   	   fails as when it calls the associated eRPC function a nesting error occurs
	   	   as the ra_erpc_server_event_handler has not yet had time to finish and so
	   	   the nestingDetection flag is still true...

	   	   According to the Zephyr documentation, the system thread running this init
	   	   function has the highest priority and therefore we can simply increase the
	   	   priority of the erpc_wifi_server_thread to resolve this issue:
	   	   https://docs.zephyrproject.org/latest/kernel/services/threads/system_threads.html 
		*/
		err = k_sem_take(&data->sem_if_ready, K_MSEC(CONFIG_WIFI_ERPC_WIFI_RESET_TIMEOUT));
		if (err) {
			return err;
		}

		data->driver_state = ERPC_WIFI_DRIVER_INITIALIZED;
	#endif
#endif /* DT_INST_NODE_HAS_PROP(0, reset_gpios) */

	LOG_DBG("complete!");

	return 0;
}
