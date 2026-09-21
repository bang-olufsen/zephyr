/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-in-the-loop ztest for the Renesas eRPC Wi-Fi driver.
 *
 * This exercises the real host -> server send path, no sample application
 * required:
 *
 *   net_mgmt(NET_REQUEST_WIFI_CONNECT)
 *     -> erpc_wifi_mgmt_connect()            (zephyr host driver)
 *     -> erpc_wifi_send_cmd(ERPC_WIFI_AP_CONNECT_CMD)
 *     -> WIFI_ConnectAP()                    (RA6W1 ra_wifi_erpc_server)
 *
 * Requires: EK-RA6M4 + RRQ61051 mikroBUS SPI shield, an RA6W1 flashed with
 * ra_wifi_erpc_server, and a reachable AP matching the credentials below.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(erpc_wifi_connect_test, LOG_LEVEL_INF);

/* EDIT THESE to match your AP before flashing. */
#define TEST_WIFI_SSID "Anjana"
#define TEST_WIFI_PSK  "12345678"

#define WIFI_EVENT_MASK   (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)
/* Must exceed the driver worst case: 3 connect attempts at 20 s plus the wake handshake. */
#define CONNECT_TIMEOUT   K_SECONDS(75)
#define DISCONNECT_TIMEOUT K_SECONDS(15)

struct erpc_wifi_connect_test_fixture {
	struct net_if *iface;
	struct net_mgmt_event_callback cb;
	struct k_sem connect_done;
	struct k_sem disconnect_done;
	int connect_status;
	int disconnect_status;
	bool connected;
};

static struct erpc_wifi_connect_test_fixture test_fixture;

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			       struct net_if *iface)
{
	struct erpc_wifi_connect_test_fixture *fixture = &test_fixture;
	const struct wifi_status *status = (const struct wifi_status *)cb->info;

	ARG_UNUSED(iface);

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		fixture->connect_status = status->status;
		LOG_INF("connect result: %d", status->status);
		k_sem_give(&fixture->connect_done);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		fixture->disconnect_status = status->status;
		LOG_INF("disconnect result: %d", status->status);
		k_sem_give(&fixture->disconnect_done);
		break;
	default:
		break;
	}
}

static void *erpc_wifi_connect_test_setup(void)
{
	struct erpc_wifi_connect_test_fixture *fixture = &test_fixture;

	k_sem_init(&fixture->connect_done, 0, 1);
	k_sem_init(&fixture->disconnect_done, 0, 1);

	fixture->iface = net_if_get_first_wifi();
	zassert_not_null(fixture->iface, "No Wi-Fi interface found - is the shield attached "
					"and the erpc_wifi driver enabled?");

	net_mgmt_init_event_callback(&fixture->cb, wifi_event_handler, WIFI_EVENT_MASK);
	net_mgmt_add_event_callback(&fixture->cb);

	/* Give the eRPC transport / RA6W1 server time to come up after reset. */
	k_sleep(K_SECONDS(2));

	return fixture;
}

static void erpc_wifi_connect_test_before(void *f)
{
	struct erpc_wifi_connect_test_fixture *fixture = f;

	k_sem_reset(&fixture->connect_done);
	k_sem_reset(&fixture->disconnect_done);
	fixture->connect_status = -1;
	fixture->disconnect_status = -1;
}

static void erpc_wifi_connect_test_teardown(void *f)
{
	struct erpc_wifi_connect_test_fixture *fixture = f;

	net_mgmt_del_event_callback(&fixture->cb);
}

/* The Wi-Fi interface must exist before any command can be sent to the server. */
ZTEST_F(erpc_wifi_connect_test, test_01_wifi_iface_present)
{
	zassert_not_null(fixture->iface, "Wi-Fi interface missing");
	zassert_true(net_if_is_admin_up(fixture->iface), "Wi-Fi interface is not admin-up");
}

/* Sends the connect command over eRPC and waits for the async result event. */
ZTEST_F(erpc_wifi_connect_test, test_02_connect_to_ap)
{
	struct wifi_connect_req_params params = {
		.ssid = (const uint8_t *)TEST_WIFI_SSID,
		.ssid_length = sizeof(TEST_WIFI_SSID) - 1,
		.psk = (const uint8_t *)TEST_WIFI_PSK,
		.psk_length = sizeof(TEST_WIFI_PSK) - 1,
		.security = WIFI_SECURITY_TYPE_PSK,
		.channel = WIFI_CHANNEL_ANY,
		.band = WIFI_FREQ_BAND_2_4_GHZ,
		.mfp = WIFI_MFP_OPTIONAL,
	};
	int ret;

	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, fixture->iface, &params, sizeof(params));
	zassert_equal(ret, 0, "NET_REQUEST_WIFI_CONNECT failed (%d)", ret);

	ret = k_sem_take(&fixture->connect_done, CONNECT_TIMEOUT);
	zassert_equal(ret, 0, "Timed out waiting for NET_EVENT_WIFI_CONNECT_RESULT");

	zassert_equal(fixture->connect_status, WIFI_STATUS_CONN_SUCCESS,
		      "Connect failed with status %d", fixture->connect_status);

	fixture->connected = true;
}

/* Reads back status from the server to confirm the association really happened. */
ZTEST_F(erpc_wifi_connect_test, test_03_iface_status_matches_ap)
{
	struct wifi_iface_status status = {0};
	int ret;

	if (!fixture->connected) {
		ztest_test_skip();
	}

	ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, fixture->iface, &status, sizeof(status));
	zassert_equal(ret, 0, "NET_REQUEST_WIFI_IFACE_STATUS failed (%d)", ret);

	zassert_equal(status.state, WIFI_STATE_COMPLETED, "Unexpected Wi-Fi state %d", status.state);
	zassert_equal(status.ssid_len, sizeof(TEST_WIFI_SSID) - 1, "Unexpected SSID length %d",
		      status.ssid_len);
	zassert_mem_equal(status.ssid, TEST_WIFI_SSID, sizeof(TEST_WIFI_SSID) - 1,
			  "Connected to the wrong SSID");
}

/* Leaves the radio in a clean state and exercises the disconnect command path. */
ZTEST_F(erpc_wifi_connect_test, test_04_disconnect_from_ap)
{
	int ret;

	if (!fixture->connected) {
		ztest_test_skip();
	}

	ret = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, fixture->iface, NULL, 0);
	zassert_equal(ret, 0, "NET_REQUEST_WIFI_DISCONNECT failed (%d)", ret);

	ret = k_sem_take(&fixture->disconnect_done, DISCONNECT_TIMEOUT);
	zassert_equal(ret, 0, "Timed out waiting for NET_EVENT_WIFI_DISCONNECT_RESULT");

	fixture->connected = false;
}

ZTEST_SUITE(erpc_wifi_connect_test, NULL, erpc_wifi_connect_test_setup,
	    erpc_wifi_connect_test_before, NULL, erpc_wifi_connect_test_teardown);
