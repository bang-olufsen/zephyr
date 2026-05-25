#include <zephyr/net/socket_offload.h>
#include <zephyr/net/socket.h>
#include "erpc_wifi.h"
#include "c_wifi_host_to_ra_client.h"
#include "c_wifi_ra_to_host_client.h"
#include <c_wifi_host_to_ra_client.h>

/* * The Offload Function (Using Compact Struct)
 */
static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	WIFIReturnCode_t server_status;
	uint8_t actual_count = 0;
	int ret = EAI_SYSTEM; // Default to system error
	struct zsock_addrinfo *head = NULL;
	struct zsock_addrinfo *prev = NULL;
	WIFIIPAddress_t *result;
	;
	// 1. CRITICAL: Initialize output pointer to NULL to prevent Bus Fault on error
	*res = NULL;

	result = malloc(DNS_MAX_ADDRESSES * sizeof(WIFIIPAddress_t));
	// 2. Call the eRPC generated client stub
	// The addresses buffer is filled by the eRPC framework.
	erpc_wifi_lock();
	server_status = ra6w1_dns_getaddrinfo(node, result, DNS_MAX_ADDRESSES, &actual_count);
	erpc_wifi_unlock();

	// 3. Check eRPC Transport Error
	if (server_status != eWiFiSuccess) {
		// Assume non-success means a transport error or unrecoverable LwIP error
		return EAI_SYSTEM;
	}

	// 4. Check LwIP Error (The server returns the LwIP error code in WIFIReturnCode_t)
	// NOTE: If your server uses a dedicated error_code field, you must check that instead.
	if (server_status < 0) {
		// If the return code is a negative LwIP error, map it to EAI_FAIL
		return EAI_FAIL;
	}

	// 5. Check for No Results Found
	if (actual_count == 0) {
		return EAI_NODATA;
	}

	// 6. Build the Zephyr linked list from the data received over eRPC
	for (uint8_t i = 0; i < actual_count; i++) {
		WIFIIPAddress_t *ip_addr_t = &result[i];

		// Allocate space for the node and the sockaddr struct
		struct zsock_addrinfo *ai = k_calloc(1, sizeof(struct zsock_addrinfo));
		struct sockaddr_in *addr4 = k_calloc(1, sizeof(struct sockaddr_in));

		if (!ai || !addr4) {
			ret = EAI_MEMORY;
			goto cleanup; // Go to cleanup to free partially built list
		}

		// Fill data for IPv4 (based on your implementation supporting V4)
		if (ip_addr_t->xType == eWiFiIPAddressTypeV4) {
			ai->ai_family = AF_INET;
			ai->ai_socktype = SOCK_STREAM; // Assuming SOCK_STREAM for now
			ai->ai_addrlen = sizeof(struct sockaddr_in);
			ai->ai_addr = (struct sockaddr *)addr4;

			addr4->sin_family = AF_INET;

			// Copy the raw 4 bytes of the IP address from the ulAddress array
			// This assumes the server copied the address in Network Byte Order.
			memcpy(&addr4->sin_addr.s_addr, &ip_addr_t->ulAddress[0], sizeof(uint32_t));
		}

		// Link the list
		if (prev) {
			prev->ai_next = ai;
		} else {
			head = ai;
		}
		prev = ai;
	}

	*res = head;
	return 0;

cleanup:

	// Note: No need to free 'result' as it was dynamically allocated.
	free(result);
	return ret;
}
/* * 2. The Free Function
 */
static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	struct zsock_addrinfo *p, *next;

	// We must manually free the structs we k_calloc'd above
	for (p = res; p != NULL; p = next) {
		next = p->ai_next;
		if (p->ai_addr) {
			k_free(p->ai_addr);
		}
		k_free(p);
	}
}

/* * 3. Registration (Call this in your init!)
 */
static const struct socket_dns_offload dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};

void erpc_wifi_dns_offload_init(void)
{
	printk("Registering eRPC WiFi DNS offload\n");
	socket_offload_dns_register(&dns_ops);
}
