#include "erpc_wifi_cmd_process_handlers.h"
#include "erpc_wifi_cmd.h"
#include "erpc_wifi.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "zephyr/net/socket.h"

LOG_MODULE_REGISTER(erpc_wifi_cmd_socket_process, CONFIG_WIFI_LOG_LEVEL);

/* Forward declarations from erpc_wifi_socket_offload.c */
extern int32_t ra6w1_bind(int32_t fd, const struct ra_erpc_sockaddr *addr, uint32_t addrlen);
extern int32_t ra6w1_connect(int32_t fd, const struct ra_erpc_sockaddr *addr, uint32_t addrlen);
extern int32_t ra6w1_listen(int32_t fd, int32_t backlog);
extern int32_t ra6w1_accept(int32_t fd, struct ra_erpc_sockaddr *addr, uint32_t *addrlen);
extern int32_t ra6w1_recv(int32_t fd, uint8_t *buff, uint32_t len, int32_t flags);
extern int32_t ra6w1_recvfrom(int32_t fd, uint8_t *buff, uint32_t len, int32_t flags,
							   struct ra_erpc_sockaddr *src_addr, uint32_t *addrlen);
extern int32_t ra6w1_send(int32_t fd, const uint8_t *buff, uint32_t len, int32_t flags);
extern int32_t ra6w1_sendto(int32_t fd, const uint8_t *buff, uint32_t len, int32_t flags,
							 const struct ra_erpc_sockaddr *addr, uint32_t addrlen);
extern int32_t ra6w1_socket(int32_t domain, int32_t type, int32_t protocol);
extern int32_t ra6w1_close(int32_t fd);
extern int32_t ra6w1_getsockopt(int32_t fd, int32_t level, int32_t optname,
								uint32_t *optval, uint32_t *optlen);
extern int32_t ra6w1_setsockopt(int32_t fd, int32_t level, int32_t optname,
								const uint32_t *optval, uint32_t optlen);
extern uint32_t get_socket_events(int32_t fd);

/* Socket operation handlers - executed in queue handler thread */

static int erpc_wifi_bind_msg_process(void *data)
{
	erpc_wifi_msg_bind_t *msg = (erpc_wifi_msg_bind_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_bind(msg->fd, &msg->addr_erpc_wifi, msg->addr_erpc_wifi.sa_len);
}

static int erpc_wifi_connect_msg_process(void *data)
{
	erpc_wifi_msg_connect_t *msg = (erpc_wifi_msg_connect_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_connect(msg->fd, &msg->addr_erpc_wifi, msg->addr_erpc_wifi.sa_len);
}

static int erpc_wifi_listen_msg_process(void *data)
{
	erpc_wifi_msg_listen_t *msg = (erpc_wifi_msg_listen_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_listen(msg->fd, msg->backlog);
}

static int erpc_wifi_accept_msg_process(void *data)
{
	erpc_wifi_msg_accept_t *msg = (erpc_wifi_msg_accept_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_accept(msg->fd, msg->addr_erpc_wifi, msg->addrlen);
}

static int erpc_wifi_recv_msg_process(void *data)
{
	erpc_wifi_msg_recv_t *msg = (erpc_wifi_msg_recv_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_recv(msg->fd, msg->buff, msg->buff_size, msg->flags);
}

static int erpc_wifi_recvfrom_msg_process(void *data)
{
	erpc_wifi_msg_recvfrom_t *msg = (erpc_wifi_msg_recvfrom_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_recvfrom(msg->fd, msg->buff, msg->buff_size, msg->flags,
						 msg->src_addr, msg->src_addrlen);
}

static int erpc_wifi_send_msg_process(void *data)
{
	erpc_wifi_msg_send_t *msg = (erpc_wifi_msg_send_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_send(msg->fd, msg->buff, msg->buff_size, msg->flags);
}

static int erpc_wifi_sendto_msg_process(void *data)
{
	erpc_wifi_msg_sendto_t *msg = (erpc_wifi_msg_sendto_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_sendto(msg->fd, msg->buff, msg->buff_size, msg->flags,
					   msg->addr_erpc_wifi, msg->addr_erpc_wifi->sa_len);
}

static int erpc_wifi_socket_create_msg_process(void *data)
{
	erpc_wifi_msg_socket_create_t *msg = (erpc_wifi_msg_socket_create_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_socket(msg->domain, msg->type, msg->protocol);
}

static int erpc_wifi_socket_close_msg_process(void *data)
{
	int *fd = (int *)data;
	if (!fd) return -EINVAL;

	return ra6w1_close(*fd);
}

static int erpc_wifi_socket_getopt_msg_process(void *data)
{
	erpc_wifi_msg_sockgetopt_t *msg = (erpc_wifi_msg_sockgetopt_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_getsockopt(msg->fd, msg->level, msg->optname, msg->optval, msg->optlen);
}

static int erpc_wifi_socket_setopt_msg_process(void *data)
{
	erpc_wifi_msg_socksetopt_t *msg = (erpc_wifi_msg_socksetopt_t *)data;
	if (!msg) return -EINVAL;

	return ra6w1_setsockopt(msg->fd, msg->level, msg->optname, msg->optval, msg->optlen);
}

static int erpc_wifi_socket_event_msg_process(void *data)
{
	int *fd = (int *)data;
	if (!fd) return -EINVAL;

	return get_socket_events(*fd);
}

static int erpc_wifi_wait_ra_awake_msg_process(void *data)
{
	/* Placeholder - wake management handled separately */
	ARG_UNUSED(data);
	return 0;
}

/* Handler registration function */
int erpc_wifi_cmd_socket_handlers_init(void)
{
	erpc_wifi_register_cmd_handler(ERPC_WIFI_BIND_CMD, erpc_wifi_bind_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_CONNECT_CMD, erpc_wifi_connect_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_LISTEN_CMD, erpc_wifi_listen_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_ACCEPT_CMD, erpc_wifi_accept_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_RECV_CMD, erpc_wifi_recv_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_RECVFROM_CMD, erpc_wifi_recvfrom_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SEND_CMD, erpc_wifi_send_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SENDTO_CMD, erpc_wifi_sendto_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_CREATE_CMD, erpc_wifi_socket_create_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_CLOSE_CMD, erpc_wifi_socket_close_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_GETOPT_CMD, erpc_wifi_socket_getopt_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_SETOPT_CMD, erpc_wifi_socket_setopt_msg_process);

	erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD, erpc_wifi_socket_event_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_WAIT_RA_AWAKE_CMD, erpc_wifi_wait_ra_awake_msg_process);

	LOG_INF("Socket handlers initialized");

	return 0;
}
