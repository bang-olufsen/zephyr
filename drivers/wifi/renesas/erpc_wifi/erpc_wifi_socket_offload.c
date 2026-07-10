/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
/* PMGR DPM job ids used by RA6W1 socket shim */
#ifndef ERPC_PMGR_JOB_ID_SEND
#define ERPC_PMGR_JOB_ID_SEND (1U)
#endif
#ifndef ERPC_PMGR_JOB_ID_RECV
#define ERPC_PMGR_JOB_ID_RECV (2U)
#endif

LOG_MODULE_REGISTER(erpc_wifi_socket_offload, CONFIG_WIFI_LOG_LEVEL);

void erpc_wifi_lock(void);
void erpc_wifi_unlock(void);
uint32_t get_socket_events(int32_t fd);
extern bool erpc_wifi_ps_is_enabled(void);

extern void erpc_wifi_gpio_trigger_wakeup(void);


static atomic_t g_erpc_tx_blocked;
static atomic_t g_erpc_tx_block_timeout_ms;
void erpc_wifi_socket_tx_block_set(bool enable, uint32_t timeout_ms)
{
	atomic_set(&g_erpc_tx_blocked, enable ? 1 : 0);
	atomic_set(&g_erpc_tx_block_timeout_ms, (atomic_val_t)timeout_ms);
}

extern int32_t ra6w1_pmgr_dpm_is_wakeup(void);
extern int32_t ra6w1_pmgr_dpm_is_enabled(void);
extern int32_t ra6w1_pmgr_dpm_wakeup_done(uint32_t job_id);
extern int32_t ra6w1_pmgr_dpm_rcv_ready_set(uint32_t job_id);
//extern int32_t ra6w1_pmgr_dpm_job_name_set(uint32_t job_id, const char *job_name);
extern int32_t ra6w1_pmgr_add_sleep_constraint(uint32_t constraint);
extern int erpc_wifi_transport_slave_ready(void);

/* Forward declaration for pmgr_ram_hold used in erpc_wifi_ensure_awake_tx */
static inline int pmgr_ram_hold(void);

/*
 * erpc_wifi_ensure_awake_rx - Wait for RA module to assert slave-ready for an
 * incoming RX operation.  Unlike erpc_wifi_ensure_awake_tx() this function
 * does NOT pulse the wakeup GPIO because the RA6W1 already woke itself when it
 * received the TCP packet and will assert slave-ready to notify the host.
 * Pulsing the GPIO on an RX path is incorrect: the direction of wakeup is
 * RA→Host, not Host→RA.
 */
static int erpc_wifi_ensure_awake_rx(uint32_t job_id)
{
	if (!erpc_wifi_ps_is_enabled()) {
		return 0;
	}

	int64_t start = k_uptime_get();
	/* Use same timeout as TX path default */
	int64_t tmo_ms = 5000;

	for (;;) {
		int sr = erpc_wifi_transport_slave_ready();

		if (sr == 1) {
			int stable = 0;

			for (int i = 0; i < 3; i++) {
				if (erpc_wifi_transport_slave_ready() == 1) {
					stable++;
				}
				k_msleep(3);
			}

			if (stable < 3) {
				k_msleep(5);
				continue;
			}

			k_msleep(500);
			if (pmgr_ram_hold() < 0) {
				k_msleep(50);
				continue;
			}

			erpc_wifi_lock();
			int32_t rc = ra6w1_pmgr_dpm_rcv_ready_set(job_id);
			erpc_wifi_unlock();
			if (rc < 0) {
				LOG_WRN("RX wake prep failed: rcv_ready_set rc=%d", rc);
				k_msleep(50);
				continue;
			}
			return 0;
		}

		if ((k_uptime_get() - start) > tmo_ms) {
			LOG_WRN("RX wait timeout: slave-ready not asserted after %lldms", tmo_ms);
			return -EAGAIN;
		}

		k_msleep(10);
	}
}

static int erpc_wifi_ensure_awake_tx(uint32_t job_id)
{
	if (atomic_get(&g_erpc_tx_blocked) == 0) {
		if (!erpc_wifi_ps_is_enabled() || erpc_wifi_transport_slave_ready() == 1) {
			LOG_DBG("TX wake skipped (tx_blocked=0, job=%u)", job_id);
			return 0;
		}

		LOG_INF("TX wake override (job=%u): tx_blocked=0 but slave-ready=0 while PS enabled",
			job_id);
	}

	int64_t start = k_uptime_get();
	uint32_t tmo = (uint32_t)atomic_get(&g_erpc_tx_block_timeout_ms);
	int64_t last_pulse = start;

	/* Proactively pulse Wakeup GPIO */
	erpc_wifi_gpio_trigger_wakeup();

	for (;;) {
		int32_t awake;
		int sr = erpc_wifi_transport_slave_ready();

		/* SPI slave-ready going active means RA side is up enough for eRPC traffic. */
		if (sr == 1) {
			int stable = 0;

			for (int i = 0; i < 3; i++) {
				if (erpc_wifi_transport_slave_ready() == 1) {
					stable++;
				}
				k_msleep(3);
			}

			if (stable < 3) {
				k_msleep(5);
				continue;
			}

			k_msleep(500);

			if (pmgr_ram_hold() < 0) {
				k_msleep(50);
				continue;
			}
			
			/* Tell PMGR this module has pending I/O work only after wake is visible. */
			erpc_wifi_lock();
			int32_t rc = ra6w1_pmgr_dpm_rcv_ready_set(job_id);
			erpc_wifi_unlock();
			if (rc < 0) {
				LOG_WRN("TX wake prep failed: rcv_ready_set rc=%d", rc);
				k_msleep(50);
				continue;
			}
			// /* Re-arm sleep timer so module returns to DPM after this I/O. */
			// erpc_wifi_ps_notify_wakeup();
			return 0;
		}

		/* Some boards require repeated wake pulses while RA6W1 is in DPM. */
		if ((k_uptime_get() - last_pulse) > 300) {
			erpc_wifi_gpio_trigger_wakeup();
			last_pulse = k_uptime_get();
		}

		if (tmo > 0U && (k_uptime_get() - start) > (int64_t)tmo) {
			LOG_WRN("TX wake timeout job=%u tmo=%u", job_id, tmo);
			return -EAGAIN;
		}

		//k_msleep(50);
		k_msleep(10);
	}
}
extern void erpc_wifi_lock(void);
extern void erpc_wifi_unlock(void);

#ifndef PMGR_CONSTRAINT_SLEEP_PROHIBITED
#define PMGR_CONSTRAINT_SLEEP_PROHIBITED (1U << 0)
#endif
#ifndef PMGR_CONSTRAINT_POWER_RAM
#define PMGR_CONSTRAINT_POWER_RAM (1U << 2)
#endif

static int pmgr_dpm_cached_enabled(void)
{
	static int cached = -1;

	if (cached >= 0) {
		return cached;
	}

	erpc_wifi_lock();
	int32_t en = ra6w1_pmgr_dpm_is_enabled();
	erpc_wifi_unlock();
	cached = (en == 1) ? 1 : 0;
	return cached;
}

static inline int pmgr_ram_hold(void)
{
	if (pmgr_dpm_cached_enabled()) {
		erpc_wifi_lock();
	//	(void)ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);
        int32_t rc = ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);
		if (rc < 0) {
			LOG_ERR("pmgr_ram_hold failed: %d", rc);
			erpc_wifi_unlock();
			return rc;
		}
		erpc_wifi_unlock();
	}

	return 0;
}

void erpc_wifi_pmgr_ram_release(uint32_t job_id)
{
	if (pmgr_dpm_cached_enabled()) {
		erpc_wifi_lock();
		int32_t rc = ra6w1_pmgr_remove_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);
		if (rc < 0) {
			LOG_ERR("pmgr_ram_release failed: %d", rc);
		}
		if (job_id != 0) {
			(void)ra6w1_pmgr_dpm_wakeup_done(job_id);
		}
		erpc_wifi_unlock();
	}
}

/*
 * erpc_wifi_wake_for_tx - exported wrapper for callers outside this file
 * (e.g. DNS offload) that need to perform a host-initiated eRPC operation
 * while the RA6W1 may be in DPM sleep.
 */
int erpc_wifi_wake_for_tx(void)
{
	return erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
}

#include <zephyr/sys/atomic.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <zephyr/posix/fcntl.h>

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include "sockets_internal.h"

#include "erpc_wifi.h"
#include "c_wifi_host_to_ra_client.h"
#include "c_wifi_ra_to_host_client.h"

#include <zephyr/net/socket.h>

#define ERPC_WIFI_AF_INET  2
#define ERPC_WIFI_AF_INET6 10

#define ERPC_WIFI_SOL_SOCKET 0xfff

#define ERPC_WIFI_SO_DEBUG        0x0001
#define ERPC_WIFI_SO_REUSEADDR    0x0004
#define ERPC_WIFI_SO_TYPE         0x1008
#define ERPC_WIFI_SO_ERROR        0x1007
#define ERPC_WIFI_SO_DONTROUTE    0x0010
#define ERPC_WIFI_SO_BROADCAST    0x0020
#define ERPC_WIFI_SO_SNDBUF       0x1001
#define ERPC_WIFI_SO_RCVBUF       0x1002
#define ERPC_WIFI_SO_KEEPALIVE    0x0008
#define ERPC_WIFI_SO_OOBINLINE    0x0100
#define ERPC_WIFI_SO_LINGER       0x0080
#define ERPC_WIFI_SO_REUSEPORT    0x0200
#define ERPC_WIFI_SO_RCVLOWAT     0x1004
#define ERPC_WIFI_SO_SNDLOWAT     0x1003
#define ERPC_WIFI_SO_RCVTIMEO     0x1006
#define ERPC_WIFI_SO_SNDTIMEO     0x1005
#define ERPC_WIFI_SO_BINDTODEVICE 0x100b
#define ERPC_WIFI_SO_ACCEPTCONN   0x0002

#define ERPC_WIFI_MAX_SOCKETS 4

// Event flags that match what you'll return
#define SOCKET_EVENT_RX    0x01 // POLLIN - Data available to read
#define SOCKET_EVENT_TX    0x02 // POLLOUT - Ready to write
#define SOCKET_EVENT_ERR   0x04 // POLLERR - Error condition
#define SOCKET_EVENT_CLOSE 0x08 // POLLHUP - Connection closed

#if 0
struct erpc_wifi_socket {
    int fd;          // remote FD
    int zfd;          // Zephyr FD
    bool in_use;

    bool waiting;
    struct k_poll_signal poll_signal;  // Add this
    short poll_events;                 // Events poll is waiting for
    short triggered_events;            // Events that occurred
    int type;        // POSIX socket type (e.g. SOCK_STREAM)
    uint16_t bound_port;   // Local port from bind() (host order)
    bool tcp_dpm_filter_set; // true if TCP DPM wake filter installed

};
#endif
struct erpc_wifi_socket {
	int zfd; // Zephyr FD
	int fd;  // remote FD
	int family;
	int type;
	int protocol;
	bool in_use;

	bool waiting;
	struct k_poll_signal poll_signal; // Add this
	//	short poll_events;                 // Events poll is waiting for
	short triggered_events; // Events that occurred
	/* poll bookkeeping */
	struct zsock_pollfd *pollfd; /* pointer stored in POLL_PREPARE */
	uint16_t poll_events;        /* events user requested in pollfd->events */
	uint16_t pending_revents;    /* events actually occurred, to be returned in POLL_UPDATE */

	struct k_sem read_sem;
	struct k_fifo fifo;
	
	struct k_work socket_connect_work;
	struct ra_erpc_sockaddr *addr_erpc_wifi;

	// erpc_service_t service;

	erpc_server_t server;
	erpc_client_t client;

	char addr_str[NET_IPV4_ADDR_LEN];
	int port;

	struct k_mutex *lock;    /* for ZFD_IOCTL_SET_LOCK */
	uint32_t flags;          /* O_NONBLOCK etc */
	uint16_t bound_port;     // Local port from bind() (host order)
	bool tcp_dpm_filter_set; // true if TCP DPM wake filter installed
	uint32_t recv_timeout_ms;
};

static struct erpc_wifi_socket sockets[ERPC_WIFI_MAX_SOCKETS];

K_THREAD_STACK_DEFINE(erpc_wifi_socket_poll_stack, 8192);
static struct k_thread erpc_wifi_socket_poll_thread_data;
static k_tid_t erpc_wifi_socket_poll_tid;
static struct k_sem poll_task_sem;
static void erpc_wifi_socket_poll_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int64_t last_not_ready_log = 0;

	while (1) {
		/* Determine if we need to poll for autonomous DPM wakeups */
		bool dpm_polling_needed = false;
		if (erpc_wifi_ps_is_enabled()) {
			for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
				if (sockets[i].in_use) {
					dpm_polling_needed = true;
					break;
				}
			}
		}

		/* Wait until signaled, or timeout if we need to poll DPM srdy */
		k_timeout_t wait_timeout = dpm_polling_needed ? K_MSEC(200) : K_FOREVER;
		(void)k_sem_take(&poll_task_sem, wait_timeout);

		bool activity = true;
		while (activity) {
			activity = false;
			for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
				struct erpc_wifi_socket *sock = &sockets[i];

				if (sock->in_use && (sock->waiting || erpc_wifi_ps_is_enabled())) {
					if (sock->waiting) {
						activity = true;
					}
					int srdy = erpc_wifi_transport_slave_ready();

					if (erpc_wifi_ps_is_enabled() && !srdy) {
						if (erpc_wifi_ps_sleep_is_sent()) {
							erpc_wifi_ps_confirm_sleep();
						}
						int64_t now = k_uptime_get();
						if ((now - last_not_ready_log) > 1000) {
							LOG_INF("poll wait: fd=%d srdy=0 waiting for module ready", sock->fd);
							last_not_ready_log = now;
						}
						continue;
					}
					if (erpc_wifi_ps_sleep_is_sent()) {
						if (erpc_wifi_ps_sleep_is_confirmed() && srdy) {
							LOG_INF("Autonomous DPM wakeup detected (fd=%d), processing events", sock->fd);
							erpc_wifi_ps_notify_wakeup();
						} else {
							/* Waiting for module to sleep. Don't add constraints. */
							continue;
						}
					}

					int __w = erpc_wifi_ensure_awake_rx(ERPC_PMGR_JOB_ID_RECV);
					if (__w != 0) {
						continue;
					}

					erpc_wifi_lock();
					uint32_t events = get_socket_events(sock->fd);
					erpc_wifi_unlock();

					erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);

					if (events == UINT32_MAX) {
						LOG_WRN("poll get_socket_events failed fd=%d; deferring", sock->fd);
						continue;
					}

					if (events != 0U) {
						LOG_INF("poll events: fd=%d ev=0x%x", sock->fd, events);
					}

					if (events & (SOCKET_EVENT_RX | SOCKET_EVENT_ERR |
						      SOCKET_EVENT_CLOSE)) {
						sock->triggered_events = (short)events;
						sock->waiting = false;
						k_poll_signal_raise(&sock->poll_signal,
								    (int)events);
					}
				}
			}

			if (activity) {
				k_msleep(200);
			}
		}
	}
}

static void ensure_poll_task_started(void)
{
	if (erpc_wifi_socket_poll_tid != NULL) {
		return;
	}

	erpc_wifi_socket_poll_tid = k_thread_create(
		&erpc_wifi_socket_poll_thread_data, erpc_wifi_socket_poll_stack,
		K_THREAD_STACK_SIZEOF(erpc_wifi_socket_poll_stack), erpc_wifi_socket_poll_task,
		NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(erpc_wifi_socket_poll_tid, "erpc_socket_poll");
}
static const struct socket_op_vtable erpc_wifi_socket_fd_op_vtable;

static int erpc_wifi_socket_level_from_posix(int level, int *level_erpc_wifi)
{
	switch (level) {
	case SOL_SOCKET:
		*level_erpc_wifi = ERPC_WIFI_SOL_SOCKET;
		break;
		case IPPROTO_TCP:
		*level_erpc_wifi = IPPROTO_TCP;
		break;
	default:
		return -EAFNOSUPPORT;
		break;
	}

	return 0;
}

static int erpc_wifi_socket_option_from_posix(int level, int optname, int *optname_erpc_wifi)
{
	if (level == IPPROTO_TCP) {
		*optname_erpc_wifi = optname;
		return 0;
	}
	
	switch (optname) {
	case SO_DEBUG:
		*optname_erpc_wifi = ERPC_WIFI_SO_DEBUG;
		break;
	case SO_REUSEADDR:
		*optname_erpc_wifi = ERPC_WIFI_SO_REUSEADDR;
		break;
	case SO_TYPE:
		*optname_erpc_wifi = ERPC_WIFI_SO_TYPE;
		break;
	case SO_ERROR:
		*optname_erpc_wifi = ERPC_WIFI_SO_ERROR;
		break;
	case SO_DONTROUTE:
		*optname_erpc_wifi = ERPC_WIFI_SO_DONTROUTE;
		break;
	case SO_BROADCAST:
		*optname_erpc_wifi = ERPC_WIFI_SO_BROADCAST;
		break;
	case SO_SNDBUF:
		*optname_erpc_wifi = ERPC_WIFI_SO_SNDBUF;
		break;
	case SO_RCVBUF:
		*optname_erpc_wifi = ERPC_WIFI_SO_RCVBUF;
		break;
	case SO_KEEPALIVE:
		*optname_erpc_wifi = ERPC_WIFI_SO_KEEPALIVE;
		break;
	case SO_OOBINLINE:
		*optname_erpc_wifi = ERPC_WIFI_SO_OOBINLINE;
		break;
	case SO_LINGER:
		*optname_erpc_wifi = ERPC_WIFI_SO_LINGER;
		break;
	case SO_REUSEPORT:
		*optname_erpc_wifi = ERPC_WIFI_SO_REUSEPORT;
		break;
	case SO_RCVLOWAT:
		*optname_erpc_wifi = ERPC_WIFI_SO_RCVLOWAT;
		break;
	case SO_SNDLOWAT:
		*optname_erpc_wifi = ERPC_WIFI_SO_SNDLOWAT;
		break;
	case SO_RCVTIMEO:
		*optname_erpc_wifi = ERPC_WIFI_SO_RCVTIMEO;
		break;
	case SO_SNDTIMEO:
		*optname_erpc_wifi = ERPC_WIFI_SO_SNDTIMEO;
		break;
	case SO_BINDTODEVICE:
		*optname_erpc_wifi = ERPC_WIFI_SO_BINDTODEVICE;
		break;
	case SO_ACCEPTCONN:
		*optname_erpc_wifi = ERPC_WIFI_SO_ACCEPTCONN;
		break;
	default:
		return -EAFNOSUPPORT;
		break;
	}

	return 0;
}

static int erpc_wifi_socket_family_to_posix(uint8_t family_erpc_wifi, int *family)
{
	switch (family_erpc_wifi) {
	case ERPC_WIFI_AF_INET:
		*family = AF_INET;
		break;
	case ERPC_WIFI_AF_INET6:
		*family = AF_INET6;
		break;
	default:
		return -EAFNOSUPPORT;
		break;
	}

	return 0;
}

static int erpc_wifi_socket_family_from_posix(int family, uint8_t *family_erpc_wifi)
{
	switch (family) {
	case AF_INET:
		*family_erpc_wifi = ERPC_WIFI_AF_INET;
		break;
	case AF_INET6:
		*family_erpc_wifi = ERPC_WIFI_AF_INET6;
		break;
	default:
		return -EAFNOSUPPORT;
		break;
	}

	return 0;
}

static int erpc_wifi_socket_addr_from_posix(const struct sockaddr *addr,
					    struct ra_erpc_sockaddr *addr_erpc_wifi)
{
	int err;

	memset(addr_erpc_wifi, 0, sizeof(struct ra_erpc_sockaddr));

	err = erpc_wifi_socket_family_from_posix(addr->sa_family, &addr_erpc_wifi->sa_family);
	if (err) {
		LOG_ERR("%s - unsupported family: %d", __FUNCTION__, addr->sa_family);
		return err;
	}

	if (addr->sa_family == AF_INET) {
		const struct sockaddr_in *sin = net_sin(addr);
		addr_erpc_wifi->sa_len = 16;
		memcpy(addr_erpc_wifi->sa_data, &sin->sin_port, 2);
		memcpy(&addr_erpc_wifi->sa_data[2], &sin->sin_addr, 4);
	}
#if defined(CONFIG_NET_IPV6)
	else if (addr->sa_family == AF_INET6) {
		const struct sockaddr_in6 *sin6 = net_sin6(addr);
		addr_erpc_wifi->sa_len = 28;
		/* Map Zephyr sockaddr_in6 to RA6W1/LwIP layout (28 bytes in sa_data) */
		/* 0-1: port */
		memcpy(addr_erpc_wifi->sa_data, &sin6->sin6_port, 2);
		/* 2-5: flowinfo (not in Zephyr) */
		memset(&addr_erpc_wifi->sa_data[2], 0, 4);
		/* 6-21: addr */
		memcpy(&addr_erpc_wifi->sa_data[6], &sin6->sin6_addr, 16);
		/* 22-25: scope_id (1 byte in Zephyr, 4 in RA6W1) */
		memset(&addr_erpc_wifi->sa_data[22], 0, 4);
		addr_erpc_wifi->sa_data[22] = sin6->sin6_scope_id;
	}
#endif
	else {
		return -EAFNOSUPPORT;
	}

	return 0;
}

static int erpc_wifi_socket_addr_to_posix(struct sockaddr *addr,
					  struct ra_erpc_sockaddr *addr_erpc_wifi)
{
	int err;

	err = erpc_wifi_socket_family_to_posix(addr_erpc_wifi->sa_family, (int *)&addr->sa_family);
	if (err) {
		LOG_ERR("%s - unsupported family: %d", __FUNCTION__, addr_erpc_wifi->sa_family);
		return err;
	}

	if (addr->sa_family == AF_INET) {
		struct sockaddr_in *sin = net_sin(addr);
		memcpy(&sin->sin_port, addr_erpc_wifi->sa_data, 2);
		memcpy(&sin->sin_addr, &addr_erpc_wifi->sa_data[2], 4);
	}
#if defined(CONFIG_NET_IPV6)
	else if (addr->sa_family == AF_INET6) {
		struct sockaddr_in6 *sin6 = net_sin6(addr);
		/* Map RA6W1/LwIP layout back to Zephyr */
		memcpy(&sin6->sin6_port, addr_erpc_wifi->sa_data, 2);
		/* flowinfo at sa_data[2] is ignored/not in Zephyr */
		memcpy(&sin6->sin6_addr, &addr_erpc_wifi->sa_data[6], 16);
		sin6->sin6_scope_id = addr_erpc_wifi->sa_data[22];
	}
#endif
	else {
		return -EAFNOSUPPORT;
	}

	return 0;
}

static struct erpc_wifi_socket *erpc_wifi_socket_allocate(int fd, int zfd)
{
	struct erpc_wifi_socket *socket = NULL;

	erpc_wifi_lock();
	
	/* If RA6W1 reallocated an FD that Zephyr still thinks is in use (e.g., the RA6W1 implicitly
	 * closed a DHCP UDP socket and reused the FD for a TLS TCP socket), we must force-close
	 * the old Zephyr socket so background threads like DHCP don't steal the new socket's data!
	 */
	for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
		if (sockets[i].in_use && sockets[i].fd == fd) {
			LOG_WRN("RA6W1 FD %d reused! Force-closing stale Zephyr socket (zfd=%d)", 
				fd, sockets[i].zfd);
			sockets[i].in_use = false;
			sockets[i].fd = -1;
			sockets[i].triggered_events |= SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE;
			if (sockets[i].waiting) {
				k_poll_signal_raise(&sockets[i].poll_signal, 0);
			}
		}
	}

	for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
		if (sockets[i].in_use == false) {
			sockets[i].fd = fd;
			sockets[i].zfd = zfd;
			sockets[i].in_use = true;
			sockets[i].waiting = false;
			k_poll_signal_init(&sockets[i].poll_signal);
			sockets[i].recv_timeout_ms = 0;
			sockets[i].triggered_events = 0;
			socket = &sockets[i];
			break;
		}
	}
	erpc_wifi_unlock();

	if (socket != NULL) {
		/* Wake the poll task so it begins monitoring the newly allocated socket */
		k_sem_give(&poll_task_sem);
	}

	return socket;
}

static void erpc_wifi_socket_free(int fd)
{
	erpc_wifi_lock();
	for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
		if (sockets[i].in_use && sockets[i].fd == fd) {
			sockets[i].in_use = false;
			break;
		}
	}
	erpc_wifi_unlock();
}

static int erpc_wifi_socket_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	if (sock->tcp_dpm_filter_set && sock->bound_port != 0) {
		erpc_wifi_lock();
		(void)ra6w1_wifi_dpm_tcp_port_delete(sock->bound_port);
		erpc_wifi_unlock();
		sock->tcp_dpm_filter_set = false;
		LOG_INF("TCP DPM wake filter removed for port %u", sock->bound_port);
	}

	sock->bound_port = 0;
	if (addr && addr->sa_family == AF_INET) {
		const struct sockaddr_in *sin = (const struct sockaddr_in *)addr;
		sock->bound_port = ntohs(sin->sin_port);
		LOG_DBG("bind: local port=%u", sock->bound_port);
	}

	LOG_DBG("fd: %d", sock->fd);

	ret = erpc_wifi_socket_addr_from_posix(addr, &addr_erpc_wifi);
	if (ret) {
		return ret;
	}

	erpc_wifi_lock();
	ret = ra6w1_bind(sock->fd, &addr_erpc_wifi, addr_erpc_wifi.sa_len);
	erpc_wifi_unlock();

	LOG_DBG("ra6w1_bind: %d", ret);

	return ret;
}

static int erpc_wifi_socket_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	erpc_wifi_ps_notify_socket_connect_start();

	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		erpc_wifi_ps_notify_socket_connect_failed();
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		return __w;
	}
	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("sa_family: %d", addr->sa_family);
	LOG_DBG("addrlen: %d", addrlen);
	LOG_DBG("fd: %d", sock->fd);
	// addr->sa_family = AF_INET6;
	if (addr->sa_family == AF_INET) {
		char addr_str[INET_ADDRSTRLEN];
		struct sockaddr_in *s_addr = net_sin(addr);

		net_addr_ntop(addr->sa_family, &s_addr->sin_addr, addr_str, sizeof(addr_str));
		LOG_DBG("sin: addr: %s port: %d", addr_str, ntohs(s_addr->sin_port));
		if (sock->type == SOCK_STREAM) {
			strncpy(sock->addr_str, addr_str, sizeof(sock->addr_str) - 1);
			sock->addr_str[sizeof(sock->addr_str) - 1] = '\0';
			sock->port = ntohs(s_addr->sin_port);
		}
	}
#if defined(CONFIG_NET_IPV6)
	else if (addr->sa_family == AF_INET6) {
		char addr_str[INET6_ADDRSTRLEN];
		struct sockaddr_in6 *s_addr = net_sin6(addr);

		net_addr_ntop(addr->sa_family, &s_addr->sin6_addr, addr_str, sizeof(addr_str));
		LOG_DBG("sin6: addr: %s port: %d", addr_str, ntohs(s_addr->sin6_port));
	}
#endif

	if (sock->type == SOCK_STREAM && sock->bound_port == 0) {
		struct sockaddr_in local_addr = {
			.sin_family = AF_INET,
			.sin_port = htons(55000 + sock->fd),
			.sin_addr = { .s_addr = INADDR_ANY }
		};
		int bind_ret = erpc_wifi_socket_bind(sock, (const struct sockaddr *)&local_addr, sizeof(local_addr));
		if (bind_ret < 0) {
			LOG_WRN("Auto-bind to port %u failed: %d", 55000 + sock->fd, bind_ret);
		} else {
			LOG_INF("Auto-bound TCP client socket to port %u for DPM tracking", 55000 + sock->fd);
		}
	}

	ret = erpc_wifi_socket_addr_from_posix(addr, &addr_erpc_wifi);
	if (ret) {
		erpc_wifi_ps_notify_socket_connect_failed();
		return ret;
	}
	erpc_wifi_lock();
	ret = ra6w1_connect(sock->fd, &addr_erpc_wifi, addr_erpc_wifi.sa_len);
	erpc_wifi_unlock();
	k_msleep(100);

	/* Register TCP DPM wake filter BEFORE releasing the RAM/awake constraint.
	 * If done after pmgr_ram_release + ps_notify_wakeup the module may have
	 * started its sleep transition and the eRPC call gets a CRC error. */
	if (ret == 0 && sock->type == SOCK_STREAM && sock->bound_port != 0 &&
	    !sock->tcp_dpm_filter_set) {
		erpc_wifi_lock();
		int32_t rc = ra6w1_wifi_dpm_tcp_port_filter_set(sock->bound_port);
		erpc_wifi_unlock();
		if (rc == 0) {
			sock->tcp_dpm_filter_set = true;
			LOG_INF("TCP DPM wake filter set for connected port %u",
				sock->bound_port);
		} else {
			LOG_WRN("Failed to set TCP DPM wake filter for connected port %u (rc=%d)",
				sock->bound_port, rc);
		}
	}

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_connect: %d", ret);

	//erpc_wifi_ps_notify_socket_connected();
	if (ret == 0) {
		erpc_wifi_ps_notify_socket_connected();
	} else {
		erpc_wifi_ps_notify_socket_connect_failed();
	}
	return ret;
}

static int erpc_wifi_socket_listen(void *obj, int backlog)
{
	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("fd: %d", sock->fd);
	LOG_DBG("backlog: %d", backlog);

	erpc_wifi_lock();
	ret = ra6w1_listen(sock->fd, backlog);
	erpc_wifi_unlock();

	LOG_DBG("ra6w1_listen: %d", ret);

	if (ret == 0 && sock->type == SOCK_STREAM && sock->bound_port != 0 &&
	    !sock->tcp_dpm_filter_set) {
		erpc_wifi_lock();
		int32_t rc = ra6w1_wifi_dpm_tcp_port_filter_set(sock->bound_port);
		erpc_wifi_unlock();
		if (rc == 0) {
			sock->tcp_dpm_filter_set = true;
			LOG_INF("TCP DPM wake filter set for port %u", sock->bound_port);
		} else {
			LOG_WRN("Failed to set TCP DPM wake filter for port %u (rc=%d)",
				sock->bound_port, rc);
		}
	}

	return ret;
}

static int erpc_wifi_socket_accept(void *obj, struct sockaddr *addr, socklen_t *addrlen)
{
	int fd;
	int conn_fd;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;
	struct erpc_wifi_socket *socket;

	LOG_DBG("fd: %d", sock->fd);

	fd = zvfs_reserve_fd();

	/* Accept returns file descriptor for new connected socket */
	erpc_wifi_lock();
	conn_fd = ra6w1_accept(sock->fd, &addr_erpc_wifi, addrlen);
	erpc_wifi_unlock();
	LOG_DBG("ra6w1_accept: %d", conn_fd);

	if (conn_fd < 0) {
		zvfs_free_fd(fd);
		return conn_fd;
	}

	int ret = erpc_wifi_socket_addr_to_posix(addr, &addr_erpc_wifi);
	if (ret < 0) {
		zvfs_free_fd(fd);
		LOG_DBG("erpc_wifi_socket_addr_to_posix error: %d", ret);
		return ret;
	}

	/* Keep track of the new fd returned by accept */
	socket = erpc_wifi_socket_allocate(conn_fd, fd);

	if (socket == NULL) {
		zvfs_free_fd(fd);
		return -1;
	}

	/* Associate zephyr file descriptor with the socket obj and offload table
	   ensures subsequent socket operations using zephyr fd will use the
	       appropriate RA6W1 fd */
	zvfs_finalize_typed_fd(fd, socket,
			       (const struct fd_op_vtable *)&erpc_wifi_socket_fd_op_vtable,
			       ZVFS_MODE_IFSOCK);

	return fd;
}

static ssize_t erpc_wifi_socket_sendto(void *obj, const void *buf, size_t len, int flags,
				       const struct sockaddr *dest_addr, socklen_t addrlen)
{
	LOG_INF("TX wake request (len=%u)", (unsigned int)len);

    int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		return __w; 

	}

	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("fd: %d", sock->fd);
	LOG_DBG("len: %d", len);
	LOG_DBG("dest_addr: %x", (uint32_t)dest_addr);
	LOG_DBG("addrlen: %d", addrlen);

	int ra_flags = 0;
	if (flags & ZSOCK_MSG_DONTWAIT) {
		ra_flags |= 0x08;
	}

	if (dest_addr) {
		ret = erpc_wifi_socket_addr_from_posix(dest_addr, &addr_erpc_wifi);
		if (ret) {
			return ret;
		}
		erpc_wifi_lock();
		ret = ra6w1_sendto(sock->fd, buf, len, ra_flags, &addr_erpc_wifi,
				   addr_erpc_wifi.sa_len);
		erpc_wifi_unlock();
		LOG_DBG("ra6w1_sendto: %d", ret);

		LOG_DBG("ra6w1_sendto: %d", ret);

		if (dest_addr) {
			LOG_DBG("family: %d", dest_addr->sa_family);
			LOG_DBG("address: %d.%d.%d.%d", dest_addr->data[2], dest_addr->data[3],
				dest_addr->data[4], dest_addr->data[5]);
			LOG_DBG("port: %d", ((struct sockaddr_in *)dest_addr)->sin_port);
			LOG_DBG("addrlen: %d", addrlen);
		}
	} else {
		erpc_wifi_lock();
		ret = ra6w1_send(sock->fd, buf, len, ra_flags);
		erpc_wifi_unlock();

		LOG_DBG("ra6w1_send: %d", ret);
	}

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	/* Re-arm PS only after this TX operation completed. */
	erpc_wifi_ps_notify_wakeup();

	return ret;
}

ssize_t erpc_wifi_socket_sendmsg(void *obj, const struct msghdr *msg, int flags)
{

	if (msg->msg_iov) {
		ssize_t len = 0;

		for (int i = 0; i < msg->msg_iovlen; i++) {
			int ret = erpc_wifi_socket_sendto(obj, msg->msg_iov[i].iov_base,
							  msg->msg_iov[i].iov_len, flags, NULL, 0);

			if (ret < 0) {
				return ret;
			}

			len += ret;
		}

		return len;
	}

	return -ENODATA;
}

static ssize_t erpc_wifi_socket_recvfrom(void *obj, void *buf, size_t max_len, int flags,
					 struct sockaddr *src_addr, socklen_t *addrlen)
{
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	if ((sock->flags & O_NONBLOCK) &&
	    !(sock->triggered_events & (SOCKET_EVENT_RX | SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE))) {
		errno = EAGAIN;
		return -1;
	}

	int64_t start_time = k_uptime_get();
	uint32_t timeout_ms = sock->recv_timeout_ms;
	if (timeout_ms == 0) {
		timeout_ms = UINT32_MAX; // infinite
	}

	int ret;

	for (;;) {
		int __w = erpc_wifi_ensure_awake_rx(ERPC_PMGR_JOB_ID_RECV);
		if (__w != 0) {
			LOG_INF("erpc_wifi_ensure_awake_rx failed: IN %s = %d", __func__, __w);
			errno = (int)-__w;
			return -1;
		}

		erpc_wifi_ps_hold_during_recv();

		LOG_DBG("fd: %d", sock->fd);
		LOG_DBG("max_len: %d", max_len);
		LOG_DBG("src_addr: %x", (uint32_t)src_addr);
		LOG_DBG("addrlen: %x", (uint32_t)addrlen);

		int ra_flags = 0x08;
		if (flags & ZSOCK_MSG_PEEK) {
			ra_flags |= 0x02;
		}

		if (src_addr && sock->type != SOCK_STREAM) {
			struct ra_erpc_sockaddr addr_erpc_wifi;
			uint32_t addrlen_erpc = sizeof(struct ra_erpc_sockaddr);

			erpc_wifi_lock();
			ret = ra6w1_recvfrom(sock->fd, buf, max_len, ra_flags, &addr_erpc_wifi,
					     &addrlen_erpc);
			erpc_wifi_unlock();
			LOG_DBG("ra6w1_recvfrom non-blocking: %d", ret);

			if (ret >= 0) {
				int translate_rc = erpc_wifi_socket_addr_to_posix(src_addr, &addr_erpc_wifi);
				if (translate_rc < 0) {
					LOG_ERR("erpc_wifi_socket_addr_to_posix failed: %d", translate_rc);
				} else {
					if (addrlen) {
						*addrlen = (src_addr->sa_family == AF_INET6) ? 
								sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
					}
				}

				LOG_DBG("family: %d", src_addr->sa_family);
				LOG_DBG("address: %d.%d.%d.%d", src_addr->data[2], src_addr->data[3],
					src_addr->data[4], src_addr->data[5]);
				LOG_DBG("port: %d", ((struct sockaddr_in *)src_addr)->sin_port);

				if (addrlen) {
					LOG_DBG("addrlen: %d", *addrlen);
				}
			}
		} else {
			erpc_wifi_lock();
			(void)ra6w1_pmgr_dpm_rcv_ready_set(ERPC_PMGR_JOB_ID_RECV);
			ret = ra6w1_recv(sock->fd, buf, max_len, ra_flags);
			erpc_wifi_unlock();
			LOG_DBG("ra6w1_recv non-blocking: %d", ret);

			if (ret >= 0 && src_addr && sock->type == SOCK_STREAM) {
				struct sockaddr_in *sin = net_sin(src_addr);

				(void)memset(sin, 0, sizeof(*sin));
				sin->sin_family = AF_INET;
				sin->sin_port = htons((uint16_t)sock->port);
				if (sock->addr_str[0] != '\0') {
					(void)net_addr_pton(AF_INET, sock->addr_str, &sin->sin_addr);
				}
				if (addrlen) {
					*addrlen = sizeof(struct sockaddr_in);
				}
			}
		}

		erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
		erpc_wifi_ps_notify_wakeup();

		/*
		 * RA6W1 non-blocking recv may return -1 for "no data yet" instead of
		 * -EAGAIN/-EWOULDBLOCK. Treat it as transient so higher layers can
		 * continue polling during TLS handshake.
		 */
		if (ret >= 0 || (ret != -EAGAIN && ret != -EWOULDBLOCK && ret != -6 && ret != -1)) {
			if (ret < 0) {
				LOG_ERR("ra6w1_recv failed: %d", ret);
				errno = (ret == -1) ? EIO : (int)-ret;
				return -1;
			}
			return ret;
		}
		sock->triggered_events &= ~SOCKET_EVENT_RX;

		/* If MSG_DONTWAIT was specified in this call, or O_NONBLOCK is configured on the socket,
		 * return -EAGAIN immediately.
		 */
		if ((flags & ZSOCK_MSG_DONTWAIT) || (sock->flags & O_NONBLOCK)) {
			errno = EAGAIN;
			return -1;
		}

wait_poll:
		/* Check if our receive timeout has expired. */
		int64_t elapsed = k_uptime_get() - start_time;
		if (timeout_ms != UINT32_MAX && elapsed >= (int64_t)timeout_ms) {
			errno = EAGAIN;
			return -1;
		}

		/* Calculate remaining timeout for the poll block. */
		k_timeout_t rem_timeout = (timeout_ms == UINT32_MAX) ? K_FOREVER : K_MSEC(timeout_ms - elapsed);

		sock->triggered_events = 0;
		sock->waiting = true;
		sock->poll_events = ZVFS_POLLIN;
		k_poll_signal_reset(&sock->poll_signal);
		k_sem_give(&poll_task_sem);

		struct k_poll_event event;
		k_poll_event_init(&event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sock->poll_signal);

		LOG_DBG("recvfrom: blocking on poll signal, remaining tmo: %lld ms", 
			(timeout_ms == UINT32_MAX) ? -1LL : (int64_t)(timeout_ms - elapsed));
		
		int poll_rc = k_poll(&event, 1, rem_timeout);
		sock->waiting = false;

		if (poll_rc != 0) {
			/* k_poll returned a timeout or error. */
			errno = EAGAIN;
			return -1;
		}

		/* Poll task signaled that data has arrived (SOCKET_EVENT_RX). Retry the non-blocking read. */
		LOG_DBG("recvfrom: poll signal raised, retrying read");
	}
}

static int erpc_wifi_socket_getsockopt(void *obj, int level, int optname, void *optval,
				       socklen_t *optlen)
{
    int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		return __w;

	}

	int ret;
	int level_erpc_wifi;
	int optname_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("level: %d", level);
	LOG_DBG("optname: %d", optname);

	ret = erpc_wifi_socket_level_from_posix(level, &level_erpc_wifi);
	if (ret) {
		return ret;
	}

	//ret = erpc_wifi_socket_option_from_posix(optname, &optname_erpc_wifi);
	ret = erpc_wifi_socket_option_from_posix(level, optname, &optname_erpc_wifi);
	if (ret) {
		return ret;
	}

	erpc_wifi_lock();
	ret = ra6w1_getsockopt(sock->fd, level_erpc_wifi, optname_erpc_wifi, optval, optlen);
	erpc_wifi_unlock();

	erpc_wifi_pmgr_ram_release(0);
	erpc_wifi_ps_notify_wakeup();

	LOG_DBG("ra6w1_getsockopt: %d", ret);

	return ret;
}

static int erpc_wifi_socket_setsockopt(void *obj, int level, int optname, const void *optval,
				       socklen_t optlen)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		return __w;
	}

	int ret;
	int level_erpc_wifi;
	int optname_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("level: %d", level);
	LOG_DBG("optname: %d", optname);
	printk("TLS setsockopt: fd=%d level=%d opt=%d len=%d\n", sock ? sock->fd : -1, level,
	       optname, optlen);

	if (level == 1 && optname == 20 && optval != NULL) {
		const struct timeval *tv = (const struct timeval *)optval;
		sock->recv_timeout_ms = (uint32_t)(tv->tv_sec * 1000 + tv->tv_usec / 1000);
	}

	ret = erpc_wifi_socket_level_from_posix(level, &level_erpc_wifi);
	if (ret) {
		return ret;
	}

	//ret = erpc_wifi_socket_option_from_posix(optname, &optname_erpc_wifi);
	ret = erpc_wifi_socket_option_from_posix(level, optname, &optname_erpc_wifi);
	if (ret) {
		return ret;
	}

	erpc_wifi_lock();
	ret = ra6w1_setsockopt(sock->fd, level_erpc_wifi, optname_erpc_wifi, (uint32_t *)optval,
			       optlen);
	erpc_wifi_unlock();

	erpc_wifi_pmgr_ram_release(0);
	erpc_wifi_ps_notify_wakeup();

	LOG_DBG("ra6w1_setsockopt: %d", ret);

	return ret;
}

static ssize_t erpc_wifi_socket_read(void *obj, void *buf, size_t sz)
{
	return erpc_wifi_socket_recvfrom(obj, buf, sz, 0, NULL, NULL);
}

static ssize_t erpc_wifi_socket_write(void *obj, const void *buf, size_t sz)
{
	return erpc_wifi_socket_sendto(obj, buf, sz, 0, NULL, 0);
}

static int erpc_wifi_socket_close(void *obj)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) {
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__, __w);
		return __w;
	}

	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("erpc_wifi_socket_close");

	if (sock->tcp_dpm_filter_set && sock->bound_port != 0) {
		erpc_wifi_lock();
		(void)ra6w1_wifi_dpm_tcp_port_delete(sock->bound_port);
		erpc_wifi_unlock();
		sock->tcp_dpm_filter_set = false;
		LOG_INF("TCP DPM wake filter removed for port %u", sock->bound_port);
	}

	erpc_wifi_socket_free(sock->fd);
	erpc_wifi_lock();
	ret = ra6w1_close(sock->fd);
	erpc_wifi_unlock();
	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_close: %d", ret);
	if (sock->lock != NULL) {
		k_mutex_unlock(sock->lock);
	}
	return ret;
}

static struct erpc_wifi_socket *find_socket_by_fd(int zfd)
{
	for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
		if (sockets[i].in_use && sockets[i].zfd == zfd) {
			return &sockets[i];
		}
	}

	return NULL;
}

static int erpc_wifi_socket_poll_offload(struct zvfs_pollfd *fds, int nfds, int timeout)
{
	struct k_poll_event events[ERPC_WIFI_MAX_SOCKETS];
	struct erpc_wifi_socket *tracked_socks[ERPC_WIFI_MAX_SOCKETS];
	int tracked_count = 0;
	int ready_count = 0;

	LOG_INF("poll offload start nfds=%d timeout=%d", nfds, timeout);

	ensure_poll_task_started();

	for (int i = 0; i < nfds; i++) {
		fds[i].revents = 0;
		struct erpc_wifi_socket *sock = find_socket_by_fd(fds[i].fd);

		if (!sock) {
			/* Mixed fd tables are valid in Zephyr (e.g. socket_service eventfd).
			 * Leave non-driver fds untouched so their owner can handle them.
			 */
			continue;
		}

		if (tracked_count < ERPC_WIFI_MAX_SOCKETS) {
			k_poll_signal_reset(&sock->poll_signal);
			sock->poll_events = fds[i].events;
			sock->triggered_events = 0;
			sock->waiting = true;

			k_poll_event_init(&events[tracked_count], K_POLL_TYPE_SIGNAL,
					  K_POLL_MODE_NOTIFY_ONLY, &sock->poll_signal);
			tracked_socks[tracked_count] = sock;
			tracked_count++;
		}
	}

	if (tracked_count > 0) {
		k_sem_give(&poll_task_sem);
	}

	if (ready_count > 0 || timeout == 0) {
		goto update_revents;
	}

	if (tracked_count > 0) {
		int ret = k_poll(events, tracked_count, K_MSEC(timeout));

		if (ret != 0 && ret != -EAGAIN) {
			LOG_ERR("k_poll failed: %d", ret);
		}
	}

update_revents:
	for (int i = 0; i < nfds; i++) {
		struct erpc_wifi_socket *sock = find_socket_by_fd(fds[i].fd);

		if (sock) {
			sock->waiting = false;
			uint32_t ev = (uint32_t)sock->triggered_events;

			if ((ev & SOCKET_EVENT_RX) && (fds[i].events & ZVFS_POLLIN)) {
				fds[i].revents |= ZVFS_POLLIN;
			}
			if ((ev & SOCKET_EVENT_TX) && (fds[i].events & ZVFS_POLLOUT)) {
				fds[i].revents |= ZVFS_POLLOUT;
			}
			if ((ev & SOCKET_EVENT_ERR) && (fds[i].events & ZVFS_POLLERR)) {
				fds[i].revents |= ZVFS_POLLERR;
			}
			if ((ev & SOCKET_EVENT_CLOSE) && (fds[i].events & ZVFS_POLLHUP)) {
				fds[i].revents |= ZVFS_POLLHUP;
			}

			if (fds[i].revents != 0) {
				ready_count++;
			}
		}
	}

	LOG_INF("poll offload done ready_count=%d", ready_count);
	return ready_count;
}

static int erpc_wifi_socket_poll_update(struct zvfs_pollfd *pfd, struct k_poll_event **pev)
{
	struct erpc_wifi_socket *sock = find_socket_by_fd(pfd->fd);

	if (!sock) {
		/* Not owned by this driver: ignore and let owner handle update path. */
		return 0;
	}

	sock->waiting = false;

	if (*pev != NULL && (*pev)->type == K_POLL_TYPE_SIGNAL) {
		if ((*pev)->state == K_POLL_STATE_SIGNALED) {
			uint32_t ev = (uint32_t)sock->triggered_events;

			if ((ev & SOCKET_EVENT_RX) && (pfd->events & ZVFS_POLLIN)) {
				pfd->revents |= ZVFS_POLLIN;
			}
			if ((ev & SOCKET_EVENT_TX) && (pfd->events & ZVFS_POLLOUT)) {
				pfd->revents |= ZVFS_POLLOUT;
			}
			if ((ev & SOCKET_EVENT_ERR) && (pfd->events & ZVFS_POLLERR)) {
				pfd->revents |= ZVFS_POLLERR;
			}
			if ((ev & SOCKET_EVENT_CLOSE) && (pfd->events & ZVFS_POLLHUP)) {
				pfd->revents |= ZVFS_POLLHUP;
			}
		}
		(*pev)++;
	}

	return 0;
}

static int erpc_wifi_socket_ioctl(void *obj, unsigned int request, va_list args)
{
	struct erpc_wifi_socket *sock = obj;
	int ret;

	switch (request) {
	case F_SETFL:
		sock->flags = va_arg(args, int);
		__fallthrough;
	case F_GETFL:
		LOG_INF("Socket flags request: %d: 0x%x", request, sock->flags);
		return sock->flags;

	case ZFD_IOCTL_POLL_PREPARE: {
		struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
		struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);

		if (*pev >= pev_end) {
			return -ENOSPC;
		}

		ensure_poll_task_started();

		k_poll_signal_reset(&sock->poll_signal);
		sock->poll_events = pfd->events;
		sock->triggered_events = 0;
		sock->waiting = true;

		k_poll_event_init(*pev, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
				  &sock->poll_signal);
		(*pev)++;

		k_sem_give(&poll_task_sem);

		return 0;
	}

	case ZFD_IOCTL_POLL_OFFLOAD: {
		struct zvfs_pollfd *fds = va_arg(args, struct zvfs_pollfd *);
		int nfds = va_arg(args, int);
		int timeout = va_arg(args, int);

		return erpc_wifi_socket_poll_offload(fds, nfds, timeout);
	}

	case ZFD_IOCTL_POLL_UPDATE: {
		struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);

		// For mixed polling (optional)
		return erpc_wifi_socket_poll_update(pfd, pev);
	}
#if 1
	case ZFD_IOCTL_SET_LOCK:
		struct k_mutex *lock = va_arg(args, struct k_mutex *);
		sock->lock = lock;
		printk("SET_LOCK stored lock=%p zfd=%d remote=%d\n", (void *)lock, sock->zfd,
		       sock->fd);
		return 0;
#endif
	default:
		errno = EINVAL;
		return -1;
	}
}

static const struct socket_op_vtable erpc_wifi_socket_fd_op_vtable = {
	.fd_vtable =
		{
			.read = erpc_wifi_socket_read,
			.write = erpc_wifi_socket_write,
			.close = erpc_wifi_socket_close,
			.ioctl = erpc_wifi_socket_ioctl,
		},

	.bind = erpc_wifi_socket_bind,
	.connect = erpc_wifi_socket_connect,
	.listen = erpc_wifi_socket_listen,
	.accept = erpc_wifi_socket_accept,
	.sendto = erpc_wifi_socket_sendto,
	.recvfrom = erpc_wifi_socket_recvfrom,
	.getsockopt = erpc_wifi_socket_getsockopt,
	.setsockopt = erpc_wifi_socket_setsockopt,
	.sendmsg = erpc_wifi_socket_sendmsg,

};

static int erpc_wifi_socket_create(int family, int type, int proto)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		return __w;
	}

	int err;
	int sock;
	uint8_t family_erpc_wifi;
	int fd = zvfs_reserve_fd();
	struct erpc_wifi_socket *socket = NULL;

	LOG_DBG("erpc_wifi_socket_create");
	LOG_DBG("family: %d", family);
	LOG_DBG("type: %d", type);
	LOG_DBG("proto: %d", proto);

	/* Map Zephyr socket.h family to RA RPC's */
	err = erpc_wifi_socket_family_from_posix(family, &family_erpc_wifi);
	if (err) {
		LOG_ERR("unsupported family: %d", family);
		return err;
	}
	erpc_wifi_lock();
	sock = ra6w1_socket(family_erpc_wifi, type, proto);
	erpc_wifi_unlock();
	erpc_wifi_pmgr_ram_release(0);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_socket: %d", sock);

	if (sock < 0) {
		zvfs_free_fd(fd);
		return -1;
	}

	socket = erpc_wifi_socket_allocate(sock, fd);

	if (socket == NULL) {
		zvfs_free_fd(fd);
		return -1;
	}
	
	socket->type = type;
	zvfs_finalize_typed_fd(fd, socket,
			       (const struct fd_op_vtable *)&erpc_wifi_socket_fd_op_vtable,
			       ZVFS_MODE_IFSOCK);

	return fd;
}

static bool erpc_wifi_socket_is_supported(int family, int type, int proto)
{
	if (family != AF_INET && family != AF_INET6) {
		return false;
	}

	if (type != SOCK_DGRAM && type != SOCK_STREAM && type != SOCK_RAW) {
		return false;
	}

	if (proto != IPPROTO_TCP && proto != IPPROTO_UDP && proto != IPPROTO_ICMPV6) {
		return false;
	}

	return true;
}

int erpc_wifi_socket_offload_init(struct net_if *iface)
{
	memset(sockets, 0, sizeof(sockets));
	k_sem_init(&poll_task_sem, 0, 1);

	net_if_socket_offload_set(iface, erpc_wifi_socket_create);

#ifndef CONFIG_DNS_RESOLVER
	erpc_wifi_dns_offload_init();
#endif

	return 0;
}

#ifdef CONFIG_NET_SOCKETS_OFFLOAD
NET_SOCKET_OFFLOAD_REGISTER(erpc_wifi, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, AF_UNSPEC,
			    erpc_wifi_socket_is_supported, erpc_wifi_socket_create);
#endif