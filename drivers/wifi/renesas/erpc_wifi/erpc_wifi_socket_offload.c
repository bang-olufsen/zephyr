/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include "erpc_wifi.h"
#include "erpc_wifi_transport.h"
#include "erpc_wifi_cmd.h"
#include "erpc_wifi_cmd_process_handlers.h"
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
extern int32_t ra6w1_pmgr_dpm_job_name_set(uint32_t job_id, const char *job_name);
extern int32_t ra6w1_pmgr_add_sleep_constraint(uint32_t constraint);
extern int32_t ra6w1_pmgr_remove_sleep_constraint(uint32_t constraint);
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
	erpc_wifi_ps_wait_awake_rx();

	if (!erpc_wifi_ps_is_enabled()) {
		return 0;
	}

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
			return -EAGAIN;
		}
		k_msleep(10);

		/* Re-verify SRDY after wait: RA6W1 may have
		 * gone back to DPM sleep during the wait. */
		if (!erpc_wifi_transport_slave_ready()) {
			k_msleep(50);
			return -EAGAIN;
		}

		if (pmgr_ram_hold() < 0) {
			k_msleep(50);
			return -EAGAIN;
		}

		{
			erpc_wifi_pmgr_job_t job_msg = { .job_id = job_id };
			int32_t rc = (int32_t)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_DPM_RCV_READY_SET_CMD,
												 &job_msg, sizeof(job_msg), -1);
			if (rc < 0) {
				LOG_WRN("RX wake prep failed: rcv_ready_set rc=%d", rc);
				erpc_wifi_pmgr_ram_release(0);
				k_msleep(50);
				return -EAGAIN;
			}
		}
		
		return 0;
	}

	return -EAGAIN;
}

static int erpc_wifi_ensure_awake_tx(uint32_t job_id)
{
	erpc_wifi_ps_wait_awake_tx();

	erpc_wifi_ps_cancel_sleep_work();

	if (atomic_get(&g_erpc_tx_blocked) == 0) {
		if (!erpc_wifi_ps_is_enabled() || erpc_wifi_transport_slave_ready() == 1) {
			if (erpc_wifi_ps_is_enabled() && erpc_wifi_transport_slave_ready() == 1) {
				erpc_wifi_ps_hold_awake("tx-awake");
			}
			LOG_DBG("TX wake skipped (tx_blocked=0, job=%u)", job_id);
			erpc_wifi_ps_reset_state_awake();
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
			k_msleep(10);

			/* Re-verify SRDY after wait: RA6W1 may have
			 * gone back to DPM sleep during the wait. */
			if (!erpc_wifi_transport_slave_ready()) {
				k_msleep(50);
				continue;
			}

			if (pmgr_ram_hold() < 0) {
				k_msleep(50);
				continue;
			}
			
			erpc_wifi_ps_hold_awake("tx-awake");

			/* Tell PMGR this module has pending I/O work only after wake is visible. */
			{
				erpc_wifi_pmgr_job_t job_msg = { .job_id = job_id };
				int32_t rc = (int32_t)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_DPM_RCV_READY_SET_CMD,
													 &job_msg, sizeof(job_msg), -1);
				if (rc < 0) {
					LOG_WRN("TX wake prep failed: rcv_ready_set rc=%d", rc);
					k_msleep(50);
					continue;
				}
			}
			// /* Re-arm sleep timer so module returns to DPM after this I/O. */
			// erpc_wifi_ps_notify_wakeup();
			erpc_wifi_ps_reset_state_awake();
			return 0;
		}

		/* Some boards require repeated wake pulses while RA6W1 is in DPM. */
		if ((k_uptime_get() - last_pulse) > 300) {
			erpc_wifi_gpio_trigger_wakeup();
			last_pulse = k_uptime_get();
		}

		if (tmo > 0U && (k_uptime_get() - start) > (int64_t)tmo) {
			LOG_WRN("TX wake timeout job=%u tmo=%u", job_id, tmo);
			erpc_wifi_ps_reset_state_awake();
			return -EAGAIN;
		}

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

	int32_t en = (int32_t)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_DPM_IS_ENABLED_CMD, NULL, 0, -1);
	cached = (en == 1) ? 1 : 0;
	return cached;
}

static inline int pmgr_ram_hold(void)
{
	if (pmgr_dpm_cached_enabled()) {
		int32_t rc = -1;
		for (int i = 0; i < 5; i++) {
			erpc_wifi_pmgr_constraint_t cstr = { .constraint = PMGR_CONSTRAINT_POWER_RAM };
			rc = (int32_t)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_ADD_SLEEP_CONSTRAINT_CMD,
											  &cstr, sizeof(cstr), -1);
			if (rc >= 0) {
				break;
			}
			/* Transient SPI CRC error (err 8) is common on the first frame after DPM wake due to
			 * hardware synchronization. Retry. */
			k_msleep(10);
		}
		if (rc < 0) {
			LOG_ERR("pmgr_ram_hold failed: %d", rc);
			return rc;
		}
	}

	return 0;
}

void erpc_wifi_pmgr_ram_release(uint32_t job_id)
{
	if (pmgr_dpm_cached_enabled()) {
		{
			erpc_wifi_pmgr_constraint_t cstr = { .constraint = PMGR_CONSTRAINT_POWER_RAM };
			int32_t rc = (int32_t)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_REMOVE_SLEEP_CONSTRAINT_CMD,
												  &cstr, sizeof(cstr), -1);
			if (rc < 0) {
				LOG_ERR("pmgr_ram_release failed: %d", rc);
			}
		}
		if (job_id != 0) {
			erpc_wifi_pmgr_job_t job_msg = { .job_id = job_id };
			(void)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_DPM_WAKEUP_DONE_CMD,
								   &job_msg, sizeof(job_msg), -1);
		}
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

static struct net_if *net_iface;

static struct erpc_wifi_socket *find_socket_by_fd(int zfd);

static int erpc_wifi_poll_hup_on_iface_down(struct zvfs_pollfd *fds, int nfds)
{
	int ret = 0;

	if (net_if_is_up(net_iface)) {
		return 0;
	}

	for (int i = 0; i < nfds; i++) {
		if (find_socket_by_fd(fds[i].fd)) {
			fds[i].revents = ZSOCK_POLLHUP;
			ret++;
		}
	}

	return ret;
}
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
#include <zephyr/spinlock.h>

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

	char addr_str[NET_IPV6_ADDR_LEN]; /* Supports both IPv4 and IPv6 */
	int port;

	struct k_mutex *lock;    /* for ZFD_IOCTL_SET_LOCK */
	bool connect_pending;
	bool connected;
	int socket_error;
	uint32_t flags;          /* O_NONBLOCK etc */
	uint16_t bound_port;     // Local port from bind() (host order)
	bool tcp_dpm_filter_set; // true if TCP DPM wake filter installed
	uint32_t recv_timeout_ms;
	uint32_t send_timeout_ms;
	bool closing;            // true if close() is in progress
	struct k_spinlock state_lock;

	struct sockaddr addr;
	socklen_t addrlen;
};

static struct erpc_wifi_socket sockets[ERPC_WIFI_MAX_SOCKETS];

K_THREAD_STACK_DEFINE(erpc_wifi_socket_poll_stack, 8192);
static struct k_thread erpc_wifi_socket_poll_thread_data;
static k_tid_t erpc_wifi_socket_poll_tid;
struct k_sem poll_task_sem;
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

		/* Wait until signaled by SRDY/socket events, but keep fallback timeout
		 * to recover from missed/coalesced edges while sockets are active. */
		k_timeout_t wait_timeout = dpm_polling_needed ? K_MSEC(500) : K_FOREVER;
		(void)k_sem_take(&poll_task_sem, wait_timeout);

		bool activity = true;
		/* NOTE: Do NOT block the poll task with wait_awake_rx() here.
		 * The poll task is the primary detector of autonomous DPM wakeups
		 * (UC packet).  It must keep polling SRDY to detect when RA6W1
		 * wakes and call notify_wakeup().  The SRDY re-verification after
		 * 500ms and the ensure_awake_rx guards are sufficient to prevent
		 * premature eRPC calls. */
		while (activity) {
			activity = false;
			bool any_polled = false;

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
						/* SRDY=1 is the strongest wake evidence; sleep_confirmed may lag. */
						if (srdy) {
							if (!erpc_wifi_ps_sleep_is_confirmed()) {
								LOG_INF("Autonomous DPM wakeup via SRDY high (fd=%d), sleep_confirmed pending", sock->fd);
							} else {
								LOG_INF("Autonomous DPM wakeup detected (fd=%d), processing events", sock->fd);
							}
							
							int stable = 0;
							for (int j = 0; j < 3; j++) {
								if (erpc_wifi_transport_slave_ready() == 1) {
									stable++;
								}
								k_msleep(3);
							}

							if (stable < 3) {
								k_msleep(5);
								continue;
							}
							k_msleep(10);

							/* Re-verify SRDY after wait: RA6W1 may have
							 * gone back to DPM sleep during the wait. */
							if (!erpc_wifi_transport_slave_ready()) {
								continue;
							}

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

					any_polled = true;

					int ev_fd = sock->fd;
					uint32_t events = (uint32_t)erpc_wifi_send_cmd(EPRC_WIFI_GET_SOCKET_EVT_CMD,
															  &ev_fd, sizeof(ev_fd), -1);
					if (events != 0 && events != UINT32_MAX) {
						//LOG_WRN("poll_task: fd=%d events=0x%x", sock->fd, events);
					}

					erpc_wifi_pmgr_ram_release(0);

					if (events == UINT32_MAX) {
						LOG_WRN("poll get_socket_events failed fd=%d; deferring", sock->fd);
						continue;
					}

					bool notify_connected = false;
					bool notify_failed = false;
					int log_connected_fd = -1;
					int log_failed_fd = -1;

					k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
					uint32_t all_events = events & (SOCKET_EVENT_RX | SOCKET_EVENT_TX | SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE);
					sock->triggered_events |= (short)all_events;

					if (sock->connect_pending) {
						if (events & SOCKET_EVENT_TX) {
							sock->connect_pending = false;
							sock->connected = true;
							sock->socket_error = 0;
							notify_connected = true;
							log_connected_fd = sock->fd;
						} else if (events & (SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE)) {
							sock->connect_pending = false;
							sock->connected = false;
							sock->socket_error = -1;
							notify_failed = true;
							log_failed_fd = sock->fd;
						}
					}

					uint32_t requested_mask = 0;
					if (sock->poll_events & ZVFS_POLLIN) {
						requested_mask |= SOCKET_EVENT_RX;
					}
					if (sock->poll_events & ZVFS_POLLOUT) {
						requested_mask |= SOCKET_EVENT_TX;
					}
					requested_mask |= SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE;

					uint32_t ready = sock->triggered_events & requested_mask;
					bool do_signal = false;

					if (sock->waiting && ready != 0) {
						sock->waiting = false;
						do_signal = true;
					}
					k_spin_unlock(&sock->state_lock, key);

					if (notify_connected) {
						erpc_wifi_ps_notify_socket_connected();
						LOG_INF("Non-blocking connect completed for fd %d", log_connected_fd);
					}
					if (notify_failed) {
						erpc_wifi_ps_notify_socket_connect_failed();
						LOG_WRN("Non-blocking connect failed for fd %d", log_failed_fd);
					}

					if (do_signal) {
						k_poll_signal_raise(&sock->poll_signal, (int)ready);
					}
				}
			}

			if (!any_polled && activity) {
				/* If we have waiting sockets but couldn't poll any of them
				 * (because module is asleep), break the loop to prevent a 100% CPU
				 * spin-loop. The SRDY interrupt or timeout will wake us later. */
				activity = false;
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
			struct erpc_wifi_socket *s = &sockets[i];
			memset(s, 0, sizeof(*s));
			s->fd = fd;
			s->zfd = zfd;
			s->in_use = true;
			s->waiting = false;
			k_poll_signal_init(&s->poll_signal);
			k_sem_init(&s->read_sem, 0, 1);
			k_fifo_init(&s->fifo);
			s->recv_timeout_ms = 0;
			s->triggered_events = 0;
			socket = s;
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

static void erpc_wifi_socket_free(struct erpc_wifi_socket *sock)
{
	k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
	sock->in_use = false;
	sock->connected = false;
	sock->connect_pending = false;
	sock->waiting = false;
	k_spin_unlock(&sock->state_lock, key);
}

static int erpc_wifi_socket_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) {
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__, __w);
		errno = -__w;
		return -1;
	}

	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	if (sock->tcp_dpm_filter_set && sock->bound_port != 0) {
		erpc_wifi_dpm_tcp_port_delete_t dpm_msg = {.port = sock->bound_port};
		(void)erpc_wifi_send_cmd(ERPC_WIFI_DPM_TCP_PORT_DELETE_CMD, &dpm_msg, sizeof(dpm_msg), -1);
		sock->tcp_dpm_filter_set = false;
		LOG_INF("TCP DPM wake filter removed for port %u (via queue)", sock->bound_port);
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
		erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
		erpc_wifi_ps_notify_wakeup();
		errno = -ret;
		return -1;
	}

	/* Use message queue for FIFO-serialized socket operation */
	erpc_wifi_msg_bind_t bind_msg = {
		.fd = sock->fd,
		.addr_erpc_wifi = addr_erpc_wifi
	};
	ret = erpc_wifi_send_cmd(ERPC_WIFI_BIND_CMD, &bind_msg, sizeof(bind_msg), -1);

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();

	LOG_DBG("ra6w1_bind: %d", ret);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
}

static int erpc_wifi_socket_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	erpc_wifi_ps_notify_socket_connect_start();

	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		erpc_wifi_ps_notify_socket_connect_failed();
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		errno = -__w;
		return -1;
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
		if (sock->type == SOCK_STREAM) {
			strncpy(sock->addr_str, addr_str, sizeof(sock->addr_str) - 1);
			sock->addr_str[sizeof(sock->addr_str) - 1] = '\0';
			sock->port = ntohs(s_addr->sin6_port);
		}
	}
#endif

	if (sock->type == SOCK_STREAM && sock->bound_port == 0) {
		if (sock->family == AF_INET) {
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
#if defined(CONFIG_NET_IPV6)
		else if (sock->family == AF_INET6) {
			struct sockaddr_in6 local_addr6 = {
				.sin6_family = AF_INET6,
				.sin6_port = htons(55000 + sock->fd),
				.sin6_addr = IN6ADDR_ANY_INIT,
			};
			int bind_ret = erpc_wifi_socket_bind(sock, (const struct sockaddr *)&local_addr6, sizeof(local_addr6));
			if (bind_ret < 0) {
				LOG_WRN("Auto-bind IPv6 to port %u failed: %d", 55000 + sock->fd, bind_ret);
			} else {
				LOG_INF("Auto-bound IPv6 TCP client socket to port %u for DPM tracking", 55000 + sock->fd);
			}
		}
#endif
	}

	ret = erpc_wifi_socket_addr_from_posix(addr, &addr_erpc_wifi);
	if (ret) {
		erpc_wifi_ps_notify_socket_connect_failed();
		return ret;
	}

	/* Use message queue for FIFO-serialized socket operation */
	erpc_wifi_msg_connect_t connect_msg = {
		.fd = sock->fd,
		.addr_erpc_wifi = addr_erpc_wifi
	};
	ret = erpc_wifi_send_cmd(ERPC_WIFI_CONNECT_CMD, &connect_msg, sizeof(connect_msg), -1);
	k_msleep(100);

	k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
	if (ret == 0) {
		sock->connected = true;
		sock->connect_pending = false;
		sock->socket_error = 0;
		memcpy(&sock->addr, addr, sizeof(sock->addr));
		sock->addrlen = addrlen;
		k_spin_unlock(&sock->state_lock, key);
		/* Register TCP DPM wake filter BEFORE releasing the RAM/awake constraint.
		 * If done after pmgr_ram_release + ps_notify_wakeup the module may have
		 * started its sleep transition and the eRPC call gets a CRC error.
		 * Now uses queue-based command for thread-safe execution. */
		if (sock->type == SOCK_STREAM && sock->bound_port != 0 &&
		    !sock->tcp_dpm_filter_set) {
			erpc_wifi_dpm_tcp_port_filter_set_t dpm_msg = {.port = sock->bound_port};
			int32_t rc = erpc_wifi_send_cmd(ERPC_WIFI_DPM_TCP_PORT_FILTER_SET_CMD, &dpm_msg, sizeof(dpm_msg), -1);
			if (rc == 0) {
				sock->tcp_dpm_filter_set = true;
				LOG_INF("TCP DPM wake filter set for connected port %u (via queue)",
					sock->bound_port);
			} else {
				LOG_WRN("Failed to set TCP DPM wake filter for connected port %u (rc=%d, via queue)",
					sock->bound_port, rc);
			}
		}
		erpc_wifi_ps_notify_socket_connected();
	} else if (ret == -EINPROGRESS || ret == -EALREADY) {
		sock->connected = false;
		sock->connect_pending = true;
		sock->socket_error = 0;
		k_spin_unlock(&sock->state_lock, key);
		LOG_INF("Non-blocking connect in progress for fd %d", sock->fd);
	} else {
		sock->connected = false;
		sock->connect_pending = false;
		sock->socket_error = -ret;
		k_spin_unlock(&sock->state_lock, key);
		erpc_wifi_ps_notify_socket_connect_failed();
	}

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_connect: %d", ret);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}
	return 0;
}

static int erpc_wifi_socket_listen(void *obj, int backlog)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) {
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__, __w);
		errno = -__w;
		return -1;
	}

	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("fd: %d", sock->fd);
	LOG_DBG("backlog: %d", backlog);

	/* Use message queue for FIFO-serialized socket operation */
	erpc_wifi_msg_listen_t listen_msg = {
		.fd = sock->fd,
		.backlog = backlog
	};
	ret = erpc_wifi_send_cmd(ERPC_WIFI_LISTEN_CMD, &listen_msg, sizeof(listen_msg), -1);

	LOG_DBG("ra6w1_listen: %d", ret);

	/* Register TCP DPM wake filter using queue-based command (thread-safe) */
	if (ret == 0 && sock->type == SOCK_STREAM && sock->bound_port != 0 &&
	    !sock->tcp_dpm_filter_set) {
		erpc_wifi_dpm_tcp_port_filter_set_t dpm_msg = {.port = sock->bound_port};
		int32_t rc = erpc_wifi_send_cmd(ERPC_WIFI_DPM_TCP_PORT_FILTER_SET_CMD, &dpm_msg, sizeof(dpm_msg), -1);
		if (rc == 0) {
			sock->tcp_dpm_filter_set = true;
			LOG_INF("TCP DPM wake filter set for port %u (via queue)", sock->bound_port);
		} else {
			LOG_WRN("Failed to set TCP DPM wake filter for port %u (rc=%d, via queue)",
				sock->bound_port, rc);
		}
	}

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
}

static int erpc_wifi_socket_accept(void *obj, struct sockaddr *addr, socklen_t *addrlen)
{
	int fd;
	int conn_fd;
	struct ra_erpc_sockaddr remote_addr;
	uint32_t remote_addrlen = sizeof(remote_addr);
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;
	struct erpc_wifi_socket *socket;

	LOG_DBG("fd: %d", sock->fd);

	if ((addr == NULL) != (addrlen == NULL)) {
		errno = EINVAL;
		return -1;
	}

	for (;;) {
		k_spinlock_key_t key2 = k_spin_lock(&sock->state_lock);
		if (sock->triggered_events & (SOCKET_EVENT_RX | SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE)) {
			sock->triggered_events &= ~SOCKET_EVENT_RX;
			k_spin_unlock(&sock->state_lock, key2);
			break;
		}
		
		if (sock->flags & O_NONBLOCK) {
			k_spin_unlock(&sock->state_lock, key2);
			errno = EAGAIN;
			return -1;
		}

		sock->waiting = true;
		sock->poll_events = ZVFS_POLLIN;
		k_poll_signal_reset(&sock->poll_signal);
		k_spin_unlock(&sock->state_lock, key2);
		k_sem_give(&poll_task_sem);

		struct k_poll_event event;
		k_poll_event_init(&event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sock->poll_signal);

		int poll_rc = k_poll(&event, 1, K_FOREVER);
		k_spinlock_key_t key3 = k_spin_lock(&sock->state_lock);
		sock->waiting = false;
		k_spin_unlock(&sock->state_lock, key3);

		if (poll_rc < 0 && poll_rc != -EAGAIN) {
			errno = -poll_rc;
			return -1;
		}
	}

	fd = zvfs_reserve_fd();
	if (fd < 0) {
		errno = ENFILE;
		return -1;
	}

	erpc_wifi_ps_hold_during_recv();
	int __w = erpc_wifi_ensure_awake_rx(ERPC_PMGR_JOB_ID_RECV);
	if (__w != 0) {
		zvfs_free_fd(fd);
		errno = -__w;
		return -1;
	}

	/* Accept returns file descriptor for new connected socket */
	erpc_wifi_msg_accept_t accept_msg = { .fd = sock->fd, .addr_erpc_wifi = &remote_addr, .addrlen = &remote_addrlen };
	conn_fd = (int)erpc_wifi_send_cmd(ERPC_WIFI_ACCEPT_CMD, &accept_msg, sizeof(accept_msg), -1);
	LOG_DBG("ra6w1_accept: %d", conn_fd);

	if (conn_fd < 0) {
		erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
		erpc_wifi_ps_notify_wakeup();
		zvfs_free_fd(fd);
		errno = -conn_fd;
		return -1;
	}

	if (addr != NULL && addrlen != NULL) {
		socklen_t caller_len = *addrlen;

		if (sock->family == AF_INET && caller_len < sizeof(struct sockaddr_in)) {
			(void)erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, &conn_fd, sizeof(conn_fd), -1);
			erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
			erpc_wifi_ps_notify_wakeup();
			zvfs_free_fd(fd);
			errno = EINVAL;
			return -1;
		}
#if defined(CONFIG_NET_IPV6)
		if (sock->family == AF_INET6 && caller_len < sizeof(struct sockaddr_in6)) {
			(void)erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, &conn_fd, sizeof(conn_fd), -1);
			erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
			erpc_wifi_ps_notify_wakeup();
			zvfs_free_fd(fd);
			errno = EINVAL;
			return -1;
		}
#endif

		int ret = erpc_wifi_socket_addr_to_posix(addr, &remote_addr);
		if (ret < 0) {
			(void)erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, &conn_fd, sizeof(conn_fd), -1);
			erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
			erpc_wifi_ps_notify_wakeup();
			zvfs_free_fd(fd);
			LOG_DBG("erpc_wifi_socket_addr_to_posix error: %d", ret);
			errno = -ret;
			return -1;
		}

		*addrlen = (addr->sa_family == AF_INET6)
			 ? sizeof(struct sockaddr_in6)
			 : sizeof(struct sockaddr_in);
	}

	/* Keep track of the new fd returned by accept */
	socket = erpc_wifi_socket_allocate(conn_fd, fd);

	if (socket == NULL) {
		(void)erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, &conn_fd, sizeof(conn_fd), -1);
		erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
		erpc_wifi_ps_notify_wakeup();
		zvfs_free_fd(fd);
		errno = ENFILE;
		return -1;
	}

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);
	erpc_wifi_ps_notify_wakeup();

	socket->family = sock->family;
	socket->type = sock->type;
	socket->protocol = sock->protocol;
	socket->connected = true;
	socket->connect_pending = false;
	socket->socket_error = 0;

	/* Associate zephyr file descriptor with the socket obj and offload table
	   ensures subsequent socket operations using zephyr fd will use the
	       appropriate RA6W1 fd */
	zvfs_finalize_typed_fd(fd, socket,
			       (const struct fd_op_vtable *)&erpc_wifi_socket_fd_op_vtable,
			       ZVFS_MODE_IFSOCK);

	return fd;
}

static void erpc_wifi_sync_recv_job_for_socket(struct erpc_wifi_socket *sock, const char *path)
{
	if (!erpc_wifi_ps_is_enabled()) {
		return;
	}

	if ((sock == NULL) || (sock->type != SOCK_STREAM) || (sock->bound_port == 0U)) {
		return;
	}

	/* PMGR RECV mapping on RA6W1 is global; bind it to the active TCP socket before I/O. */
	erpc_wifi_pmgr_job_name_t jn_msg = { .job_id = (uint32_t)sock->bound_port, .job_name = "ERPC_TCP_RECV" };
	int32_t rc = (int32_t)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_DPM_JOB_NAME_SET_CMD,
										   &jn_msg, sizeof(jn_msg), -1);

	if (rc < 0) {
		LOG_WRN("%s: failed to map ERPC_TCP_RECV to port %u (rc=%d)",
			path ? path : "unknown", sock->bound_port, rc);
	}
}

static ssize_t erpc_wifi_socket_sendto(void *obj, const void *buf, size_t len, int flags,
				       const struct sockaddr *dest_addr, socklen_t addrlen)
{
	LOG_INF("TX wake request (len=%u)", (unsigned int)len);

	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		errno = -__w;
		return -1;
	}

	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("fd: %d", sock->fd);
	LOG_DBG("len: %d", len);
	LOG_DBG("dest_addr: %x", (uint32_t)dest_addr);
	LOG_DBG("addrlen: %d", addrlen);

	int ra_flags = 0;
	if ((flags & ZSOCK_MSG_DONTWAIT) || (sock->flags & O_NONBLOCK)) {
		ra_flags |= 0x08;
	}

	if (dest_addr) {
		ret = erpc_wifi_socket_addr_from_posix(dest_addr, &addr_erpc_wifi);
		if (ret) {
			erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
			erpc_wifi_ps_notify_wakeup();
			errno = -ret;
			return -1;
		}
	}

	bool is_nonblock = (flags & ZSOCK_MSG_DONTWAIT) || (sock->flags & O_NONBLOCK);
	uint32_t timeout_ms = sock->send_timeout_ms == 0 ? UINT32_MAX : sock->send_timeout_ms;
	int64_t start_time = k_uptime_get();

	for (;;) {
		erpc_wifi_sync_recv_job_for_socket(sock, "sendto");

		if (dest_addr) {
			erpc_wifi_msg_sendto_t st_msg = { .fd = sock->fd, .buff = (uint8_t *)buf, .buff_size = len,
										  .flags = ra_flags, .addr_erpc_wifi = &addr_erpc_wifi };
			ret = erpc_wifi_send_cmd(ERPC_WIFI_SENDTO_CMD, &st_msg, sizeof(st_msg), -1);
			LOG_DBG("ra6w1_sendto: %d", ret);
		} else {
			erpc_wifi_msg_send_t s_msg = { .fd = sock->fd, .buff = (uint8_t *)buf, .buff_size = len,
									   .flags = ra_flags };
			ret = erpc_wifi_send_cmd(ERPC_WIFI_SEND_CMD, &s_msg, sizeof(s_msg), -1);
			LOG_DBG("ra6w1_send: %d", ret);
		}

		if (ret >= 0) {
			break;
		}

		if (ret != -EAGAIN && ret != -EWOULDBLOCK) {
			break;
		}

		if (is_nonblock) {
			k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
			sock->triggered_events &= ~SOCKET_EVENT_TX;
			k_spin_unlock(&sock->state_lock, key);
			break;
		}

		/* Blocking mode: wait for POLLOUT */
		int64_t elapsed = k_uptime_get() - start_time;
		if (timeout_ms != UINT32_MAX && elapsed >= (int64_t)timeout_ms) {
			ret = -ETIMEDOUT;
			break;
		}

		k_timeout_t rem_timeout = (timeout_ms == UINT32_MAX) ? K_FOREVER : K_MSEC(timeout_ms - elapsed);

		k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
		if (sock->triggered_events & (SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE)) {
			k_spin_unlock(&sock->state_lock, key);
			ret = -EPIPE;
			break;
		}
		sock->triggered_events &= ~SOCKET_EVENT_TX;
		sock->waiting = true;
		sock->poll_events = ZVFS_POLLOUT;
		k_poll_signal_reset(&sock->poll_signal);
		k_spin_unlock(&sock->state_lock, key);

		k_sem_give(&poll_task_sem);

		struct k_poll_event event;
		k_poll_event_init(&event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sock->poll_signal);

		/* Release constraints while blocked */
		erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
		erpc_wifi_ps_notify_wakeup();

		int poll_rc = k_poll(&event, 1, rem_timeout);

		/* Re-acquire awake constraint before retrying */
		int __w2 = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
		if (__w2 != 0) {
			errno = -__w2;
			return -1;
		}

		key = k_spin_lock(&sock->state_lock);
		sock->waiting = false;
		k_spin_unlock(&sock->state_lock, key);

		if (poll_rc != 0) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();

	if (ret < 0) {
		errno = -ret;
		return -1;
	}
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
				return len > 0 ? len : -1;
			}

			len += ret;
			if (ret < msg->msg_iov[i].iov_len) {
				break;
			}
		}

		return len;
	}

	errno = EINVAL;
	return -1;
}

static ssize_t erpc_wifi_socket_recvfrom(void *obj, void *buf, size_t max_len, int flags,
					 struct sockaddr *src_addr, socklen_t *addrlen)
{
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	bool is_nonblock = (sock->flags & O_NONBLOCK) || (flags & ZSOCK_MSG_DONTWAIT);
	if (is_nonblock) {
		k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
		bool no_data = !(sock->triggered_events & (SOCKET_EVENT_RX | SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE));
		k_spin_unlock(&sock->state_lock, key);

		if (erpc_wifi_ps_is_enabled() && erpc_wifi_transport_slave_ready() == 0 && no_data) {
			errno = EAGAIN;
			return -1;
		}
	}

	int64_t start_time = k_uptime_get();
	uint32_t timeout_ms = sock->recv_timeout_ms;
	if (timeout_ms == 0) {
		timeout_ms = UINT32_MAX; // infinite
	}

	int ret;

	for (;;) {
		erpc_wifi_ps_hold_during_recv();

		/* Rebind global RECV mapping before wake/ready prep so ensure_awake_rx
		 * and subsequent recv operate on the same TCP socket context. */
		erpc_wifi_sync_recv_job_for_socket(sock, "recvfrom-prewake");

		int __w = erpc_wifi_ensure_awake_rx(ERPC_PMGR_JOB_ID_RECV);
		if (__w != 0) {
			LOG_INF("erpc_wifi_ensure_awake_rx failed: IN %s = %d", __func__, __w);
			errno = (int)-__w;
			return -1;
		}

		erpc_wifi_ps_hold_awake("recv-blocking");

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

			erpc_wifi_msg_recvfrom_t rf_msg = { .fd = sock->fd, .buff = buf, .buff_size = max_len,
											   .flags = ra_flags, .src_addr = &addr_erpc_wifi,
											   .src_addrlen = &addrlen_erpc };
			ret = erpc_wifi_send_cmd(ERPC_WIFI_RECVFROM_CMD, &rf_msg, sizeof(rf_msg), -1);
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
				{
					erpc_wifi_pmgr_job_t job_msg = { .job_id = ERPC_PMGR_JOB_ID_RECV };
					(void)erpc_wifi_send_cmd(ERPC_WIFI_PMGR_DPM_RCV_READY_SET_CMD,
										   &job_msg, sizeof(job_msg), -1);
				}
				erpc_wifi_msg_recv_t r_msg = { .fd = sock->fd, .buff = buf, .buff_size = max_len,
										  .flags = ra_flags };
				ret = erpc_wifi_send_cmd(ERPC_WIFI_RECV_CMD, &r_msg, sizeof(r_msg), -1);

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
			k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
			sock->triggered_events &= ~SOCKET_EVENT_RX;
			k_spin_unlock(&sock->state_lock, key);
			return ret;
		}
		/* If MSG_DONTWAIT was specified in this call, or O_NONBLOCK is configured on the socket,
		 * return -EAGAIN immediately.
		 */
		if ((flags & ZSOCK_MSG_DONTWAIT) || (sock->flags & O_NONBLOCK)) {
			/* Nonblocking consumer is exiting now; drop stale RX latch to avoid
			 * immediate spin/rearm loops from old readiness state. */
			k_spinlock_key_t key_nb = k_spin_lock(&sock->state_lock);
			sock->triggered_events &= ~SOCKET_EVENT_RX;
			k_spin_unlock(&sock->state_lock, key_nb);
			errno = EAGAIN;
			return -1;
		}
		/* Check if our receive timeout has expired. */
		int64_t elapsed = k_uptime_get() - start_time;
		if (timeout_ms != UINT32_MAX && elapsed >= (int64_t)timeout_ms) {
			errno = EAGAIN;
			return -1;
		}

		/* Calculate remaining timeout for the poll block. */
		k_timeout_t rem_timeout = (timeout_ms == UINT32_MAX) ? K_FOREVER : K_MSEC(timeout_ms - elapsed);

		k_spinlock_key_t key2 = k_spin_lock(&sock->state_lock);
		if (sock->triggered_events & (SOCKET_EVENT_RX | SOCKET_EVENT_ERR | SOCKET_EVENT_CLOSE)) {
			k_spin_unlock(&sock->state_lock, key2);
			continue;
		}
		sock->triggered_events &= ~SOCKET_EVENT_RX;
		sock->waiting = true;
		sock->poll_events = ZVFS_POLLIN;
		k_poll_signal_reset(&sock->poll_signal);
		k_spin_unlock(&sock->state_lock, key2);
		k_sem_give(&poll_task_sem);

		struct k_poll_event event;
		k_poll_event_init(&event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sock->poll_signal);

		LOG_DBG("recvfrom: blocking on poll signal, remaining tmo: %lld ms", 
			(timeout_ms == UINT32_MAX) ? -1LL : (int64_t)(timeout_ms - elapsed));
		
		int poll_rc = k_poll(&event, 1, rem_timeout);
		k_spinlock_key_t key3 = k_spin_lock(&sock->state_lock);
		sock->waiting = false;
		k_spin_unlock(&sock->state_lock, key3);

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
		errno = -__w;
		return -1;
	}

	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	if (level == SOL_SOCKET && optname == SO_ERROR) {
		int remote_error = 0;
		socklen_t remote_len = sizeof(remote_error);

		if (optval == NULL || optlen == NULL ||
		    *optlen < sizeof(remote_error)) {
			errno = EINVAL;
			return -1;
		}

		erpc_wifi_msg_sockgetopt_t go_msg = { .fd = sock->fd, .level = ERPC_WIFI_SOL_SOCKET,
										  .optname = ERPC_WIFI_SO_ERROR,
										  .optval = (uint32_t *)&remote_error, .optlen = &remote_len };
		int rc = erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_GETOPT_CMD, &go_msg, sizeof(go_msg), -1);

		erpc_wifi_pmgr_ram_release(0);
		erpc_wifi_ps_notify_wakeup();

		if (rc < 0) {
			errno = -rc;
			return -1;
		}

		*(int *)optval = remote_error;
		*optlen = sizeof(remote_error);

		k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
		if (remote_error == 0) {
			sock->socket_error = 0;
			sock->triggered_events &= ~SOCKET_EVENT_ERR;
			if (sock->connect_pending && (sock->triggered_events & SOCKET_EVENT_TX)) {
				sock->connect_pending = false;
				sock->connected = true;
			}
		} else {
			sock->connected = false;
			sock->connect_pending = false;
			sock->socket_error = remote_error;
		}
		k_spin_unlock(&sock->state_lock, key);

		LOG_INF("SO_ERROR fd=%d rc=%d value=%d", sock->fd, rc, remote_error);
		return 0;
	}

	int ret;
	int level_erpc_wifi;
	int optname_erpc_wifi;

	LOG_DBG("level: %d", level);
	LOG_DBG("optname: %d", optname);

	ret = erpc_wifi_socket_level_from_posix(level, &level_erpc_wifi);
	if (ret) {
		errno = -ret;
		return -1;
	}

	ret = erpc_wifi_socket_option_from_posix(level, optname, &optname_erpc_wifi);
	if (ret) {
		errno = -ret;
		return -1;
	}

	erpc_wifi_msg_sockgetopt_t go_msg2 = { .fd = sock->fd, .level = level_erpc_wifi,
									   .optname = optname_erpc_wifi,
									   .optval = optval, .optlen = optlen };
	ret = erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_GETOPT_CMD, &go_msg2, sizeof(go_msg2), -1);

	erpc_wifi_pmgr_ram_release(0);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_getsockopt: %d", ret);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
}

static int erpc_wifi_socket_setsockopt(void *obj, int level, int optname, const void *optval,
				       socklen_t optlen)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		errno = -__w;
		return -1;
	}

	int ret;
	int level_erpc_wifi;
	int optname_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("level: %d", level);
	LOG_DBG("optname: %d", optname);
	printk("TLS setsockopt: fd=%d level=%d opt=%d len=%d\n", sock ? sock->fd : -1, level,
	       optname, optlen);

	if (level == SOL_SOCKET && optname == SO_RCVTIMEO && optval != NULL && optlen >= sizeof(struct timeval)) {
		const struct timeval *tv = (const struct timeval *)optval;
		sock->recv_timeout_ms = (uint32_t)(tv->tv_sec * 1000 + tv->tv_usec / 1000);
	}

	if (level == SOL_SOCKET && optname == SO_SNDTIMEO && optval != NULL && optlen >= sizeof(struct timeval)) {
		const struct timeval *tv = (const struct timeval *)optval;
		sock->send_timeout_ms = (uint32_t)(tv->tv_sec * 1000 + tv->tv_usec / 1000);
	}

	ret = erpc_wifi_socket_level_from_posix(level, &level_erpc_wifi);
	if (ret) {
		errno = -ret;
		return -1;
	}

	ret = erpc_wifi_socket_option_from_posix(level, optname, &optname_erpc_wifi);
	if (ret) {
		errno = -ret;
		return -1;
	}

	erpc_wifi_msg_socksetopt_t so_msg = { .fd = sock->fd, .level = level_erpc_wifi,
									  .optname = optname_erpc_wifi,
									  .optval = (uint32_t *)optval, .optlen = optlen };
	ret = erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_SETOPT_CMD, &so_msg, sizeof(so_msg), -1);

	erpc_wifi_pmgr_ram_release(0);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_setsockopt: %d", ret);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
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
		errno = -__w;
		return -1;
	}

	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("erpc_wifi_socket_close");

	if (sock->type == SOCK_DGRAM) {
		LOG_INF("UDP socket closed");
	}

	bool was_waiting = false;
	bool notify_failed = false;
	int log_fd = -1;

	k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
	sock->closing = true;
	if (sock->connect_pending) {
		sock->connect_pending = false;
		notify_failed = true;
		log_fd = sock->fd;
	}
	sock->connected = false;
	if (sock->waiting) {
		was_waiting = true;
		sock->waiting = false;
	}
	sock->triggered_events |= SOCKET_EVENT_CLOSE;
	k_spin_unlock(&sock->state_lock, key);

	if (notify_failed) {
		erpc_wifi_ps_notify_socket_connect_failed();
		LOG_INF("Pending connection check canceled for fd %d", log_fd);
	}

	if (was_waiting) {
		k_poll_signal_raise(&sock->poll_signal, SOCKET_EVENT_CLOSE);
	}

	if (sock->tcp_dpm_filter_set && sock->bound_port != 0) {
		erpc_wifi_dpm_tcp_port_delete_t dpm_msg = {.port = sock->bound_port};
		(void)erpc_wifi_send_cmd(ERPC_WIFI_DPM_TCP_PORT_DELETE_CMD, &dpm_msg, sizeof(dpm_msg), -1);
		sock->tcp_dpm_filter_set = false;
		LOG_INF("TCP DPM wake filter removed for port %u (via queue)", sock->bound_port);
	}

	/* Use message queue for FIFO-serialized socket close operation */
	ret = erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, &sock->fd, sizeof(sock->fd), -1);
	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_close: %d", ret);

	if (sock->lock != NULL) {
		k_mutex_unlock(sock->lock);
	}

	erpc_wifi_socket_free(sock);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
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

	bool immediate_ready = false;
	for (int i = 0; i < nfds; i++) {
		fds[i].revents = 0;
		struct erpc_wifi_socket *sock = find_socket_by_fd(fds[i].fd);

		if (!sock) {
			/* Mixed fd tables are valid in Zephyr (e.g. socket_service eventfd).
			 * Leave non-driver fds untouched so their owner can handle them.
			 */
			continue;
		}

		/* Synthetically report ZVFS_POLLOUT if requested, 
		   unless we are waiting for a background connect to finish. */
		bool notify_connected = false;
		k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
		uint32_t cached = (uint32_t)sock->triggered_events;
		uint32_t requested_mask = 0;
		if (fds[i].events & ZVFS_POLLIN) {
			requested_mask |= SOCKET_EVENT_RX;
		}
		if (fds[i].events & ZVFS_POLLOUT) {
			requested_mask |= SOCKET_EVENT_TX;
		}
		if (fds[i].events & ZVFS_POLLERR) {
			requested_mask |= SOCKET_EVENT_ERR;
		}
		if (fds[i].events & ZVFS_POLLHUP) {
			requested_mask |= SOCKET_EVENT_CLOSE;
		}

		if (sock->connect_pending && (cached & SOCKET_EVENT_TX)) {
			sock->connect_pending = false;
			sock->connected = true;
			sock->socket_error = 0;
			notify_connected = true;
		}

		bool writable = (sock->type == SOCK_DGRAM) ||
				(sock->type == SOCK_STREAM && sock->connected && !sock->connect_pending && sock->socket_error == 0);

		/* Synthetically report ZVFS_POLLOUT if writable and requested */
		if ((fds[i].events & ZVFS_POLLOUT) && writable) {
			fds[i].revents |= ZVFS_POLLOUT;
			sock->triggered_events |= SOCKET_EVENT_TX;
			immediate_ready = true;
			k_spin_unlock(&sock->state_lock, key);
			if (notify_connected) {
				erpc_wifi_ps_notify_socket_connected();
			}
			continue;
		}

		uint32_t ready = sock->triggered_events & requested_mask;
		if (ready != 0) {
			/* Already has ready events, will be filled in update_revents. Just continue without blocking. */
			immediate_ready = true;
			k_spin_unlock(&sock->state_lock, key);
			if (notify_connected) {
				erpc_wifi_ps_notify_socket_connected();
			}
			continue;
		}

		if (tracked_count < ERPC_WIFI_MAX_SOCKETS) {
			k_poll_signal_reset(&sock->poll_signal);
			sock->poll_events = fds[i].events;
			sock->waiting = true;

			k_poll_event_init(&events[tracked_count], K_POLL_TYPE_SIGNAL,
					  K_POLL_MODE_NOTIFY_ONLY, &sock->poll_signal);
			tracked_socks[tracked_count] = sock;
			tracked_count++;
		}
		k_spin_unlock(&sock->state_lock, key);

		if (notify_connected) {
			erpc_wifi_ps_notify_socket_connected();
		}
	}

	if (tracked_count > 0) {
		k_sem_give(&poll_task_sem);
	}

	/* Short-circuit poll with POLLHUP when interface is already down. */
	int hup_count = erpc_wifi_poll_hup_on_iface_down(fds, nfds);
	if (hup_count > 0) {
		LOG_DBG("poll offload iface-down hup_count=%d", hup_count);
		return hup_count;
	}

	if (immediate_ready || timeout == 0) {
		goto update_revents;
	}

	if (tracked_count > 0) {
		k_timeout_t poll_timeout = (timeout < 0) ? K_FOREVER : K_MSEC(timeout);
		int ret = k_poll(events, tracked_count, poll_timeout);

		if (ret != 0 && ret != -EAGAIN) {
			LOG_ERR("k_poll failed: %d", ret);
		}
	}

update_revents:
	ready_count = 0;
	for (int i = 0; i < nfds; i++) {
		struct erpc_wifi_socket *sock = find_socket_by_fd(fds[i].fd);

		if (sock) {
			k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
			sock->waiting = false;
			uint32_t ev = (uint32_t)sock->triggered_events;

			if ((ev & SOCKET_EVENT_RX) && (fds[i].events & ZVFS_POLLIN)) {
				fds[i].revents |= ZVFS_POLLIN;
			}
			if ((ev & SOCKET_EVENT_TX) && (fds[i].events & ZVFS_POLLOUT)) {
				fds[i].revents |= ZVFS_POLLOUT;
			}
			if (ev & SOCKET_EVENT_ERR) {
				fds[i].revents |= ZVFS_POLLERR;
			}
			if (ev & SOCKET_EVENT_CLOSE) {
				fds[i].revents |= ZVFS_POLLHUP;
			}

			if (fds[i].revents != 0) {
				ready_count++;
			}
			k_spin_unlock(&sock->state_lock, key);
		}
	}

	LOG_INF("poll offload done ready_count=%d", ready_count);
	return ready_count;
}

static int erpc_wifi_socket_poll_update(struct erpc_wifi_socket *sock, struct zvfs_pollfd *pfd, struct k_poll_event **pev)
{
	if (!sock) {
		/* Not owned by this driver: ignore and let owner handle update path. */
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
	sock->waiting = false;

	if (*pev != NULL && (*pev)->type == K_POLL_TYPE_SIGNAL) {
		if ((*pev)->state == K_POLL_STATE_SIGNALED) {
			k_poll_signal_reset(&sock->poll_signal);
			uint32_t ev = (uint32_t)sock->triggered_events;

			if ((ev & SOCKET_EVENT_RX) && (pfd->events & ZVFS_POLLIN)) {
				pfd->revents |= ZVFS_POLLIN;
			}
			if ((ev & SOCKET_EVENT_TX) && (pfd->events & ZVFS_POLLOUT)) {
				pfd->revents |= ZVFS_POLLOUT;
			}
			if (ev & SOCKET_EVENT_ERR) {
				pfd->revents |= ZVFS_POLLERR;
			}
			if (ev & SOCKET_EVENT_CLOSE) {
				pfd->revents |= ZVFS_POLLHUP;
			}
		}
		(*pev)++;
	}
	k_spin_unlock(&sock->state_lock, key);

	return 0;
}

static int erpc_wifi_socket_ioctl(void *obj, unsigned int request, va_list args)
{
	struct erpc_wifi_socket *sock = obj;

	switch (request) {
#define ERPC_WIFI_SOL_SOCKET  0xfff
#define ERPC_WIFI_SO_NONBLOCK 0x2000

	case F_SETFL: {
		int new_flags = va_arg(args, int);
		int old_flags = sock->flags;
		
		if ((new_flags ^ old_flags) & O_NONBLOCK) {
			uint32_t val = (new_flags & O_NONBLOCK) ? 1U : 0U;
			int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
			if (__w != 0) {
				LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__, __w);
				errno = -__w;
				return -1;
			}
			erpc_wifi_msg_socksetopt_t nb_msg = { .fd = sock->fd,
											  .level = ERPC_WIFI_SOL_SOCKET,
											  .optname = ERPC_WIFI_SO_NONBLOCK,
											  .optval = &val, .optlen = sizeof(val) };
			int rc = erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_SETOPT_CMD, &nb_msg, sizeof(nb_msg), -1);
			erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);
			erpc_wifi_ps_notify_wakeup();
			if (rc < 0) {
				errno = -rc;
				return -1;
			}
			LOG_INF("Forwarded O_NONBLOCK to server for fd %d: val=%u", sock->fd, val);
		}
		sock->flags = new_flags;
		return 0;
	}
	case F_GETFL:
		LOG_INF("Socket flags request: %d: 0x%x", request, sock->flags);
		return sock->flags;

	case ZFD_IOCTL_POLL_PREPARE: {
		struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
		struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);

		// LOG_INF("POLL_PREPARE fd=%d events=0x%x", pfd->fd, pfd->events);

		if (*pev >= pev_end) {
			return -ENOSPC;
		}

		ensure_poll_task_started();

		k_spinlock_key_t key = k_spin_lock(&sock->state_lock);
		uint32_t cached = (uint32_t)sock->triggered_events;
		uint32_t requested_mask = 0;

		if (pfd->events & ZVFS_POLLIN) {
			requested_mask |= SOCKET_EVENT_RX;
		}
		if (pfd->events & ZVFS_POLLOUT) {
			requested_mask |= SOCKET_EVENT_TX;
		}
		if (pfd->events & ZVFS_POLLERR) {
			requested_mask |= SOCKET_EVENT_ERR;
		}
		if (pfd->events & ZVFS_POLLHUP) {
			requested_mask |= SOCKET_EVENT_CLOSE;
		}

		bool notify_connected = false;
		if (sock->connect_pending && (cached & SOCKET_EVENT_TX)) {
			sock->connect_pending = false;
			sock->connected = true;
			sock->socket_error = 0;
			notify_connected = true;
		}

		k_poll_signal_reset(&sock->poll_signal);
		sock->poll_events = pfd->events;
		sock->waiting = true;

		k_poll_event_init(*pev, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
				  &sock->poll_signal);
		(*pev)++;

		uint32_t ready = sock->triggered_events & requested_mask;
		bool writable = (sock->type == SOCK_DGRAM) ||
				(sock->type == SOCK_STREAM && sock->connected && !sock->connect_pending && sock->socket_error == 0);

		bool do_signal = false;
		int sig_val = 0;

		if (ready != 0) {
			sock->waiting = false;
			do_signal = true;
			sig_val = (int)ready;
		} else if ((pfd->events & ZVFS_POLLOUT) && writable) {
			/* Synthetically report POLLOUT if writable */
			sock->triggered_events |= SOCKET_EVENT_TX;
			sock->waiting = false;
			do_signal = true;
			sig_val = SOCKET_EVENT_TX;
		} else {
			k_sem_give(&poll_task_sem);
		}
		k_spin_unlock(&sock->state_lock, key);

		if (notify_connected) {
			erpc_wifi_ps_notify_socket_connected();
		}

		if (do_signal) {
			k_poll_signal_raise(&sock->poll_signal, sig_val);
		}

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

		int ret = erpc_wifi_socket_poll_update(sock, pfd, pev);
		// LOG_INF("POLL_UPDATE fd=%d events=0x%x revents=0x%x ret=%d", pfd->fd, pfd->events, pfd->revents, ret);
		return ret;
	}
#if 1
	case ZFD_IOCTL_SET_LOCK: {
		struct k_mutex *lock = va_arg(args, struct k_mutex *);
		sock->lock = lock;
		printk("SET_LOCK stored lock=%p zfd=%d remote=%d\n", (void *)lock, sock->zfd,
		       sock->fd);
		return 0;
	}
#endif
	default:
		errno = EINVAL;
		return -1;
	}
}

static int erpc_wifi_socket_getpeername(void *obj, struct sockaddr *addr, socklen_t *addrlen)
{
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	if (sock == NULL || !sock->in_use) {
		errno = EBADF;
		return -1;
	}

	if (!sock->connected) {
		errno = ENOTCONN;
		return -1;
	}

	if (addr == NULL || addrlen == NULL) {
		errno = EINVAL;
		return -1;
	}

	if (sock->family == AF_INET) {
		if (*addrlen < sizeof(struct sockaddr_in)) {
			errno = EINVAL;
			return -1;
		}

		struct sockaddr_in *sin = (struct sockaddr_in *)addr;
		memset(sin, 0, sizeof(*sin));
		sin->sin_family = AF_INET;
		sin->sin_port = htons(sock->port);

		if (net_addr_pton(AF_INET, sock->addr_str, &sin->sin_addr) < 0) {
			errno = EINVAL;
			return -1;
		}

		*addrlen = sizeof(*sin);
	}
#if defined(CONFIG_NET_IPV6)
	else if (sock->family == AF_INET6) {
		if (*addrlen < sizeof(struct sockaddr_in6)) {
			errno = EINVAL;
			return -1;
		}

		struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)addr;
		memset(sin6, 0, sizeof(*sin6));
		sin6->sin6_family = AF_INET6;
		sin6->sin6_port = htons(sock->port);

		if (net_addr_pton(AF_INET6, sock->addr_str, &sin6->sin6_addr) < 0) {
			errno = EINVAL;
			return -1;
		}

		*addrlen = sizeof(*sin6);
	}
#endif
	else {
		errno = EAFNOSUPPORT;
		return -1;
	}

	return 0;
}

static int erpc_wifi_socket_getsockname(void *obj, struct sockaddr *addr, socklen_t *addrlen)
{
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	if (sock == NULL || !sock->in_use) {
		errno = EBADF;
		return -1;
	}

	if (addr == NULL || addrlen == NULL) {
		errno = EINVAL;
		return -1;
	}

	if (sock->family == AF_INET) {
		if (*addrlen < sizeof(struct sockaddr_in)) {
			errno = EINVAL;
			return -1;
		}

		struct sockaddr_in *sin = (struct sockaddr_in *)addr;
		memset(sin, 0, sizeof(*sin));
		sin->sin_family = AF_INET;
		sin->sin_port = htons(sock->bound_port);
		sin->sin_addr.s_addr = INADDR_ANY;

		*addrlen = sizeof(*sin);
	}
#if defined(CONFIG_NET_IPV6)
	else if (sock->family == AF_INET6) {
		if (*addrlen < sizeof(struct sockaddr_in6)) {
			errno = EINVAL;
			return -1;
		}

		struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)addr;
		memset(sin6, 0, sizeof(*sin6));
		sin6->sin6_family = AF_INET6;
		sin6->sin6_port = htons(sock->bound_port);
		memset(&sin6->sin6_addr, 0, sizeof(struct in6_addr)); // IN6ADDR_ANY

		*addrlen = sizeof(*sin6);
	}
#endif
	else {
		errno = EAFNOSUPPORT;
		return -1;
	}

	return 0;
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
	.getpeername = erpc_wifi_socket_getpeername,
	.getsockname = erpc_wifi_socket_getsockname,

};

static int erpc_wifi_socket_create(int family, int type, int proto)
{
	int __w = erpc_wifi_ensure_awake_tx(ERPC_PMGR_JOB_ID_SEND);
	if (__w != 0) { 
		LOG_INF("erpc_wifi_ensure_awake_tx failed: IN %s = %d", __func__,__w);
		errno = -__w;
		return -1;
	}

	int err;
	int sock;
	uint8_t family_erpc_wifi;
	int fd = zvfs_reserve_fd();
	struct erpc_wifi_socket *socket = NULL;

	if (fd < 0) {
		errno = ENFILE;
		erpc_wifi_pmgr_ram_release(0);
		erpc_wifi_ps_notify_wakeup();
		return -1;
	}

	LOG_DBG("erpc_wifi_socket_create");
	LOG_DBG("family: %d", family);
	LOG_DBG("type: %d", type);
	LOG_DBG("proto: %d", proto);

	/* Map Zephyr socket.h family to RA RPC's */
	err = erpc_wifi_socket_family_from_posix(family, &family_erpc_wifi);
	if (err) {
		LOG_ERR("unsupported family: %d", family);
		zvfs_free_fd(fd);
		erpc_wifi_pmgr_ram_release(0);
		erpc_wifi_ps_notify_wakeup();
		errno = -err;
		return -1;
	}
	erpc_wifi_msg_socket_create_t sc_msg = { .domain = family_erpc_wifi, .type = type, .protocol = proto };
	sock = erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CREATE_CMD, &sc_msg, sizeof(sc_msg), -1);
	erpc_wifi_pmgr_ram_release(0);
	erpc_wifi_ps_notify_wakeup();
	LOG_DBG("ra6w1_socket: %d", sock);

	if (sock < 0) {
		zvfs_free_fd(fd);
		errno = -sock;
		return -1;
	}

	socket = erpc_wifi_socket_allocate(sock, fd);

	if (socket == NULL) {
		(void)erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, &sock, sizeof(sock), -1);
		zvfs_free_fd(fd);
		errno = ENFILE;
		return -1;
	}
	
	socket->family = family;
	socket->type = type;
	socket->protocol = proto;

	if (type == SOCK_DGRAM) {
		LOG_INF("UDP socket created");
	}

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

static void erpc_wifi_offload_srdy_callback(void)
{
	/* Wake up the poll task immediately on SRDY interrupt */
	extern struct k_sem poll_task_sem;
	k_sem_give(&poll_task_sem);
}

extern void erpc_wifi_dns_offload_init(void);

int erpc_wifi_socket_offload_init(struct net_if *iface)
{
	memset(sockets, 0, sizeof(sockets));
	k_sem_init(&poll_task_sem, 0, 1);

	erpc_transport_register_srdy_cb(erpc_wifi_offload_srdy_callback);

	net_if_socket_offload_set(iface, erpc_wifi_socket_create);

	net_iface = iface;

#ifndef CONFIG_DNS_RESOLVER
	erpc_wifi_dns_offload_init();
#endif

	return 0;
}

#ifdef CONFIG_NET_SOCKETS_OFFLOAD
NET_SOCKET_OFFLOAD_REGISTER(erpc_wifi, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, AF_UNSPEC,
			    erpc_wifi_socket_is_supported, erpc_wifi_socket_create);
#endif