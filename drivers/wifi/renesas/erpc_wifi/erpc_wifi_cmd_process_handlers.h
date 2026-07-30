#pragma once

#include "erpc_wifi_cmd.h"

/* Event flags that match socket event notifications */
#define SOCKET_EVENT_RX    0x01  /* POLLIN - Data available to read */
#define SOCKET_EVENT_TX    0x02  /* POLLOUT - Ready to write */
#define SOCKET_EVENT_ERR   0x04  /* POLLERR - Error condition */
#define SOCKET_EVENT_CLOSE 0x08  /* POLLHUP - Connection closed */

/* PMGR DPM job ids used by RA6W1 socket shim */
#ifndef ERPC_PMGR_JOB_ID_SEND
#define ERPC_PMGR_JOB_ID_SEND (1U)
#endif
#ifndef ERPC_PMGR_JOB_ID_RECV
#define ERPC_PMGR_JOB_ID_RECV (2U)
#endif

/* Handler initialization functions */
int erpc_wifi_cmd_socket_handlers_init(void);
int erpc_wifi_cmd_ap_handlers_init(void);
