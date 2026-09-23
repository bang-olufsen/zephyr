#include "erpc_wifi_cmd.h"

#include "erpc_wifi.h"
#include "erpc_wifi_socket_offload.h"
#include <c_wifi_host_to_ra_client.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(erpc_wifi_cmd, CONFIG_WIFI_LOG_LEVEL);

#define ERPC_WIFI_MSG_MAX 64
#define MSG_TASK_STACK_SIZE 3200
#define ERPC_WIFI_CMD_TIMEOUT_MARGIN_MS 1000

K_THREAD_STACK_DEFINE(msg_task_stack, MSG_TASK_STACK_SIZE);

static struct k_thread msgq_thread;
static k_tid_t msgq_task_tid;

/*
 * iface up/down support:
 *  - g_cmd_suspended: while set (iface_down teardown), no command may reach the
 *    eRPC client. New requests fail with -ENETDOWN and queued requests are
 *    completed with -ENETDOWN without touching the transport.
 *  - g_cmd_handler_busy: set while the handler thread owns a dequeued message,
 *    so erpc_wifi_cmd_suspend() can wait for an in-flight eRPC call to finish
 *    before the client/transport are deinitialized.
 */
static atomic_t g_cmd_suspended;
static atomic_t g_cmd_handler_busy;

/*
 * Link health (see erpc_wifi_cmd_link_stats()). Seconds are used for the
 * timestamp so the 32-bit atomic cannot wrap during a long-run test.
 */
static atomic_t g_cmd_fail_streak;
static atomic_t g_cmd_last_ok_s;

static void erpc_wifi_cmd_account_result(erpc_wifi_cmd_t cmd, int ret)
{
	/*
	 * The server-event poll carries no liveness information: its handler
	 * (erpc_wifi_get_server_evt_msg_process) returns 0 unconditionally,
	 * whatever the eRPC call did. It runs every ~200 ms, so counting it as a
	 * success would clear the failure streak continuously and the link
	 * watchdog could never fire on a wedged link.
	 */
	if (cmd == EPRC_WIFI_GET_SERVER_EVT_CMD) {
		return;
	}

	if (ret >= 0) {
		atomic_set(&g_cmd_fail_streak, 0);
		atomic_set(&g_cmd_last_ok_s, (atomic_val_t)(k_uptime_get() / 1000));
		return;
	}

	/* Teardown rejects every command by design; that is not a link fault. */
	if (ret == -ENETDOWN) {
		return;
	}

	atomic_inc(&g_cmd_fail_streak);
}

void erpc_wifi_cmd_link_stats(uint32_t *fail_streak, uint32_t *secs_since_ok)
{
	int64_t now_s = k_uptime_get() / 1000;
	atomic_val_t last_s = atomic_get(&g_cmd_last_ok_s);

	if (fail_streak) {
		*fail_streak = (uint32_t)atomic_get(&g_cmd_fail_streak);
	}
	if (secs_since_ok) {
		int64_t d = now_s - (int64_t)last_s;

		*secs_since_ok = (d > 0) ? (uint32_t)d : 0U;
	}
}

void erpc_wifi_cmd_link_stats_reset(void)
{
	atomic_set(&g_cmd_fail_streak, 0);
	atomic_set(&g_cmd_last_ok_s, (atomic_val_t)(k_uptime_get() / 1000));
}

K_MSGQ_DEFINE(cmd_msg_queue, sizeof(erpc_wifi_msg_data_t), ERPC_WIFI_MSG_MAX, 4);

typedef struct {
	erpc_wifi_cmd_t cmd;
	erpc_wifi_msg_handler_t h;
	void *p_user_data;
} erpc_wifi_msg_prosess_t;

static erpc_wifi_msg_prosess_t erpc_wifi_socket_handlers[ERPC_WIFI_LAST_CMD] = {
	[0 ... ERPC_WIFI_LAST_CMD - 1] = {0, NULL, NULL}
};

static bool is_erpc_wifi_cmd_handler_registered(erpc_wifi_cmd_t cmd)
{
	if (cmd >= ERPC_WIFI_LAST_CMD) {
		return false;
	}
	return erpc_wifi_socket_handlers[cmd].h != NULL;
}

static int alloc_cmd_msg_data(erpc_wifi_msg_data_t *msg, void *data, size_t size)
{
	if (!msg) {
		return -EINVAL;
	}

	/* There is nothing to copy */
	if (!data || size == 0) {
		return 0;
	}

	msg->data = k_calloc(1, size);
	if (!msg->data) {
		LOG_ERR("CMD ENOMEM: data allocation failed cmd=%d size=%u",
			msg->cmd, (unsigned int)size);
		return -ENOMEM;
	}

	memcpy(msg->data, data, size);

	return 0;
}

static void free_cmd_msg_data(erpc_wifi_msg_data_t *msg)
{
	if (msg && msg->data) {
		k_free(msg->data);
		msg->data = NULL;
	}
}

int erpc_wifi_send_cmd(erpc_wifi_cmd_t cmd, void *data, size_t size, int tout)
{
	int ret = 0;
	int effective_tout = tout;
	k_timeout_t timeout = K_NO_WAIT;
	erpc_wifi_cmd_ctx_t *ctx = NULL;
	erpc_wifi_msg_data_t msg = { .cmd = cmd, .ctx = NULL };

	if (cmd >= ERPC_WIFI_LAST_CMD) {
		return -ERANGE;
	}

	if (atomic_get(&g_cmd_suspended) != 0) {
		return -ENETDOWN;
	}

	if (!is_erpc_wifi_cmd_handler_registered(cmd)) {
		LOG_ERR("Command %d not registered", cmd);
		return -ENOENT;
	}

	ret = alloc_cmd_msg_data(&msg, data, size);
	if (ret < 0) {
		return ret;
	}

	if (effective_tout > 0 &&
	    effective_tout <= CONFIG_ERPC_SPI_READY_TIMEOUT_MS) {
		effective_tout = CONFIG_ERPC_SPI_READY_TIMEOUT_MS +
			ERPC_WIFI_CMD_TIMEOUT_MARGIN_MS;
		LOG_DBG("CMD timeout raised: cmd=%d requested=%d effective=%d transport=%d",
			cmd, tout, effective_tout, CONFIG_ERPC_SPI_READY_TIMEOUT_MS);
	}

	if (effective_tout != 0) {
		timeout = (effective_tout == -1) ? K_FOREVER : K_MSEC(effective_tout);

		ctx = k_malloc(sizeof(erpc_wifi_cmd_ctx_t));
		if (!ctx) {
			LOG_ERR("CMD ENOMEM: context allocation failed cmd=%d timeout=%d", cmd,
				effective_tout);
			free_cmd_msg_data(&msg);
			return -ENOMEM;
		}

		atomic_set(&ctx->ref_count, 2);
		k_sem_init(&ctx->sem, 0, 1);
		ctx->cmd_ret = 0;
		atomic_set(&ctx->timed_out, 0);

		msg.ctx = ctx;
	}

	if (k_msgq_put(&cmd_msg_queue, &msg, K_NO_WAIT) == 0) {
		if (ctx) {
			if (k_sem_take(&ctx->sem, timeout) != 0) {
				atomic_set(&ctx->timed_out, 1);
				LOG_ERR("CMD timeout: cmd=%d requested_ms=%d effective_ms=%d transport_ms=%d",
					cmd, tout, effective_tout, CONFIG_ERPC_SPI_READY_TIMEOUT_MS);
				ret = -ETIMEDOUT;
				/*
				 * A caller timeout is the dominant symptom of a wedged
				 * link: the handler then SKIPS the message, so it never
				 * reports a result of its own. Count it here or the
				 * failure streak stays near zero while the link is dead.
				 */
				erpc_wifi_cmd_account_result(cmd, ret);
			} else {
				ret = ctx->cmd_ret;
			}

			if (atomic_dec(&ctx->ref_count) == 1) {
				k_free(ctx);
			}
		}
	} else {
		uint32_t used = k_msgq_num_used_get(&cmd_msg_queue);
		free_cmd_msg_data(&msg);
		if (ctx) {
			k_free(ctx);
		}
		LOG_ERR("CMD queue full: cmd=%d used=%u max=%u",
			cmd, (unsigned int)used, (unsigned int)ERPC_WIFI_MSG_MAX);
		erpc_wifi_cmd_account_result(cmd, -EAGAIN);
		return -EAGAIN;
	}

	return ret;
}

int erpc_wifi_register_cmd_handler(erpc_wifi_cmd_t cmd, erpc_wifi_msg_handler_t h)
{
	if (cmd >= ERPC_WIFI_LAST_CMD || !h) {
		return -EINVAL;
	}

	erpc_wifi_socket_handlers[cmd].cmd = cmd;
	erpc_wifi_socket_handlers[cmd].h = h;
	erpc_wifi_socket_handlers[cmd].p_user_data = NULL;

	return 0;
}

int erpc_wifi_unregister_cmd_handler(erpc_wifi_cmd_t cmd)
{
	if (cmd >= ERPC_WIFI_LAST_CMD) {
		return -EINVAL;
	}

	erpc_wifi_socket_handlers[cmd].h = NULL;

	return 0;
}

/* Message handler thread - runs at high priority and processes commands FIFO */
static void erpc_wifi_msg_handler_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	erpc_wifi_msg_data_t msg = {0};

	while (1) {
		/* Wait for message - blocks here until message available */
		atomic_set(&g_cmd_handler_busy, 0);
		if (k_msgq_get(&cmd_msg_queue, &msg, K_FOREVER) != 0) {
			LOG_ERR("Failed to get message from queue");
			continue;
		}

		atomic_set(&g_cmd_handler_busy, 1);
		bool timed_out = false;
		if (msg.ctx && atomic_get(&msg.ctx->timed_out) != 0) {
			timed_out = true;
		}

		if (!timed_out && atomic_get(&g_cmd_suspended) != 0) {
			/* Session is being torn down: never touch the eRPC client. */
			if (msg.ctx) {
				msg.ctx->cmd_ret = -ENETDOWN;
			}
		} else if (!timed_out) {
			if (msg.cmd < ERPC_WIFI_LAST_CMD && erpc_wifi_socket_handlers[msg.cmd].h) {
				bool server_evt_query =
					(msg.cmd == EPRC_WIFI_GET_SERVER_EVT_CMD);

				if (server_evt_query) {
					erpc_wifi_offload_server_evt_query_begin();
				}

				/*
				 * Preserve the low-latency SRDY ownership rule: every handler is a
				 * host-initiated eRPC transaction, so response SRDY must not be
				 * classified as an autonomous DPM wake.
				 */
				erpc_wifi_offload_host_erpc_begin();
				int ret = erpc_wifi_socket_handlers[msg.cmd].h(msg.data);
				erpc_wifi_offload_host_erpc_end();

				erpc_wifi_cmd_account_result(msg.cmd, ret);

				if (server_evt_query) {
					erpc_wifi_offload_server_evt_query_end();
				}

				if (msg.ctx) {
					msg.ctx->cmd_ret = ret;
				}
			} else {
				LOG_ERR("No handler registered for command %d", msg.cmd);
				if (msg.ctx) {
					msg.ctx->cmd_ret = -ENOENT;
				}
			}
		} else {
			LOG_WRN("Command %d timed out by caller; skipping execution", msg.cmd);
		}

		free_cmd_msg_data(&msg);

		if (msg.ctx) {
			if (!timed_out) {
				k_sem_give(&msg.ctx->sem);
			}
			if (atomic_dec(&msg.ctx->ref_count) == 1) {
				k_free(msg.ctx);
			}
		}
	}
}

int erpc_wifi_cmd_init(void)
{
	if (msgq_task_tid != NULL) {
		return 0;
	}

	erpc_wifi_cmd_link_stats_reset();
	/* Create handler thread at preemptive priority to ensure queue processing */
	msgq_task_tid = k_thread_create(&msgq_thread, msg_task_stack, MSG_TASK_STACK_SIZE,
					erpc_wifi_msg_handler_task, NULL, NULL, NULL,
					K_PRIO_PREEMPT(10), 0, K_NO_WAIT);

	if (!msgq_task_tid) {
		LOG_ERR("Failed to create erpc_wifi message handler thread");
		return -ENOMEM;
	}

	k_thread_name_set(msgq_task_tid, "erpc_wifi_msg_handler");

	LOG_INF("erpc_wifi message queue initialized (capacity: %d)", ERPC_WIFI_MSG_MAX);

	return 0;
}

int erpc_wifi_cmd_suspend(uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;

	atomic_set(&g_cmd_suspended, 1);

	/* Queued messages are drained by the handler thread (it completes them with
	 * -ENETDOWN). Wait until the queue is empty and no message is in progress.
	 */
	while (k_msgq_num_used_get(&cmd_msg_queue) > 0U ||
	       atomic_get(&g_cmd_handler_busy) != 0) {
		if (msgq_task_tid == NULL) {
			k_msgq_purge(&cmd_msg_queue);
			break;
		}
		if (k_uptime_get() >= deadline) {
			LOG_WRN("CMD suspend: in-flight command did not finish within %u ms",
				(unsigned int)timeout_ms);
			return -ETIMEDOUT;
		}
		k_msleep(10);
	}

	LOG_INF("erpc_wifi command queue suspended");
	return 0;
}

void erpc_wifi_cmd_resume(void)
{
	/* New session: do not let the previous session's failures trigger a
	 * recovery before the first command of this one has even run.
	 */
	erpc_wifi_cmd_link_stats_reset();
	atomic_set(&g_cmd_suspended, 0);
}

bool erpc_wifi_cmd_is_suspended(void)
{
	return atomic_get(&g_cmd_suspended) != 0;
}
