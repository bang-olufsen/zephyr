#include "erpc_wifi_cmd.h"

#include "erpc_wifi.h"
#include "erpc_wifi_socket_offload.h"
#include <c_wifi_host_to_ra_client.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(erpc_wifi_cmd, CONFIG_WIFI_LOG_LEVEL);

#define ERPC_WIFI_MSG_MAX 64
#define MSG_TASK_STACK_SIZE 3200

K_THREAD_STACK_DEFINE(msg_task_stack, MSG_TASK_STACK_SIZE);

static struct k_thread msgq_thread;
static k_tid_t msgq_task_tid;

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
	k_timeout_t timeout = K_NO_WAIT;
	erpc_wifi_cmd_ctx_t *ctx = NULL;
	erpc_wifi_msg_data_t msg = { .cmd = cmd, .ctx = NULL };

	if (cmd >= ERPC_WIFI_LAST_CMD) {
		return -ERANGE;
	}

	if (!is_erpc_wifi_cmd_handler_registered(cmd)) {
		LOG_ERR("Command %d not registered", cmd);
		return -ENOENT;
	}

	ret = alloc_cmd_msg_data(&msg, data, size);
	if (ret < 0) {
		return ret;
	}

	if (tout != 0) {
		timeout = (tout == -1) ? K_FOREVER : K_MSEC(tout);

		ctx = k_malloc(sizeof(erpc_wifi_cmd_ctx_t));
		if (!ctx) {
			LOG_ERR("CMD ENOMEM: context allocation failed cmd=%d timeout=%d", cmd, tout);
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
				LOG_ERR("CMD timeout: cmd=%d timeout_ms=%d", cmd, tout);
				ret = -ETIMEDOUT;
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
		if (k_msgq_get(&cmd_msg_queue, &msg, K_FOREVER) != 0) {
			LOG_ERR("Failed to get message from queue");
			continue;
		}

		bool timed_out = false;
		if (msg.ctx && atomic_get(&msg.ctx->timed_out) != 0) {
			timed_out = true;
		}

		if (!timed_out) {
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
