#include "erpc_wifi_cmd.h"

#include "erpc_wifi.h"
#include <c_wifi_host_to_ra_client.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(erpc_wifi_cmd, CONFIG_WIFI_LOG_LEVEL);

#define ERPC_WIFI_MSG_MAX 12
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
		LOG_ERR("Failed to allocate cmd: %d msg data", msg->cmd);
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
	int cmd_ret = 0;
	k_timeout_t timeout = K_NO_WAIT;
	erpc_wifi_msg_data_t msg = { .cmd = cmd, .sem = NULL, .cmd_ret = &cmd_ret };

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

		if ((msg.sem = k_malloc(sizeof(*msg.sem))) == NULL) {
			free_cmd_msg_data(&msg);
			return -ENOMEM;
		}

		k_sem_init(msg.sem, 0, 1);
	}

	if (k_msgq_put(&cmd_msg_queue, &msg, K_NO_WAIT) == 0) {

		if (msg.sem && (ret = k_sem_take(msg.sem, timeout)) != 0) {
			ret = -ETIMEDOUT;
		} else {
			if (msg.cmd_ret) {
				ret = *msg.cmd_ret;
			}
		}

		if (msg.sem) {
			k_free(msg.sem);
		}
	} else {
		free_cmd_msg_data(&msg);
		LOG_ERR("Failed to enqueue command: %d (queue full)", cmd);
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

		/* Dispatch to registered handler */
		if (msg.cmd < ERPC_WIFI_LAST_CMD && erpc_wifi_socket_handlers[msg.cmd].h) {
			int ret = erpc_wifi_socket_handlers[msg.cmd].h(msg.data);
			
			/* Store result for caller */
			if (msg.cmd_ret) {
				*msg.cmd_ret = ret;
			}

		} else {
			LOG_ERR("No handler registered for command %d", msg.cmd);
			if (msg.cmd_ret) {
				*msg.cmd_ret = -ENOENT;
			}
		}

		/* Free allocated message data */
		free_cmd_msg_data(&msg);

		/* Signal completion if semaphore present */
		if (msg.sem) {
			k_sem_give(msg.sem);
		}

		/* After removing the sleep constraint the PS state machine immediately
		 * transitions to ASLEEP. The SPI completion leaves SRDY briefly high as
		 * an artefact; this delay lets it deassert before the poll task runs,
		 * preventing a spurious notify_wakeup() that would re-hold the module awake. */
		if (msg.cmd == ERPC_WIFI_PMGR_REMOVE_SLEEP_CONSTRAINT_CMD) {
			k_msleep(500);
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
