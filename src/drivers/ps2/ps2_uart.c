/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT uart_ps2

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_uart);

/*
 * Settings
 */

#define PS2_UART_WRITE_MAX_RETRY 5
#define PS2_UART_READ_MAX_RETRY 3

// Custom queue for background PS/2 processing work at low priority
// We purposefully want this to be a fairly low priority, because
// this queue is used while we wait to start a write.
// If the system is very busy with interrupts and other threads, then we
// want to wait until that is over so that our write interrupts don't get
// missed.
#define PS2_UART_WORK_QUEUE_PRIORITY 10
#define PS2_UART_WORK_QUEUE_STACK_SIZE 1024

// Custom queue for calling the zephyr ps/2 callback.
// We don't want to hand it off to that API in an ISR since that callback
// could be using blocking functions.
// But we also don't want to hand it off at a low priority, since the PS/2
// packets must be dealt with quickly. So we use a fairly high priority.
#define PS2_UART_WORK_QUEUE_CB_PRIORITY 2
#define PS2_UART_WORK_QUEUE_CB_STACK_SIZE 1024

/*
 * Driver Defines
 */

// Timeout for blocking read using the zephyr PS2 ps2_read() function
// This is not a matter of PS/2 timings, but a preference of how long we let
// the user wait until we give up on reading.
#define PS2_UART_TIMEOUT_READ K_SECONDS(2)

/*
 * Global Variables
 */

typedef enum
{
    PS2_UART_MODE_READ,
    PS2_UART_MODE_WRITE
} ps2_uart_mode;

struct ps2_uart_config {
};

struct ps2_uart_data {
	const struct device *dev;
	struct gpio_dt_spec scl_gpio;	/* GPIO used for PS2 SCL line */
	struct gpio_dt_spec sda_gpio;	/* GPIO used for PS2 SDA line */

	// PS2 driver interface callback
	struct k_work callback_work;
	uint8_t callback_byte;
	ps2_callback_t callback_isr;
	ps2_resend_callback_t resend_callback_isr;

	bool callback_enabled;

	// Queue for ps2_read()
	struct k_fifo data_queue;

	ps2_uart_mode mode;

	struct k_work resend_cmd_work;
	struct k_work test_spi_work;
};


static const struct ps2_uart_config ps2_uart_config = {

};

static struct ps2_uart_data ps2_uart_data = {
	.callback_byte = 0x0,
    .callback_isr = NULL,
    .resend_callback_isr = NULL,
	.callback_enabled = false,
	.mode = PS2_UART_MODE_READ,
};

K_THREAD_STACK_DEFINE(
	ps2_uart_work_queue_stack_area,
	PS2_UART_WORK_QUEUE_STACK_SIZE
);
static struct k_work_q ps2_uart_work_queue;

K_THREAD_STACK_DEFINE(
	ps2_uart_work_queue_cb_stack_area,
	PS2_UART_WORK_QUEUE_CB_STACK_SIZE
);
static struct k_work_q ps2_uart_work_queue_cb;

/*
 * Function Definitions
 */

int ps2_uart_write_byte(uint8_t byte);


/*
 * Helpers functions
 */

#define PS2_UART_GET_BIT(data, bit_pos) ( (data >> bit_pos) & 0x1 )
#define PS2_UART_SET_BIT(data, bit_val, bit_pos) ( \
	data |= (bit_val) << bit_pos \
)

void log_binary(uint8_t value) {
	char binary_str[9];

    for(int i = 7; i >= 0; --i) {
		binary_str[7 - i] = PS2_UART_GET_BIT(value, i) ? '1' : '0';
    }

    binary_str[8] = '\0';

	LOG_INF("Received Value: %s", binary_str);
}

bool ps2_uart_get_byte_parity(uint8_t byte)
{
	int byte_parity = __builtin_parity(byte);

	// gcc parity returns 1 if there is an odd number of bits in byte
	// But the PS2 protocol sets the parity bit to 0 if there is an odd number
	return !byte_parity;
}

uint8_t ps2_uart_data_queue_get_next(uint8_t *dst_byte, k_timeout_t timeout)
{
	struct ps2_uart_data *data = &ps2_uart_data;

	uint8_t *queue_byte = k_fifo_get(&data->data_queue, timeout);
	if(queue_byte == NULL) {
		return -ETIMEDOUT;
	}

	*dst_byte =  *queue_byte;

	k_free(queue_byte);

	return 0;
}

void ps2_uart_data_queue_empty()
{
	while(true) {
		uint8_t byte;
		int err = ps2_uart_data_queue_get_next(&byte, K_NO_WAIT);
		if(err) {  // No more items in queue
			break;
		}
	}
}

void ps2_uart_data_queue_add(uint8_t byte)
{
	struct ps2_uart_data *data = &ps2_uart_data;

	uint8_t *byte_heap = (uint8_t *) k_malloc(sizeof(byte));
	if(byte_heap == NULL) {
		LOG_WRN(
			"Could not allocate heap space to add byte to fifo. "
			"Clearing fifo."
		);

		// TODO: Define max amount for read data queue instead of emptying it
		// when memory runs out.
		// But unfortunately it seems like there is no official way to query
		// how many items are currently in the fifo.
		ps2_uart_data_queue_empty();

		byte_heap = (uint8_t *) k_malloc(sizeof(byte));
		if(byte_heap == NULL) {
			LOG_ERR(
				"Could not allocate heap space after clearing fifo. "
				"Losing received byte 0x%x", byte
			);
			return;
		}
	}

	*byte_heap = byte;
	k_fifo_alloc_put(&data->data_queue, byte_heap);
}

void ps2_uart_send_cmd_resend_worker(struct k_work *item)
{
	struct ps2_uart_data *data = &ps2_uart_data;

	// Notify the PS/2 device driver that we are requesting a resend.
	// PS/2 devices don't just resend the last byte that was sent, but the
	// entire command packet, which can be multiple bytes.
	if(data->resend_callback_isr != NULL && data->callback_enabled) {

		data->resend_callback_isr(data->dev);
	}

	uint8_t cmd = 0xfe;
	// LOG_DBG("Requesting resend of data with command: 0x%x", cmd);
	ps2_uart_write_byte(cmd);
}

void ps2_uart_send_cmd_resend()
{
	struct ps2_uart_data *data = &ps2_uart_data;

    if (k_is_in_isr()) {

		// It's important to submit this on the cb queue and not on the
		// same queue as the inhibition delay.
		// Otherwise the queue will be blocked by the semaphore and the
		// inhibition delay worker will never be called.
    	k_work_submit_to_queue(
			&ps2_uart_work_queue_cb,
			&data->resend_cmd_work
		);
    } else {
        ps2_uart_send_cmd_resend_worker(NULL);
    }
}

/*
 * Writing PS2 data
 */
K_MUTEX_DEFINE(write_mutex);

int ps2_uart_write_byte(uint8_t byte)
{
	int err;

	LOG_DBG("\n");
	LOG_DBG("START WRITE: 0x%x", byte);

	err = 0;

	k_mutex_lock(&write_mutex, K_FOREVER);

	LOG_DBG("END WRITE: 0x%x\n", byte);

	k_mutex_unlock(&write_mutex);

	return err;
}

/*
 * Zephyr PS/2 driver interface
 */
static int ps2_uart_enable_callback(const struct device *dev);

static int ps2_uart_configure(const struct device *dev,
			     ps2_callback_t callback_isr,
				 ps2_resend_callback_t resend_callback_isr)
{
	struct ps2_uart_data *data = dev->data;

	if (!callback_isr && !resend_callback_isr) {
		return -EINVAL;
	}

	if(callback_isr) {
		data->callback_isr = callback_isr;
		ps2_uart_enable_callback(dev);
	}

	if(resend_callback_isr) {
		data->resend_callback_isr = resend_callback_isr;
	}

	return 0;
}

int ps2_uart_read(const struct device *dev, uint8_t *value)
{
	// TODO: Add a way to not return old queue items
	// Maybe only bytes that were received within past 10 seconds.
	uint8_t queue_byte;
	int err = ps2_uart_data_queue_get_next(&queue_byte, PS2_UART_TIMEOUT_READ);
	if(err) {  // Timeout due to no data to read in data queue
		// LOG_DBG("ps2_uart_read: Fifo timed out...");

		return -ETIMEDOUT;
	}

	// LOG_DBG("ps2_uart_read: Returning 0x%x", queue_byte);
	*value =  queue_byte;

	return 0;
}

static int ps2_uart_write(const struct device *dev, uint8_t value)
{
	return ps2_uart_write_byte(value);
}

static int ps2_uart_disable_callback(const struct device *dev)
{
	struct ps2_uart_data *data = dev->data;

	// Make sure there are no stale items in the data queue
	// from before the callback was disabled.
	ps2_uart_data_queue_empty();

	data->callback_enabled = false;

	// LOG_DBG("Disabled PS2 callback.");

	return 0;
}

static int ps2_uart_enable_callback(const struct device *dev)
{
	struct ps2_uart_data *data = dev->data;
	data->callback_enabled = true;

	// LOG_DBG("Enabled PS2 callback.");

	ps2_uart_data_queue_empty();

	return 0;
}

static const struct ps2_driver_api ps2_uart_driver_api = {
	.config = ps2_uart_configure,
	.read = ps2_uart_read,
	.write = ps2_uart_write,
	.disable_callback = ps2_uart_disable_callback,
	.enable_callback = ps2_uart_enable_callback,
};

/*
 * PS/2 GPIO Driver Init
 */

void ps2_uart_test_spi(struct k_work *item)
{
	const struct ps2_uart_data *data = &ps2_uart_data;
	const struct ps2_uart_config *config = &ps2_uart_config;
	int err;

}

static int ps2_uart_init(const struct device *dev)
{

	struct ps2_uart_data *data = dev->data;
	// const struct ps2_uart_config *config = dev->config;
	// Set the ps2 device so we can retrieve it later for
	// the ps2 callback
	data->dev = dev;

	const struct ps2_uart_config *config = dev->config;

	LOG_INF("Inside ps2_uart_init");


	// Init fifo for synchronous read operations
	k_fifo_init(&data->data_queue);

	// Custom queue for background PS/2 processing work at high priority
	k_work_queue_start(
        &ps2_uart_work_queue,
        ps2_uart_work_queue_stack_area,
        K_THREAD_STACK_SIZEOF(ps2_uart_work_queue_stack_area),
        PS2_UART_WORK_QUEUE_PRIORITY,
        NULL
    );

	// Custom queue for calling the zephyr ps/2 callback at lower priority
	k_work_queue_start(
        &ps2_uart_work_queue_cb,
        ps2_uart_work_queue_cb_stack_area,
        K_THREAD_STACK_SIZEOF(ps2_uart_work_queue_cb_stack_area),
        PS2_UART_WORK_QUEUE_CB_PRIORITY,
        NULL
    );

	k_work_init(&data->test_spi_work, ps2_uart_test_spi);



	// k_work_submit_to_queue(
	// 	&ps2_uart_work_queue_cb,
	// 	&data->test_spi_work
	// );

	return 0;
}

DEVICE_DT_INST_DEFINE(
	0,
	&ps2_uart_init,
	NULL,
	&ps2_uart_data, &ps2_uart_config,
	POST_KERNEL, 80,
	&ps2_uart_driver_api
);
