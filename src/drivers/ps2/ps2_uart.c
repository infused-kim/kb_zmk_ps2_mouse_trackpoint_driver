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
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_uarte.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_uart);

/*
 * Pin Control
 */

PINCTRL_DT_DEFINE(DT_INST_BUS(0));

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
 * PS/2 Defines
 */

#define PS2_UART_POS_START 0
#define PS2_UART_POS_DATA_FIRST 1
#define PS2_UART_POS_DATA_LAST 8
#define PS2_UART_POS_PARITY 9
#define PS2_UART_POS_STOP 10
#define PS2_UART_POS_ACK 11  // Write mode only

#define PS2_UART_TIMING_SCL_CYCLE_LEN 69


// The minimum time needed to inhibit clock to start a write
// is 100us, but we triple it just in case.
#define PS2_UART_TIMING_SCL_INHIBITION_MIN 100


// Theoretically, only 100us is required, but practically, trackponts
// seem to respond to a total duration of 1,000 us the best.
// This is also the duration my USB to PS/2 adapter is using.
#define PS2_UART_TIMING_SCL_INHIBITION ( \
	5 * PS2_UART_TIMING_SCL_INHIBITION_MIN \
)


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

struct ps2_uart_config {
    const struct device *uart_dev;
	struct gpio_dt_spec scl_gpio;
	struct gpio_dt_spec sda_gpio;
	const struct pinctrl_dev_config *pcfg;
};

struct ps2_uart_data {
	const struct device *dev;

	// SCL GPIO callback for writing
	struct gpio_callback scl_cb_data;

	// PS2 driver interface callback
	struct k_work callback_work;
	uint8_t callback_byte;
	ps2_callback_t callback_isr;
	ps2_resend_callback_t resend_callback_isr;

	bool callback_enabled;

	// Queue for ps2_read()
	struct k_fifo data_queue;

	// Write byte
	uint8_t cur_write_byte;

	struct k_work resend_cmd_work;
};

static const struct ps2_uart_config ps2_uart_config = {
	.uart_dev = DEVICE_DT_GET(DT_INST_BUS(0)),
	.scl_gpio	= GPIO_DT_SPEC_INST_GET(0, scl_gpios),
	.sda_gpio	= GPIO_DT_SPEC_INST_GET(0, sda_gpios),
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_BUS(0)),
};

static struct ps2_uart_data ps2_uart_data = {
	.callback_byte = 0x0,
    .callback_isr = NULL,
    .resend_callback_isr = NULL,
	.callback_enabled = false,
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

int ps2_uart_get_scl()
{
	const struct ps2_uart_config *config = &ps2_uart_config;
	int rc = gpio_pin_get_dt(&config->scl_gpio);

	return rc;
}

int ps2_uart_get_sda()
{
	const struct ps2_uart_config *config = &ps2_uart_config;
	int rc = gpio_pin_get_dt(&config->sda_gpio);

	return rc;
}

void ps2_uart_set_scl(int state)
{
	const struct ps2_uart_config *config = &ps2_uart_config;

	gpio_pin_set_dt(&config->scl_gpio, state);
}

void ps2_uart_set_sda(int state)
{
	const struct ps2_uart_config *config = &ps2_uart_config;

	// LOG_INF("Seting sda to %d", state);
	gpio_pin_set_dt(&config->sda_gpio, state);
}

int ps2_uart_configure_pin_scl(gpio_flags_t flags, char *descr)
{
	struct ps2_uart_config *config = (struct ps2_uart_config *) &ps2_uart_config;
	int err;

	err = gpio_pin_configure_dt(
		&config->scl_gpio,
		flags
	);
	if (err) {
		LOG_ERR("failed to configure SCL GPIO pin to %s (err %d)", descr, err);
	}

	return err;
}

int ps2_uart_configure_pin_scl_input()
{
	return ps2_uart_configure_pin_scl(
		(GPIO_INPUT),
		"input"
	);
}

int ps2_uart_configure_pin_scl_output()
{
	return ps2_uart_configure_pin_scl(
		(GPIO_OUTPUT_HIGH),
		"output"
	);
}

int ps2_uart_configure_pin_sda(gpio_flags_t flags, char *descr)
{
	struct ps2_uart_config *config = (struct ps2_uart_config *) &ps2_uart_config;
	int err;

	err = gpio_pin_configure_dt(
		&config->sda_gpio,
		flags
	);
	if (err) {
		LOG_ERR("failed to configure SDA GPIO pin to %s (err %d)", descr, err);
	}

	return err;
}

int ps2_uart_configure_pin_sda_input()
{
	return ps2_uart_configure_pin_sda(
		(GPIO_INPUT),
		"input"
	);
}

int ps2_uart_configure_pin_sda_output()
{
	return ps2_uart_configure_pin_sda(
		(GPIO_OUTPUT_HIGH),
		"output"
	);
}

int ps2_uart_set_scl_callback_enabled(bool enabled)
{
	struct ps2_uart_config *config = (struct ps2_uart_config *) &ps2_uart_config;
	int err;

	// LOG_INF("Setting ps2_uart_set_scl_callback_enabled: %d", enabled);

	if(enabled) {
		err = gpio_pin_interrupt_configure_dt(
			&config->scl_gpio,
			(GPIO_INT_EDGE_FALLING)
		);
		if (err) {
			LOG_ERR(
				"failed to enable interrupt on "
				"SCL GPIO pin (err %d)", err
			);
			return err;
		}
	} else {
		err = gpio_pin_interrupt_configure_dt(
			&config->scl_gpio,
			(GPIO_INT_DISABLE)
		);
		if (err) {
			LOG_ERR(
				"failed to disable interrupt on "
				"SCL GPIO pin (err %d)", err
			);
			return err;
		}
	}

	return err;
}

static int ps2_uart_set_mode_read()
{
	const struct ps2_uart_config *config = &ps2_uart_config;
	int err;

	// Set the SDA pin for the uart device
	err = pinctrl_apply_state(
		config->pcfg,
		PINCTRL_STATE_DEFAULT
	);
	if (err < 0) {
		LOG_ERR("Could not switch pinctrl state to DEFAULT: %d", err);
		return err;
	}

	// Make sure SCL interrupt is disabled
	ps2_uart_set_scl_callback_enabled(false);

	// Enable UART interrupt
	uart_irq_rx_enable(config->uart_dev);

	return err;
}

static int ps2_uart_set_mode_write()
{
	const struct ps2_uart_config *config = &ps2_uart_config;
	int err;

	// Disable UART interrupt
	uart_irq_rx_disable(config->uart_dev);

	// Set pincntrl with unused pins so that we can control the pins
	// through GPIO
	err = pinctrl_apply_state(
		config->pcfg,
		PINCTRL_STATE_SLEEP
	);
	if (err < 0) {
		LOG_ERR("Could not switch pinctrl state to OFF: %d", err);
		return err;
	}

	// Configure data and clock lines for output
	ps2_uart_configure_pin_scl_output();
	ps2_uart_configure_pin_sda_output();

	return err;
}

void log_binary(uint8_t value) {
	char binary_str[9];

    for(int i = 7; i >= 0; --i) {
		binary_str[7 - i] = PS2_UART_GET_BIT(value, i) ? '1' : '0';
    }

    binary_str[8] = '\0';

	LOG_INF("Binary Value of 0x%x: %s", value, binary_str);
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
 * Reading PS2 data
 */
static void ps2_uart_interrupt_handler(const struct device *uart_dev,
									   void *user_data);
void ps2_uart_read_interrupt_handler();
static int ps2_uart_read_err_check(const struct device *dev);
void ps2_uart_read_process_received_byte(uint8_t byte);
const char *ps2_uart_read_get_error_str(int err);

static void ps2_uart_interrupt_handler(const struct device *uart_dev,
									   void *user_data)
{
	int err;

	err = uart_irq_update(uart_dev);
	if(err != 1) {
		LOG_ERR("uart_irq_update returned: %d", err);
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {
		ps2_uart_read_interrupt_handler(uart_dev, user_data);
    }
}

void ps2_uart_read_interrupt_handler(const struct device *uart_dev,
									 void *user_data)
{
	int err;
	uint8_t byte;

	int byte_len = uart_fifo_read(uart_dev, &byte, 1);
	if(byte_len < 1) {
		LOG_ERR("UART read failed with error: %d", byte_len);
		return;
	}

	err = ps2_uart_read_err_check(uart_dev);
	if(err != 0) {
		const char *err_str = ps2_uart_read_get_error_str(err);
		LOG_ERR(
			"UART RX detected error for byte 0x%x: %s (%d)",
			byte, err_str, err
		);
	}

	ps2_uart_read_process_received_byte(byte);
}

static int ps2_uart_read_err_check(const struct device *dev)
{
	// TOOD: Make this function only work if nrf52 is used
	int err = uart_err_check(dev);

	// In the config we enabled even parity, because nrf52 does
	// not support odd parity.
	// But PS/2 uses odd parity. This should generate a parity error on
	// every reception.
	// If the parity error doesn't happen, it means the transfer had an
	// actual even parity, which we consider an error.
	if((err & NRF_UARTE_ERROR_PARITY_MASK) == 0) {
		err =  UART_ERROR_PARITY;
	} else if(err & NRF_UARTE_ERROR_OVERRUN_MASK) {
		err = UART_ERROR_OVERRUN;
	} else if(err & NRF_UARTE_ERROR_FRAMING_MASK) {
		err = UART_ERROR_FRAMING;
	} else if(err & NRF_UARTE_ERROR_BREAK_MASK) {
		err = UART_BREAK;
	} else { // No errors
		err = 0;
	}

	return err;
}

void ps2_uart_read_process_received_byte(uint8_t byte)
{
	struct ps2_uart_data *data = &ps2_uart_data;

	LOG_DBG("UART Received: 0x%x", byte);

	// If no callback is set, we add the data to a fifo queue
	// that can be read later with the read using `ps2_read`
	if(data->callback_isr != NULL && data->callback_enabled) {

		// Call callback from a worker to make sure the callback
		// doesn't block the interrupt.
		// Will call ps2_uart_read_callback_work_handler
		data->callback_byte = byte;
    	k_work_submit_to_queue(
			&ps2_uart_work_queue_cb,
			&data->callback_work
		);
	} else {
		ps2_uart_data_queue_add(byte);
	}
}

const char *ps2_uart_read_get_error_str(int err)
{
	switch (err)
	{
	case UART_ERROR_OVERRUN:
		return "Overrun error";
	case UART_ERROR_PARITY:
		return "Parity error";
	case UART_ERROR_FRAMING:
		return "Framing error";
	case UART_BREAK:
		return "Break interrupt";
	case UART_ERROR_COLLISION:
		return "Collision error";
	default:
		return "Unknown error";
	}
}

void ps2_uart_read_callback_work_handler(struct k_work *work)
{
	struct ps2_uart_data *data = &ps2_uart_data;

	data->callback_isr(data->dev, data->callback_byte);
	data->callback_byte = 0x0;
}


/*
 * Writing PS2 data
 */
K_MUTEX_DEFINE(write_mutex);

int ps2_uart_write_byte(uint8_t byte)
{
	int err;
	struct ps2_uart_data *data = &ps2_uart_data;

	LOG_INF("\n");
	LOG_INF("START WRITE: 0x%x", byte);
	log_binary(byte);

	k_mutex_lock(&write_mutex, K_FOREVER);

	err = ps2_uart_set_mode_write();
	if(err != 0) {
		LOG_ERR("Could not configure driver for write mode: %d", err);
		return err;
	}

	// Set the write byte so it can be used in
	// the downstream write function that is called
	// from the SCL interrupt
	data->cur_write_byte = byte;

	// Inhibit the line by setting clock low and data high for 100us
	ps2_uart_set_scl(0);
	k_busy_wait(PS2_UART_TIMING_SCL_INHIBITION);

	// Set data to value of start bit
	ps2_uart_set_sda(0);
	k_busy_wait(PS2_UART_TIMING_SCL_INHIBITION);

	// Release the clock line and configure it as input
	// This let's the device take control of the clock again
	ps2_uart_set_scl(1);
	ps2_uart_configure_pin_scl_input();

	// We need to wait for the first SCL clock
	// Execution continues once it arrives in
	// `ps2_uart_write_scl_interrupt_handler`
	ps2_uart_set_scl_callback_enabled(true);

	k_mutex_unlock(&write_mutex);

	return 0;
}

// The nrf52 is too slow to process all SCL interrupts, so we
// try to avoid them as much as possible.
//
// But, after we initiate the write transmission with SCL and SDA LOW,
// the PS/2 device doesn't always respond right away. It can take as
// much as 5,000us for it to start sending the clock for the
// transmission.
//
// Once it does start sending the clock the cycles are pretty
// consistently between 67 and 70us (at least on the trackpoints I
// tested).
//
// So, we use a GPIO interrupt to wait for the first clock cycle and
// then use delays to send the actual data at the same rate as the
// UART baud rate.
void ps2_uart_write_scl_interrupt_handler(const struct device *dev,
						   				  struct gpio_callback *cb,
						   				  uint32_t pins)
{
	struct ps2_uart_data *data = &ps2_uart_data;
	int err;

	// Disable the SCL interrupt again.
	// From here we will just use time delays.
	ps2_uart_set_scl_callback_enabled(false);

	for(int i = PS2_UART_POS_DATA_FIRST; i <= PS2_UART_POS_STOP; i++) {

		if(i >= PS2_UART_POS_DATA_FIRST && i <= PS2_UART_POS_DATA_LAST) {

			int data_pos = i - PS2_UART_POS_DATA_FIRST;
			bool data_bit = PS2_UART_GET_BIT(
				data->cur_write_byte,
				data_pos
			);

			ps2_uart_set_sda(data_bit);
		} else if(i == PS2_UART_POS_PARITY) {

			bool byte_parity = ps2_uart_get_byte_parity(data->cur_write_byte);

			ps2_uart_set_sda(byte_parity);
		} else if(i == PS2_UART_POS_STOP) {

			ps2_uart_set_sda(1);

			// Give control over data pin back to device after sending
			// the stop bit so that we can receive the ack bit from the
			// device
			ps2_uart_configure_pin_sda_input();
		} else {
			LOG_ERR("UART unknown TX bit number: %d", i);
		}

		// Sleep for the cycle length
		k_busy_wait(PS2_UART_TIMING_SCL_CYCLE_LEN);
	}

	// Check Ack
	int ack_val = ps2_uart_get_sda();
	int ret = -1;

	if(ack_val == 0) {
		LOG_INF("Write was successful");
		ret = 0;
	} else {
		LOG_ERR("Write failed on ack");
		ret = -1;
	}

	err = ps2_uart_set_mode_read();
	if(err != 0) {
		LOG_ERR("Could not configure driver for read mode: %d", err);
		return;
	}

	LOG_DBG("END WRITE: 0x%x\n", data->cur_write_byte);
	data->cur_write_byte = 0x0;

	k_mutex_unlock(&write_mutex);
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
	int ret = ps2_uart_write_byte(value);

	return ret;
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
 * PS/2 UART Driver Init
 */
static int ps2_uart_init_uart(void);
static int ps2_uart_init_gpio(void);

static int ps2_uart_init(const struct device *dev)
{
	int err;
	struct ps2_uart_data *data = dev->data;

	// Set the ps2 device so we can retrieve it later for
	// the ps2 callback
	data->dev = dev;

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

	k_work_init(&data->callback_work, ps2_uart_read_callback_work_handler);

	err = ps2_uart_init_uart();
	if(err != 0) {
		LOG_ERR("Could not init UART: %d", err);
		return err;
	}

	err = ps2_uart_init_gpio();
	if(err != 0) {
		LOG_ERR("Could not init GPIO: %d", err);
		return err;
	}

	err = ps2_uart_set_mode_read();
	if(err != 0) {
		LOG_ERR("Could not initialize in UART mode read: %d", err);
		return err;
	}

	return 0;
}

static int ps2_uart_init_uart(void)
{
	struct ps2_uart_data *data = &ps2_uart_data;
	struct ps2_uart_config *config = (struct ps2_uart_config *) &ps2_uart_config;
	int err;

    if (!device_is_ready(config->uart_dev))
    {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    } else {
		LOG_INF("UART device is ready");
	}

	struct uart_config uart_cfg;
	err = uart_config_get(config->uart_dev, &uart_cfg);
	if(err != 0) {
		LOG_ERR("Could not retrieve UART config...");
		return -ENODEV;
	}

	uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
	uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
	uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	// PS/2 uses odd parity, but nrf52840 doesn't support
	// odd parity. Despite that, setting none works.
	uart_cfg.parity = UART_CFG_PARITY_EVEN;

	err = uart_configure(config->uart_dev, &uart_cfg);
	if(err != 0) {
		LOG_ERR("Could not configure UART device: %d", err);
		return -EINVAL;
	}

	uart_irq_callback_user_data_set(
		config->uart_dev,
		ps2_uart_interrupt_handler,
		(void *)data->dev
	);

	uart_irq_rx_enable(config->uart_dev);
	uart_irq_err_enable(config->uart_dev);

	return 0;
}

static int ps2_uart_init_gpio(void)
{
	struct ps2_uart_data *data = &ps2_uart_data;
	struct ps2_uart_config *config = (struct ps2_uart_config *) &ps2_uart_config;
	int err;

	// Interrupt for clock line
	gpio_init_callback(
		&data->scl_cb_data,
		ps2_uart_write_scl_interrupt_handler,
		BIT(config->scl_gpio.pin)
	);

	err = gpio_add_callback(config->scl_gpio.port, &data->scl_cb_data);
	if (err) {
		LOG_ERR(
			"failed to enable interrupt callback on "
			"SCL GPIO pin (err %d)", err
		);
	}

	ps2_uart_set_scl_callback_enabled(false);

	return err;
}

DEVICE_DT_INST_DEFINE(
	0,
	&ps2_uart_init,
	NULL,
	&ps2_uart_data, &ps2_uart_config,
	POST_KERNEL, 80,
	&ps2_uart_driver_api
);
