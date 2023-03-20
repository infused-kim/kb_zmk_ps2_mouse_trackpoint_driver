/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gpio_ps2

#include <errno.h>
#include <device.h>
#include <drivers/ps2.h>
#include <drivers/gpio.h>

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#include <logging/log.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_gpio);

#define PS2_GPIO_TIMEOUT_READ K_SECONDS(2)
#define PS2_GPIO_POS_START 0
#define PS2_GPIO_POS_PARITY 9
#define PS2_GPIO_POS_STOP 10
#define PS2_GPIO_POS_ACK 11  // Write mode only

#define PS2_GPIO_GET_BIT(data, bit_pos) ( (data >> bit_pos) & 0x1 )
#define PS2_GPIO_SET_BIT(data, bit_val, bit_pos) ( data |= (bit_val) << bit_pos )

int ps2_gpio_write_byte(uint8_t byte);

typedef enum
{
    PS2_GPIO_MODE_READ,
    PS2_GPIO_MODE_WRITE
} ps2_gpio_mode;

// Used to keep track of blocking write status
typedef enum
{
    PS2_GPIO_WRITE_STATUS_INACTIVE,
    PS2_GPIO_WRITE_STATUS_ACTIVE,
	PS2_GPIO_WRITE_STATUS_SUCCESS,
	PS2_GPIO_WRITE_STATUS_FAILURE,
} ps2_gpio_write_status;

struct ps2_gpio_config {
	const char *scl_gpio_name;
	gpio_pin_t scl_pin;
	gpio_dt_flags_t scl_flags;

	const char *sda_gpio_name;
	gpio_pin_t sda_pin;
	gpio_dt_flags_t sda_flags;
};

struct ps2_gpio_data {
	const struct device *scl_gpio;	/* GPIO used for PS2 SCL line */
	const struct device *sda_gpio;	/* GPIO used for PS2 SDA line */

	struct gpio_callback scl_cb_data;

	ps2_callback_t callback_isr;
	bool callback_enabled;
	struct k_fifo data_queue;

	ps2_gpio_mode mode;

	uint8_t cur_read_byte;
	int cur_read_pos;

	uint16_t write_buffer;
	int cur_write_pos;
	ps2_gpio_write_status cur_write_status;
	struct k_sem write_lock;
};


static const struct ps2_gpio_config ps2_gpio_config = {
    .scl_gpio_name = DT_INST_GPIO_LABEL(0, scl_gpios),
    .scl_pin = DT_INST_GPIO_PIN(0, scl_gpios),
    .scl_flags = DT_INST_GPIO_FLAGS(0, scl_gpios),

    .sda_gpio_name = DT_INST_GPIO_LABEL(0, sda_gpios),
    .sda_pin = DT_INST_GPIO_PIN(0, sda_gpios),
    .sda_flags = DT_INST_GPIO_FLAGS(0, sda_gpios),
};

static struct ps2_gpio_data ps2_gpio_data = {
	.scl_gpio = NULL,
	.sda_gpio = NULL,

    .callback_isr = NULL,
	.callback_enabled = false,
	.mode = PS2_GPIO_MODE_READ,

	.cur_read_byte = 0x0,
	.cur_read_pos = 0,

	.write_buffer = 0x0,
	.cur_write_pos = 0,
	.cur_write_status = PS2_GPIO_WRITE_STATUS_INACTIVE,
};


/*
 * Helpers functions
 */

int ps2_gpio_get_scl()
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;
	const struct ps2_gpio_config *config = &ps2_gpio_config;
	int rc = gpio_pin_get(data->scl_gpio, config->scl_pin);

	return rc;
}

int ps2_gpio_get_sda()
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;
	const struct ps2_gpio_config *config = &ps2_gpio_config;
	int rc = gpio_pin_get(data->sda_gpio, config->sda_pin);

	return rc;
}

void ps2_gpio_set_scl(int state)
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;
	const struct ps2_gpio_config *config = &ps2_gpio_config;

	LOG_INF("Seting scl to %d", state);
	gpio_pin_set(data->sda_gpio, config->sda_pin, state);
}

void ps2_gpio_set_sda(int state)
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;
	const struct ps2_gpio_config *config = &ps2_gpio_config;

	LOG_INF("Seting sda to %d", state);
	gpio_pin_set(data->scl_gpio, config->scl_pin, state);
}

void ps2_gpio_send_cmd_resend()
{
	uint8_t cmd = 0xfe;
	ps2_gpio_write_byte(cmd);
}

void ps2_gpio_empty_data_queue()
{
	struct ps2_gpio_data *data = &ps2_gpio_data;
	while(k_fifo_get(&data->data_queue, K_NO_WAIT) != NULL) {
		// Do nothing except call k_fifo_get() until it's empty.
	}
}

bool get_bit(uint8_t byte, int bit_pos)
{
    return (byte >> bit_pos) & 0x1;
}

void set_bit(uint8_t *byte, bool bit_val, int bit_pos)
{
    *byte |= bit_val << bit_pos;
}

/*
 * Reading PS/2 data
 */

void ps2_gpio_process_received_byte(uint8_t byte)
{
	LOG_INF("Successfully received value: 0x%x", byte);

	struct ps2_gpio_data *data = &ps2_gpio_data;
	if(data->callback_isr != NULL && data->callback_enabled) {

		data->callback_isr(NULL, byte);
	} else {

		// If no callback is set, we add the data to a fifo queue
		// that can be read later with the read using `ps2_read`
		k_fifo_put(&data->data_queue, &byte);
	}
}

void ps2_gpio_abort_read()
{
	struct ps2_gpio_data *data = &ps2_gpio_data;

	ps2_gpio_send_cmd_resend();
	data->cur_read_pos = 0;
	data->cur_read_byte = 0x0;
}

bool ps2_gpio_check_parity(uint8_t byte, int parity_bit_val)
{
	int byte_parity = __builtin_parity(byte);

	// gcc parity returns 1 if there is an odd number of bits in byte
	// But the PS2 protocol sets the parity bit to 0 if there is an odd number
	if(byte_parity == parity_bit_val) {
		return 0;  // Do not match
	}

	return 1;  // Match
}

void ps2_gpio_scl_interrupt_falling_read_bit()
{
	struct ps2_gpio_data *data = &ps2_gpio_data;
	int scl_val = ps2_gpio_get_scl();
	int sda_val = ps2_gpio_get_sda();

	LOG_INF(
		"ps2_gpio_scl_interrupt_falling_read_bit called with position=%d; scl=%d; sda=%d",
		data->cur_read_pos, scl_val, sda_val
	);

	if(data->cur_read_pos == PS2_GPIO_POS_START) {
		// The first bit of every transmission should be 0.
		// If it is not, it means we are out of sync with the device.
		// So we abort the transmission and start from scratch.
		if(sda_val != 0) {
			ps2_gpio_abort_read();
			LOG_ERR("Restarting receiving due to invalid start bit.");
			return;
		}
	} else if(data->cur_read_pos == PS2_GPIO_POS_PARITY) {
		if(ps2_gpio_check_parity(data->cur_read_byte, sda_val) != true) {
			ps2_gpio_abort_read();
			LOG_ERR("Restarting receiving due to invalid parity bit.");
			return;
		}
	} else if(data->cur_read_pos == PS2_GPIO_POS_STOP) {
		if(sda_val != 1) {
			ps2_gpio_abort_read();
			LOG_ERR("Restarting receiving due to invalid stop bit.");
			return;
		}

		ps2_gpio_process_received_byte(data->cur_read_byte);
		data->cur_read_pos = 0;
		data->cur_read_byte = 0x0;

		return;
	} else { // Data Bits

		// Current position, minus start bit
		int bit_pos = data->cur_read_pos - 1;
		PS2_GPIO_SET_BIT(data->cur_read_byte, sda_val, bit_pos);
	}

 	data->cur_read_pos += 1;
}


/*
 * Writing PS2 data
 */

bool ps2_gpio_get_byte_parity(uint8_t byte)
{
	int byte_parity = __builtin_parity(byte);

	// gcc parity returns 1 if there is an odd number of bits in byte
	// But the PS2 protocol sets the parity bit to 0 if there is an odd number
	return !byte_parity;
}

int ps2_gpio_write_byte_blocking(uint8_t byte)
{
	struct ps2_gpio_data *data = &ps2_gpio_data;
	int err;

	LOG_INF("ps2_gpio_write_byte_blocking called with byte=0x%x",byte);

	// The async `write_byte` function takes the only available semaphor.
	// This causes the `k_sem_take` call below to block until
	// `ps2_gpio_scl_interrupt_rising_check_write_ack` gives it back.
	err = ps2_gpio_write_byte(byte);
    if (err) {
		LOG_ERR("Could not initiate writing of byte.");
		return err;
	}

	LOG_INF("Trying to take semaphore in ps2_gpio_write_byte_blocking...");
	err = k_sem_take(&data->write_lock, K_MSEC(500));
    if (err) {
		LOG_ERR("Write timed out...");
		return err;
	}

	if(data->cur_write_status == PS2_GPIO_WRITE_STATUS_SUCCESS) {
		LOG_INF("Blocking write finished successfully");
		err = 0;
	} else {
		LOG_INF(
			"Blocking write finished with failure status: %d",
			data->cur_write_status
		);
		err = -data->cur_write_status;
	}

	data->cur_write_status = PS2_GPIO_WRITE_STATUS_INACTIVE;

	return err;
}

int ps2_gpio_write_byte(uint8_t byte) {
	struct ps2_gpio_data *data = &ps2_gpio_data;
	struct ps2_gpio_config *config = &ps2_gpio_config;

	int err;

	LOG_INF("ps2_gpio_write_byte called with byte=0x%x", byte);

	// Take semaphore so that when `ps2_gpio_write_byte_blocking` attempts
	// taking it, the process gets blocked.
	// It is released in `ps2_gpio_scl_interrupt_rising_check_write_ack`.
	LOG_INF("Taking semaphore in ps2_gpio_write_byte");
	err = k_sem_take(&data->write_lock, K_NO_WAIT);
    if (err != 0 && err != -EBUSY) {
		LOG_ERR("ps2_gpio_write_byte could not take semaphore: %d", err);

		return err;
	}

	bool byte_parity = ps2_gpio_get_byte_parity(byte);

	// Reset write buffer
	data->write_buffer = 0x0;

	// Set bits for the entire transmission
	PS2_GPIO_SET_BIT(data->write_buffer, 0, 0);  				// Start Bit
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 0), 1);  // Data Bit 0
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 1), 2);  // Data Bit 1
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 2), 3);  // Data Bit 2
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 3), 4);  // Data Bit 3
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 4), 5);  // Data Bit 4
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 5), 6);  // Data Bit 5
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 6), 7);  // Data Bit 6
	PS2_GPIO_SET_BIT(data->write_buffer, PS2_GPIO_GET_BIT(byte, 7), 8);  // Data Bit 7
	PS2_GPIO_SET_BIT(data->write_buffer, byte_parity, 9);  		// Parity Bit
	PS2_GPIO_SET_BIT(data->write_buffer, 1, 10);  				// Stop Bit

	// Change mode and set write_pos so that falling interrupt doesn't trigger
	data->mode = PS2_GPIO_MODE_WRITE;
	data->cur_write_pos = 0;

	err = gpio_pin_configure(
		data->scl_gpio,
		config->scl_pin,
		(GPIO_OUTPUT)
	);
	if (err) {
		LOG_ERR("failed to configure SCL GPIO pin to output (err %d)", err);
		return err;
	}

	err = gpio_pin_configure(
		data->sda_gpio,
		config->sda_pin,
		(GPIO_OUTPUT)
	);
	if (err) {
		LOG_ERR("failed to configure SDA GPIO pin to output (err %d)", err);
		return err;
	}

	// Initiate host->device transmission by bringing clock down
	// for at least 100 microseconds
	ps2_gpio_set_scl(0);
	k_sleep(K_USEC(110));

	// This aborts any in-progress reads, so we
	// reset the current read byte
	data->cur_write_status = PS2_GPIO_WRITE_STATUS_ACTIVE;
	data->cur_read_byte = 0x0;
	data->cur_read_pos = 0;

	// Send the start bit
	ps2_gpio_set_sda(PS2_GPIO_GET_BIT(data->write_buffer, 0));
	data->cur_write_pos += 1;

	err = gpio_pin_configure(
		data->scl_gpio,
		config->scl_pin,
		(GPIO_INPUT)
	);
	if (err) {
		LOG_ERR("failed to configure SCL GPIO pin to input (err %d)", err);
		return err;
	}

	// Release the clock line
	ps2_gpio_set_scl(1);

	// From here on the device takes over the control of the clock again
	// Every time it is ready for the next bit to be trasmitted, it will...
	//  - Pull the clock line low
	//  - Which will trigger our `ps2_gpio_scl_interrupt_falling_handler`
	//  - Which will call `ps2_gpio_scl_interrupt_falling_write_bit`
	//  - Which will send the correct bit
	//  - After all bits are sent `interrupt_rising_check_write_ack` is called
	//  - Which verifies if the transaction was successful

	return 0;
}

void ps2_gpio_scl_interrupt_falling_write_bit()
{
	// This function is called by `ps2_gpio_scl_interrupt_falling_handler`
	// when the device pulls the clock line low after we initiated a
	// write.
	//
	// We just continue to send all the bits
	struct ps2_gpio_data *data = &ps2_gpio_data;
	struct ps2_gpio_config *config = &ps2_gpio_config;

	if(data->cur_write_pos == PS2_GPIO_POS_START)
	{
		// PS2_GPIO_POS_START is sent in ps2_gpio_write_byte
		LOG_INF(
			"ps2_gpio_scl_interrupt_falling_write_bit: Ignoring pos=%d",
			data->cur_write_pos
		);
		return;
	}

	int data_bit = PS2_GPIO_GET_BIT(data->write_buffer, data->cur_write_pos);

	LOG_INF("ps2_gpio_scl_interrupt_falling_write_bit called with pos=%d; bit=%d", data->cur_write_pos, data_bit);
	ps2_gpio_set_sda(data_bit);

	data->cur_write_pos += 1;

	if(data->cur_write_pos == PS2_GPIO_POS_ACK) {
		int err;
		err = gpio_pin_configure(
			data->sda_gpio,
			config->sda_pin,
			(GPIO_INPUT)
		);
		if (err) {
			LOG_ERR("failed to configure SDA GPIO pin to input (err %d)", err);
			return err;
		}
	}
}

void ps2_gpio_scl_interrupt_rising_check_write_ack()
{
	// This function is called by `ps2_gpio_scl_interrupt_rising_handler`
	// when the device pulls the clock line high after we send the stop bit
	// during a write.
	struct ps2_gpio_data *data = &ps2_gpio_data;

	int ack_val = ps2_gpio_get_sda();
	LOG_INF("ps2_gpio_scl_interrupt_rising_check_write_ack ack_val: %d", ack_val);

	if(ack_val == 0) {
		LOG_INF("Write was successful");
		data->cur_write_status = PS2_GPIO_WRITE_STATUS_SUCCESS;
	} else {
		LOG_INF("Write failed with ack: %d", ack_val);
		data->cur_write_status = PS2_GPIO_WRITE_STATUS_FAILURE;
	}

	// Reset write buffer and position
	data->mode = PS2_GPIO_MODE_READ;
	data->write_buffer = 0x0;
	data->cur_write_pos = 0;

	LOG_INF("Giving back semaphore in ps2_gpio_write_byte");
	k_sem_give(&data->write_lock);
}
/*
 * Interrupt Handlers
 */

void ps2_gpio_scl_interrupt_falling_handler(const struct device *dev,
						   struct gpio_callback *cb,
						   uint32_t pins)
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;

	// LOG_INF("ps2_gpio_scl_interrupt_falling_handler called with mode=%d",data->mode);

	if(data->mode == PS2_GPIO_MODE_READ) {
		ps2_gpio_scl_interrupt_falling_read_bit();
	} else {
		ps2_gpio_scl_interrupt_falling_write_bit();
	}
}

void ps2_gpio_scl_interrupt_rising_handler(const struct device *dev,
						   				   struct gpio_callback *cb,
						   				   uint32_t pins)
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;
	// LOG_INF("ps2_gpio_scl_interrupt_rising_handler called with cur_write_pos=%d, mode=%d", data->cur_write_pos, data->mode);

	if(data->mode == PS2_GPIO_MODE_WRITE &&
	   data->cur_write_pos == PS2_GPIO_POS_ACK)
	{
		ps2_gpio_scl_interrupt_rising_check_write_ack();
	}
}

void ps2_gpio_scl_interrupt_handler(const struct device *dev,
						   			struct gpio_callback *cb,
						   			uint32_t pins)
{
	// const struct ps2_gpio_data *data = &ps2_gpio_data;
	// LOG_INF("ps2_gpio_scl_interrupt_handler called with scl_val=%d, mode=%d", scl_val, data->mode);

	int scl_val = ps2_gpio_get_scl();
	if(scl_val == 0) {
		ps2_gpio_scl_interrupt_falling_handler(dev, cb, pins);
	} else {
		ps2_gpio_scl_interrupt_rising_handler(dev, cb, pins);
	}
}


/*
 * Zephyr PS/2 driver interface
 */
static int ps2_gpio_enable_callback(const struct device *dev);

static int ps2_gpio_configure(const struct device *dev,
			     ps2_callback_t callback_isr)
{
	LOG_ERR("In ps2_gpio_configure");
	struct ps2_gpio_data *data = dev->data;

	if (!callback_isr) {
		return -EINVAL;
	}

	data->callback_isr = callback_isr;
	ps2_gpio_enable_callback(dev);

	return 0;
}

int ps2_gpio_read(const struct device *dev, uint8_t *value)
{
	// TODO: Add a way to not return old queue items
	// Maybe only bytes that were received within past 10 seconds.
	LOG_INF("In ps2_gpio_read...");

	struct ps2_gpio_data *data = dev->data;

	uint8_t *queue_byte;
	queue_byte = k_fifo_get(&data->data_queue, PS2_GPIO_TIMEOUT_READ);
	if(queue_byte == NULL) {
		LOG_ERR("ps2_gpio_read: Fifo timed out...");

		return -ETIMEDOUT;
	}

	LOG_DBG("ps2_gpio_read: Returning 0x%x", *queue_byte);
	*value =  *queue_byte;

	return 0;
}

static int ps2_gpio_write(const struct device *dev, uint8_t value)
{
	return ps2_gpio_write_byte_blocking(value);
}

static int ps2_gpio_disable_callback(const struct device *dev)
{
	struct ps2_gpio_data *data = dev->data;

	// Make sure there are no stale items in the data queue
	// from before the callback was disabled.
	ps2_gpio_empty_data_queue();

	data->callback_enabled = false;

	LOG_INF("Disabled PS2 callback.");

	return 0;
}

static int ps2_gpio_enable_callback(const struct device *dev)
{
	struct ps2_gpio_data *data = dev->data;
	data->callback_enabled = true;

	LOG_INF("Enabled PS2 callback.");

	ps2_gpio_empty_data_queue();

	return 0;
}

static const struct ps2_driver_api ps2_gpio_driver_api = {
	.config = ps2_gpio_configure,
	.read = ps2_gpio_read,
	.write = ps2_gpio_write,
	.disable_callback = ps2_gpio_disable_callback,
	.enable_callback = ps2_gpio_enable_callback,
};

/*
 * PS/2 GPIO Driver Init
 */

int ps2_gpio_configure_scl_pin(struct ps2_gpio_data *data,
							   const struct ps2_gpio_config *config)
{
	int err;

	// Configure PIN
	data->scl_gpio = device_get_binding(config->scl_gpio_name);
	if (!data->scl_gpio) {
		LOG_ERR("failed to get SCL GPIO device");
		return -EINVAL;
	}

	err = gpio_pin_configure(
		data->scl_gpio,
		config->scl_pin,
		(GPIO_INPUT)
	);
	if (err) {
		LOG_ERR("failed to configure SCL GPIO pin (err %d)", err);
		return err;
	}

	// Interrupt for clock line
	// Almost all actions happen on the falling edge, but at the end of a write
	// the device sends an ack bit on the rising edge. Setting up both edges
	// allows us to detect it.
	err = gpio_pin_interrupt_configure(
		data->scl_gpio,
		config->scl_pin,
		(GPIO_INT_EDGE_BOTH)
	);
	if (err) {
		LOG_ERR(
			"failed to configure interrupt on "
			"SCL GPIO pin (err %d)", err
		);
		return err;
	}

	gpio_init_callback(
		&data->scl_cb_data,
		ps2_gpio_scl_interrupt_handler,
		BIT(config->scl_pin)
	);
	err = gpio_add_callback(data->scl_gpio, &data->scl_cb_data);
	if (err) {
		LOG_ERR(
			"failed to configure interrupt callback on "
			"SCL GPIO pin (err %d)", err
		);
		return err;
	}

	return 0;
}

int ps2_gpio_configure_sda_pin(struct ps2_gpio_data *data,
							   const struct ps2_gpio_config *config)
{
	int err;

	data->sda_gpio = device_get_binding(config->sda_gpio_name);
	if (!data->sda_gpio) {
		LOG_ERR("failed to get SDA GPIO device");
		return -EINVAL;
	}

	err = gpio_pin_configure(
		data->sda_gpio,
		config->sda_pin,
		(GPIO_INPUT)
	);
	if (err) {
		LOG_ERR("failed to configure SDA GPIO pin (err %d)", err);
		return err;
	}

	return 0;
}

static int ps2_gpio_init(const struct device *dev)
{
	LOG_INF("Inside ps2_gpio_init");

	struct ps2_gpio_data *data = dev->data;
	const struct ps2_gpio_config *config = dev->config;
	int err;

	err = ps2_gpio_configure_scl_pin(data, config);
	if (err) {
		return err;
	}
	err = ps2_gpio_configure_sda_pin(data, config);
	if (err) {
		return err;
	}

	// Check if this stuff is needed
	// TODO: Figure out why this is requiered.
	ps2_gpio_set_sda(1);
	ps2_gpio_set_scl(1);

	LOG_INF("Finished configuring ps2_gpio.");

	// Init fifo for synchronous read operations
	k_fifo_init(&data->data_queue);

	// Init semaphore for blocking writes
	LOG_INF("Initializing semaphore.");
	k_sem_init(&data->write_lock, 0, 1);

	return 0;
}

DEVICE_DT_INST_DEFINE(
	0,
	&ps2_gpio_init,
	NULL,
	&ps2_gpio_data, &ps2_gpio_config,
	POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,
	&ps2_gpio_driver_api
);

// #endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
