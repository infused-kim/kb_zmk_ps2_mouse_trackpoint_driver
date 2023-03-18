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

#define PS2_GPIO_POS_START 0
#define PS2_GPIO_POS_PARITY 9
#define PS2_GPIO_POS_STOP 10

typedef enum
{
    PS2_GPIO_MODE_READ,
    PS2_GPIO_MODE_WRITE
} ps2_gpio_mode;

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

	ps2_gpio_mode mode;

	int cur_read_byte : 8;
	int cur_read_pos;

	int write_buffer: 11;
	int cur_write_pos;
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
	.mode = PS2_GPIO_MODE_READ,

	.cur_read_byte = 0x0,
	.cur_read_pos = 0,

	.write_buffer = 0x0,
	.cur_write_pos = 0,
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

	gpio_pin_set(data->sda_gpio, config->sda_pin, state);
}

void ps2_gpio_set_sda(int state)
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;
	const struct ps2_gpio_config *config = &ps2_gpio_config;

	gpio_pin_set(data->scl_gpio, config->scl_pin, state);
}

int ps2_gpio_send_byte(uint8_t byte)
{
	LOG_ERR("Not implemented ps2_gpio_send_byte");

	return -1;
}

int ps2_gpio_send_cmd_resend()
{
	uint8_t cmd = 0xfe;
	return ps2_gpio_send_byte(cmd);
}

int ps2_gpio_abort_read()
{
	struct ps2_gpio_data *data = &ps2_gpio_data;

	int err = ps2_gpio_send_cmd_resend();
	data->cur_read_pos = 0;

	return err;
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

void ps2_gpio_process_received_byte(uint8_t byte)
{

	LOG_INF("Successfully received value: 0x%x", byte);
}

/*
 * Interrupt Handler
 */

void scl_interrupt_handler_mode_read()
{
	struct ps2_gpio_data *data = &ps2_gpio_data;
	int scl_val = ps2_gpio_get_scl();
	int sda_val = ps2_gpio_get_sda();

	LOG_INF(
		"scl_interrupt_handler called with position=%d; scl=%d; sda=%d",
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
		if(sda_val != 0) {
			ps2_gpio_abort_read();
			LOG_ERR("Restarting receiving due to invalid stop bit.");
			return;
		}

		data->cur_read_pos = 0;
		ps2_gpio_process_received_byte(data->cur_read_byte);

		return;
	} else { // Data Bits

		// Current position, minus start bit
		int byte_bit_pos = data->cur_read_pos - 1;
		data->cur_read_byte |= sda_val << byte_bit_pos;
	}

 	data->cur_read_pos += 1;
}

void scl_interrupt_handler_mode_write()
{
	LOG_ERR("Not implemented scl_interrupt_handler_mode_write");
}

void scl_interrupt_handler(const struct device *dev,
						   struct gpio_callback *cb,
						   uint32_t pins)
{
	const struct ps2_gpio_data *data = &ps2_gpio_data;

	if(data->mode == PS2_GPIO_MODE_READ) {
		scl_interrupt_handler_mode_read();
	} else {
		scl_interrupt_handler_mode_write();
	}
}

/*
 * Zephyr PS/2 driver interface
 */

static int ps2_gpio_configure(const struct device *dev,
			     ps2_callback_t callback_isr)
{
	LOG_ERR("In ps2_gpio_configure");

	// const struct ps2_gpio_config *config = dev->config;
	struct ps2_gpio_data *data = dev->data;

	if (!callback_isr) {
		return -EINVAL;
	}

	data->callback_isr = callback_isr;

	return 0;
}

static int ps2_gpio_write(const struct device *dev, uint8_t value)
{
	LOG_ERR("Not implemented: ps2_gpio_write");

	return 0;
}

static int ps2_gpio_inhibit_interface(const struct device *dev)
{
	LOG_ERR("Not implemented: ps2_gpio_inhibit_interface");

	return 0;
}

static int ps2_gpio_enable_interface(const struct device *dev)
{
	LOG_ERR("Not implemented: ps2_gpio_enable_interface");

	return 0;
}

static const struct ps2_driver_api ps2_gpio_driver_api = {
	.config = ps2_gpio_configure,
	.read = NULL,
	.write = ps2_gpio_write,
	.disable_callback = ps2_gpio_inhibit_interface,
	.enable_callback = ps2_gpio_enable_interface,
};

/*
 * PS/2 GPIO Driver Init
 */

static int ps2_gpio_init(const struct device *dev)
{
	LOG_INF("Inside ps2_gpio_init");

	struct ps2_gpio_data *data = dev->data;
	const struct ps2_gpio_config *config = dev->config;
	int err;

	data->scl_gpio = device_get_binding(config->scl_gpio_name);
	if (!data->scl_gpio) {
		LOG_ERR("failed to get SCL GPIO device");
		return -EINVAL;
	}

	err = gpio_pin_configure(
		data->scl_gpio,
		config->scl_pin,
		(GPIO_ACTIVE_HIGH | GPIO_INPUT | GPIO_OUTPUT_HIGH)
	);
	if (err) {
		LOG_ERR("failed to configure SCL GPIO pin (err %d)", err);
		return err;
	}

	err = gpio_pin_interrupt_configure(
		data->scl_gpio,
		config->scl_pin,
		GPIO_INT_EDGE_TO_INACTIVE
	);
	if (err) {
		LOG_ERR("failed to configure interrupt on SCL GPIO pin (err %d)", err);
		return err;
	}

	gpio_init_callback(
		&data->scl_cb_data,
		scl_interrupt_handler,
		BIT(config->scl_pin)
	);
	err = gpio_add_callback(data->scl_gpio, &data->scl_cb_data);
	if (err) {
		LOG_ERR(
			"failed to configure interrupt callbacko on SCL GPIO pin "
			"(err %d)", err
		);
		return err;
	}

	data->sda_gpio = device_get_binding(config->sda_gpio_name);
	if (!data->sda_gpio) {
		LOG_ERR("failed to get SDA GPIO device");
		return -EINVAL;
	}

	err = gpio_pin_configure(
		data->sda_gpio,
		config->sda_pin,
		(GPIO_ACTIVE_HIGH | GPIO_INPUT | GPIO_OUTPUT_HIGH)
	);
	if (err) {
		LOG_ERR("failed to configure SDA GPIO pin (err %d)", err);
		return err;
	}

	LOG_INF("Finished configuring ps2_gpio.");

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
