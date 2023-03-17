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

/* in 50us units */
#define PS2_TIMEOUT 10000

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
};


/*
 * Helpers for reading and setting scl and sda pins.
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


/*
 * Interrupt Handler
 */
void scl_interrupt_handler(const struct device *dev,
						   struct gpio_callback *cb,
						   uint32_t pins)
{
	int scl_val = ps2_gpio_get_scl();
	int sda_val = ps2_gpio_get_sda();
	LOG_INF(
		"scl_interrupt_handler called with scl=%d; sda=%d", scl_val, sda_val
	);
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
