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

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

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
	ps2_callback_t callback_isr;
};

static int ps2_gpio_configure(const struct device *dev,
			     ps2_callback_t callback_isr)
{
	const struct ps2_gpio_config *config = dev->config;
	struct ps2_gpio_data *data = dev->data;

	if (!callback_isr) {
		return -EINVAL;
	}

	data->callback_isr = callback_isr;

	// /* In case the self test for a PS2 device already finished and
	//  * set the SOURCE bit to 1 we clear it before enabling the
	//  * interrupts. Instances must be allocated before the BAT or
	//  * the host may time out.
	//  */
	// MCHP_GIRQ_SRC(config->girq_id) = BIT(config->girq_bit);
	// dummy = base->TRX_BUFF;
	// base->STATUS = MCHP_PS2_STATUS_RW1C_MASK;

	// /* Enable FSM and init instance in rx mode*/
	// base->CTRL = MCHP_PS2_CTRL_EN_POS;

	// /* We enable the interrupts in the EC aggregator so that the
	//  * result  can be forwarded to the ARM NVIC
	//  */
	// MCHP_GIRQ_ENSET(config->girq_id) = BIT(config->girq_bit);

	// k_sem_give(&data->tx_lock);

	return 0;
}

static void ps2_gpio_set_scl(const struct device *dev, int state)
{
	const struct ps2_gpio_config *config = dev->config;

	LOG_ERR("Not implemented: ps2_gpio_set_scl");

}

static void ps2_gpio_set_sda(void *io_context, int state)
{
	LOG_ERR("Not implemented: ps2_gpio_set_sda");

}


static int ps2_gpio_write(const struct device *dev, uint8_t value)
{
	const struct ps2_gpio_config *config = dev->config;
	struct ps2_gpio_data *data = dev->data;

	LOG_ERR("Not implemented: ps2_gpio_write");

	return 0;
}

static int ps2_gpio_inhibit_interface(const struct device *dev)
{
	const struct ps2_gpio_config *config = dev->config;
	struct ps2_gpio_data *data = dev->data;

	LOG_ERR("Not implemented: ps2_gpio_inhibit_interface");

	return 0;
}

static int ps2_gpio_enable_interface(const struct device *dev)
{
	const struct ps2_gpio_config *config = dev->config;
	struct ps2_gpio_data *data = dev->data;

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

static const struct ps2_gpio_config ps2_gpio_config = {
    .scl_gpio_name = DT_INST_GPIO_LABEL(0, scl_gpios),
    .scl_pin = DT_INST_GPIO_PIN(0, scl_gpios),
    .scl_flags = DT_INST_GPIO_FLAGS(0, scl_gpios),

    .sda_gpio_name = DT_INST_GPIO_LABEL(0, sda_gpios),
    .sda_pin = DT_INST_GPIO_PIN(0, sda_gpios),
    .sda_flags = DT_INST_GPIO_FLAGS(0, sda_gpios),
};

static struct ps2_gpio_data ps2_gpio_data = {
    .callback_isr = NULL
};

static int ps2_gpio_init(const struct device *dev)
{
	LOG_INF("Inside ps2_gpio_init");

	ARG_UNUSED(dev);

	struct ps2_gpio_data *data = dev->data;


	// IRQ_CONNECT(DT_INST_IRQN(0),
	// 	    DT_INST_IRQ(0, priority),
	// 	    ps2_gpio_isr, DEVICE_DT_INST_GET(0), 0);

	// irq_enable(DT_INST_IRQN(0));

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

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
