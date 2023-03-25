/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_ps2_mouse

#include <device.h>
#include <devicetree.h>
#include <drivers/ps2.h>
#include <sys/util.h>

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_PS2_LOG_LEVEL);

#define PS2_MOUSE_THREAD_STACK_SIZE 1024
#define PS2_MOUSE_THREAD_PRIORITY 10

struct zmk_ps2_mouse_config {
	const struct device *ps2_device;
};

struct zmk_ps2_mouse_data {
    K_THREAD_STACK_MEMBER(thread_stack, PS2_MOUSE_THREAD_STACK_SIZE);
    struct k_thread thread;
};


static const struct zmk_ps2_mouse_config zmk_ps2_mouse_config = {
    .ps2_device = DEVICE_DT_GET(DT_INST_PHANDLE(0, ps2_device))
};

static struct zmk_ps2_mouse_data zmk_ps2_mouse_data;


/*
 * Helpers functions
 */

/*
 * PS/2 Commands
 */

int zmk_ps2_stream_mode_enable(const struct device *ps2_device) {
    int err;
    int cmd = 0xea;
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not send stream mode enable command (0x%x): %d", cmd, err
        );
        return err;
    }

    // uint8_t cmd_res;
    // err = ps2_read(ps2_device, &cmd_res);
    // if(err) {
    //     LOG_ERR("Could not read stream mode enable reporting result: %d", err);
    //     return err;
    // }

    // if(cmd_res == 0xfa) {
    //     LOG_ERR(
    //         "Successfully enabled stream mode reporting: %d", cmd_res
    //     );

    //     return 0;
    // } else {
    //     LOG_ERR(
    //         "Could not enable stream mode enable reporting with result: 0x%x",
    //         cmd_res
    //     );

    //     return -1;
    // }

    return 0;
}

int zmk_ps2_stream_mode_enable_reporting(const struct device *ps2_device) {
    int err;
    err = ps2_write(ps2_device, 0xf4);
    if(err) {
        LOG_ERR(
            "Could not send stream mode enable reporting command: %d", err
        );
        return err;
    }

    // uint8_t cmd_res;
    // err = ps2_read(ps2_device, &cmd_res);
    // if(err) {
    //     LOG_ERR("Could not read stream mode enable reporting result: %d", err);
    //     return err;
    // }

    // if(cmd_res == 0xfa) {
    //     LOG_ERR(
    //         "Successfully enabled stream mode reporting: %d", cmd_res
    //     );

    //     return 0;
    // } else {
    //     LOG_ERR(
    //         "Could not enable stream mode enable reporting with result: 0x%x",
    //         cmd_res
    //     );

    //     return -1;
    // }

    return 0;
}

int zmk_ps2_reset(const struct device *ps2_device) {
    int err;

    uint8_t cmd = 0xff;
    LOG_INF("Sendin reset command: 0x%x", cmd);
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not reset: %d", err
        );
        return err;
    } else {
        LOG_INF("Sent command succesfully: 0x%x", cmd);
    }

    // uint8_t cmd_res;
    // err = ps2_read(ps2_device, &cmd_res);
    // if(err) {
    //     LOG_ERR("Could not read reset result: %d", err);
    //     return err;
    // }

    // if(cmd_res == 0xfa) {
    //     LOG_ERR(
    //         "Successfully reset: %d", cmd_res
    //     );

    //     return 0;
    // } else {
    //     LOG_ERR(
    //         "Could not reset with result: 0x%x",
    //         cmd_res
    //     );

    //     return -1;
    // }
}

/*
 * Mouse Movement Callback
 */

void zmk_ps2_mouse_read_callback(const struct device *ps2_device,
                                 uint8_t data)
{
    LOG_INF("Received mouse movement data: 0x%x", data);
}

/*
 * Init
 */

static void zmk_ps2_mouse_init_thread(int dev_ptr, int unused) {
    const struct device *dev = INT_TO_POINTER(dev_ptr);
	const struct zmk_ps2_mouse_config *config = dev->config;
    int err;

	LOG_INF("Inside zmk_ps2_mouse_init_thread");

    // Read self test result
    uint8_t read_val;

    while(true) {
	    LOG_INF("Reading PS/2 self-test...");
        err = ps2_read(config->ps2_device, &read_val);
        if(err) {
            LOG_ERR(
                "Could not read PS/2 device self-test result: %d. ", err
            );
            k_sleep(K_SECONDS(5));
        } else {
            LOG_INF("Got PS/2 device self-test result: 0x%x", read_val);
            break;
        }
    }
	// LOG_INF("Reading PS/2 self-test...");
    // err = ps2_read(config->ps2_device, &read_val);
    // if(err) {
    //     LOG_ERR("Could not read PS/2 device self-test result: %d", err);
    //     LOG_ERR("Sending reset command");
    //     zmk_ps2_reset(config->ps2_device);
    // } else {
    //     LOG_INF("Got PS/2 device self-test result: 0x%x", read_val);
    // }

    // Read device id
	LOG_INF("Reading PS/2 device id...");
    err = ps2_read(config->ps2_device, &read_val);
    if(err) {
        LOG_ERR("Could not read PS/2 device id: %d", err);
    } else {
        LOG_INF("Got PS/2 device id: 0x%x", read_val);
    }

    // zmk_ps2_reset(config->ps2_device);

	LOG_INF("Enabling stream mode reporting...");
    zmk_ps2_stream_mode_enable(config->ps2_device);

    k_sleep(K_SECONDS(1));

    // Enable stream mode reporting
	LOG_INF("Enabling stream mode reporting...");
    zmk_ps2_stream_mode_enable_reporting(config->ps2_device);

    // // Enable read callback
	// LOG_INF("Configuring ps2 callback...");
    // err = ps2_config(config->ps2_device, &zmk_ps2_mouse_read_callback);
    // if(err) {
    //     LOG_ERR("Could not configure ps2 interface: %d", err);
    //     return ;
    // }

	// LOG_INF("Enabling ps2 callback...");
    // err = ps2_enable_callback(config->ps2_device);
    // if(err) {
    //     LOG_ERR("Could not activate ps2 callback: %d", err);
    // } else {
    //     LOG_INF("Successfully activated ps2 callback");
    // }

	return;
}

static int zmk_ps2_mouse_init(const struct device *dev)
{
	LOG_INF("Inside zmk_ps2_mouse_init");

	LOG_INF("Creating ps2_mouse init thread.");
    k_thread_create(
        &zmk_ps2_mouse_data.thread,
        zmk_ps2_mouse_data.thread_stack,
        PS2_MOUSE_THREAD_STACK_SIZE,
        (k_thread_entry_t)zmk_ps2_mouse_init_thread,
        (struct device *)dev, 0, NULL,
        K_PRIO_COOP(PS2_MOUSE_THREAD_PRIORITY), 0, K_NO_WAIT
    );

	return 0;
}

DEVICE_DT_INST_DEFINE(
	0,
	&zmk_ps2_mouse_init,
	NULL,
	&zmk_ps2_mouse_data, &zmk_ps2_mouse_config,
	POST_KERNEL, 41,
	NULL
);

// #endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
