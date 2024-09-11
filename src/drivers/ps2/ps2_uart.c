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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_uarte.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_uart);

/*
 * Pin Control
 */

// PINCTRL_DT_DEFINE(DT_INST_BUS(0)); // moved to PS2_UART_DEFINE(n)

/*
 * Settings
 */

#define PS2_UART_WRITE_MAX_RETRY 5
#define PS2_UART_READ_MAX_RETRY 3

#define PS2_UART_DATA_QUEUE_SIZE 100

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
#define PS2_UART_POS_ACK 11 // Write mode only

#define PS2_UART_RESP_ACK 0xfa
#define PS2_UART_RESP_RESEND 0xfe
#define PS2_UART_RESP_FAILURE 0xfc

/*
 * PS/2 Timings
 */

#define PS2_UART_TIMING_SCL_CYCLE_LEN 69

// The minimum time needed to inhibit clock to start a write
// is 100us, but we triple it just in case.
#define PS2_UART_TIMING_SCL_INHIBITION_MIN 100

// Theoretically, only 100us is required, but practically, trackponts
// seem to respond to a total duration of 1,000 us the best.
// This is also the duration my USB to PS/2 adapter is using.
#define PS2_UART_TIMING_SCL_INHIBITION (5 * PS2_UART_TIMING_SCL_INHIBITION_MIN)

// PS2 uses a frequency between 10 kHz and 16.7 kHz. So clocks should arrive
// within 60-100us.
#define PS2_UART_TIMING_SCL_CYCLE_MIN 60
#define PS2_UART_TIMING_SCL_CYCLE_MAX 100

// After inhibiting and releasing the clock, the device starts sending
// the clock. It's supposed to start immediately, but some devices
// need much longer if you are asking them to interrupt an
// ongoing read.
#define PS2_UART_TIMING_SCL_INHIBITION_RESP_MAX 3000
#define PS2_UART_TIMEOUT_WRITE_SCL_START K_USEC(PS2_UART_TIMING_SCL_INHIBITION_RESP_MAX)

// Max time we allow the device to send the next clock signal during writes.
// Even though PS/2 devices send the clock at most every 100us, it doesn't mean
// that the interrupts always get triggered within that time. So we allow a
// little extra time.
#define PS2_UART_TIMEOUT_WRITE_SCL K_USEC(PS2_UART_TIMING_SCL_CYCLE_MAX + 50)

// Writes start with us inhibiting the line and then respond
// with 11 bits (start bit included in inhibition time).
// To be conservative we give it another 2 cycles to complete
#define PS2_UART_TIMING_WRITE_MAX_TIME                                                             \
    (PS2_UART_TIMING_SCL_INHIBITION + PS2_UART_TIMING_SCL_INHIBITION_RESP_MAX +                    \
     11 * PS2_UART_TIMING_SCL_CYCLE_MAX + 2 * PS2_UART_TIMING_SCL_CYCLE_MAX)

// Reads are 11bit and we give it another 2 cycles to start and stop
#define PS2_UART_TIMING_READ_MAX_TIME                                                              \
    (11 * PS2_UART_TIMING_SCL_CYCLE_MAX + 2 * PS2_UART_TIMING_SCL_CYCLE_MAX)

// Timeout for write_byte_await_response()
// PS/2 spec says that device must respond within 20msec,
// but real life devices take much longer. Especially if
// you interrupt existing transmissions.
#define PS2_UART_TIMEOUT_WRITE_AWAIT_RESPONSE K_MSEC(300)

/*
 * Driver Defines
 */

// Timeout for blocking read using the zephyr PS2 ps2_read() function
// This is not a matter of PS/2 timings, but a preference of how long we let
// the user wait until we give up on reading.
#define PS2_UART_TIMEOUT_READ K_SECONDS(2)

// Timeout for write_byte_blocking()
#define PS2_UART_TIMEOUT_WRITE_BLOCKING K_USEC(PS2_UART_TIMING_WRITE_MAX_TIME)

/*
 * Global Variables
 */

// Used to keep track of blocking write status
typedef enum {
    PS2_UART_WRITE_STATUS_INACTIVE,
    PS2_UART_WRITE_STATUS_ACTIVE,
    PS2_UART_WRITE_STATUS_SUCCESS,
    PS2_UART_WRITE_STATUS_FAILURE,
} ps2_uart_write_status;

struct ps2_uart_data_queue_item {
    uint8_t byte;
};

struct ps2_uart_config {
    int ps2_uart_idx;

    const struct device *uart_dev;
    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;
    const struct pinctrl_dev_config *pcfg;

    int scl_gpio_port_num;
    int sda_gpio_port_num;
};

struct ps2_uart_data {
    const struct device *dev;

    // SCL GPIO callback for writing
    struct gpio_callback scl_cb_data;

    // PS2 driver interface callback
    struct k_work callback_work;
    uint8_t callback_byte;
    ps2_callback_t callback_isr;
#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)
    ps2_resend_callback_t resend_callback_isr;
#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

    bool callback_enabled;

    // Queue for ps2_read()
    struct k_msgq data_queue;
    char data_queue_buffer[PS2_UART_DATA_QUEUE_SIZE * sizeof(struct ps2_uart_data_queue_item)];

    // Write byte
    ps2_uart_write_status cur_write_status;
    uint8_t cur_write_byte;
    int cur_write_pos;
    bool write_awaits_resp;
    uint8_t write_awaits_resp_byte;
    struct k_sem write_awaits_resp_sem;
    struct k_sem write_lock;
    struct k_work_delayable write_scl_timout;

    struct k_work resend_cmd_work;

    void *scl_rupt_async;
    void *scl_rupt_blocking;
};

K_THREAD_STACK_DEFINE(ps2_uart_work_queue_stack_area, PS2_UART_WORK_QUEUE_STACK_SIZE);
static struct k_work_q ps2_uart_work_queue;

K_THREAD_STACK_DEFINE(ps2_uart_work_queue_cb_stack_area, PS2_UART_WORK_QUEUE_CB_STACK_SIZE);
static struct k_work_q ps2_uart_work_queue_cb;

/*
 * Function Definitions
 */

int ps2_uart_write_byte(const struct device *dev, uint8_t byte);

/*
 * Helpers functions
 */

#define PS2_UART_GET_BIT(data, bit_pos) ((data >> bit_pos) & 0x1)

int ps2_uart_get_scl(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int rc = gpio_pin_get_dt(&config->scl_gpio);

    return rc;
}

int ps2_uart_get_sda(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int rc = gpio_pin_get_dt(&config->sda_gpio);

    return rc;
}

void ps2_uart_set_scl(const struct device *dev, int state) {
    const struct ps2_uart_config *config = dev->config;

    gpio_pin_set_dt(&config->scl_gpio, state);
}

void ps2_uart_set_sda(const struct device *dev, int state) {
    const struct ps2_uart_config *config = dev->config;

    // LOG_INF("Seting sda to %d", state);
    gpio_pin_set_dt(&config->sda_gpio, state);
}

int ps2_uart_configure_pin_scl(const struct device *dev, gpio_flags_t flags, char *descr) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    err = gpio_pin_configure_dt(&config->scl_gpio, flags);
    if (err) {
        LOG_ERR("failed to configure SCL GPIO pin to %s (err %d)", descr, err);
    }

    return err;
}

int ps2_uart_configure_pin_scl_input(const struct device *dev) {
    return ps2_uart_configure_pin_scl(dev, (GPIO_INPUT), "input");
}

int ps2_uart_configure_pin_scl_output(const struct device *dev) {
    return ps2_uart_configure_pin_scl(dev, (GPIO_OUTPUT_HIGH), "output");
}

int ps2_uart_configure_pin_sda(const struct device *dev, gpio_flags_t flags, char *descr) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    err = gpio_pin_configure_dt(&config->sda_gpio, flags);
    if (err) {
        LOG_ERR("failed to configure SDA GPIO pin to %s (err %d)", descr, err);
    }

    return err;
}

int ps2_uart_configure_pin_sda_input(const struct device *dev) {
    return ps2_uart_configure_pin_sda(dev, (GPIO_INPUT), "input");
}

int ps2_uart_configure_pin_sda_output(const struct device *dev) {
    return ps2_uart_configure_pin_sda(dev, (GPIO_OUTPUT_HIGH), "output");
}

int ps2_uart_set_scl_callback_enabled(const struct device *dev, bool enabled) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    // LOG_INF("Setting ps2_uart_set_scl_callback_enabled: %d", enabled);

    if (enabled) {
        err = gpio_pin_interrupt_configure_dt(&config->scl_gpio, (GPIO_INT_EDGE_FALLING));
        if (err) {
            LOG_ERR("failed to enable interrupt on "
                    "SCL GPIO pin (err %d)",
                    err);
            return err;
        }
    } else {
        err = gpio_pin_interrupt_configure_dt(&config->scl_gpio, (GPIO_INT_DISABLE));
        if (err) {
            LOG_ERR("failed to disable interrupt on "
                    "SCL GPIO pin (err %d)",
                    err);
            return err;
        }
    }

    return err;
}

static int ps2_uart_set_mode_read(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    // Set the SDA pin for the uart device
    err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0) {
        LOG_ERR("Could not switch pinctrl state to DEFAULT: %d", err);
        return err;
    }

    // Make sure SCL interrupt is disabled
    ps2_uart_set_scl_callback_enabled(dev, false);

    // Enable UART interrupt
    uart_irq_rx_enable(config->uart_dev);

    return err;
}

static int ps2_uart_set_mode_write(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    // Set pincntrl with unused pins so that we can control the pins
    // through GPIO
    err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
    if (err < 0) {
        LOG_ERR("Could not switch pinctrl state to OFF: %d", err);
        return err;
    }

    // Disable UART interrupt
    // Unintuitively, this has to be done AFTER applying the pincntrl state,
    // otherwise GPIO won't be able to use the data pin
    uart_irq_rx_disable(config->uart_dev);

    // Configure data and clock lines for output
    ps2_uart_set_scl_callback_enabled(dev, false);
    ps2_uart_configure_pin_scl_output(dev);
    ps2_uart_configure_pin_sda_output(dev);

    return err;
}

void log_binary(uint8_t value) {
    char binary_str[9];

    for (int i = 7; i >= 0; --i) {
        binary_str[7 - i] = PS2_UART_GET_BIT(value, i) ? '1' : '0';
    }

    binary_str[8] = '\0';

    LOG_INF("Binary Value of 0x%x: %s", value, binary_str);
}

bool ps2_uart_get_byte_parity(uint8_t byte) {
    int byte_parity = __builtin_parity(byte);

    // gcc parity returns 1 if there is an odd number of bits in byte
    // But the PS2 protocol sets the parity bit to 0 if there is an odd number
    return !byte_parity;
}

uint8_t ps2_uart_data_queue_get_next(const struct device *dev, uint8_t *dst_byte, k_timeout_t timeout) {
    struct ps2_uart_data *data = dev->data;
    struct ps2_uart_data_queue_item queue_data;
    int ret;

    ret = k_msgq_get(&data->data_queue, &queue_data, timeout);
    if (ret != 0) {
        LOG_WRN("Data queue timed out...");
        return -ETIMEDOUT;
    }

    *dst_byte = queue_data.byte;

    return 0;
}

void ps2_uart_data_queue_empty(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;

    k_msgq_purge(&data->data_queue);
}

void ps2_uart_data_queue_add(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;

    int ret;

    struct ps2_uart_data_queue_item queue_data;
    queue_data.byte = byte;

    LOG_DBG("Adding byte to data queue: 0x%x", byte);

    for (int i = 0; i < 2; i++) {
        ret = k_msgq_put(&data->data_queue, &queue_data, K_NO_WAIT);
        if (ret == 0) {
            break;
        } else {
            LOG_WRN("Data queue full. Removing oldest item.");

            uint8_t tmp_byte;
            ps2_uart_data_queue_get_next(dev, &tmp_byte, K_NO_WAIT);
        }
    }

    if (ret != 0) {
        LOG_ERR("Failed to add byte 0x%x to the data queue.", byte);
    }
}

void ps2_uart_send_cmd_resend_worker(struct k_work *item) {

    // struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct ps2_uart_data *data = CONTAINER_OF(item,
                                              struct ps2_uart_data,
                                              resend_cmd_work);
    const struct device *dev = data->dev;

#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)

    // Notify the PS/2 device driver that we are requesting a resend.
    // PS/2 devices don't just resend the last byte that was sent, but the
    // entire command packet, which can be multiple bytes.
    if (data->resend_callback_isr != NULL && data->callback_enabled) {

        data->resend_callback_isr(data->dev);
    }

#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

    uint8_t cmd = 0xfe;
    // LOG_DBG("Requesting resend of data with command: 0x%x", cmd);
    ps2_uart_write_byte(dev, cmd);
}

void ps2_uart_send_cmd_resend(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;

    if (k_is_in_isr()) {

        // It's important to submit this on the cb queue and not on the
        // same queue as the inhibition delay.
        // Otherwise the queue will be blocked by the semaphore and the
        // inhibition delay worker will never be called.
        k_work_submit_to_queue(&ps2_uart_work_queue_cb, &data->resend_cmd_work);
    } else {
        // ps2_uart_send_cmd_resend_worker(NULL);
        k_work_submit_to_queue(&ps2_uart_work_queue_cb, &data->resend_cmd_work);
    }
}

// Extract port and pin from pinctrl to log it
// Based on the defines in zephyr/dt-bindings/pinctrl/nrf-pinctrl.h:
// https://docs.zephyrproject.org/apidoc/latest/nrf-pinctrl_8h_source.html
#define PINCTRL_PSEL_NRF_EXTRACT_PORT(x) (((x) >> 5) & 0x03)
#define PINCTRL_PSEL_NRF_EXTRACT_PIN(x) ((x) & 0x1F)
int ps2_uart_get_pinctrl_port_pin(const struct pinctrl_dev_config *pcfg, uint8_t state_id,
                                  uint8_t pin_id, int *res_port, int *res_pin) {

#if IS_ENABLED(CONFIG_SOC_SERIES_NRF52X)

    if (state_id >= pcfg->state_cnt) {
        LOG_ERR("Could not retrieve pinctrl pin config, because there is no state_id %d ",
                state_id);
        return -1;
    }

    const struct pinctrl_state *state = &pcfg->states[state_id];

    if (pin_id >= state->pin_cnt) {
        LOG_ERR("Could not retrieve pinctrl pin config, because there is no pin_id %d ", pin_id);
        return -1;
    }

    pinctrl_soc_pin_t pin_psel = state->pins[pin_id];

    *res_port = PINCTRL_PSEL_NRF_EXTRACT_PORT(pin_psel);
    *res_pin = PINCTRL_PSEL_NRF_EXTRACT_PIN(pin_psel);

    return 0;
#else

    LOG_ERR("Could not retrieve pinctrl pin config, because the code is only designed to run on "
            "nrf52, but you are running on %s",
            CONFIG_SOC_SERIES);

    *res_port = -1;
    *res_pin = -1;

    return -1;

#endif
}

/*
 * Reading PS2 data
 */
static void ps2_uart_interrupt_handler(const struct device *uart_dev, void *user_data);
void ps2_uart_read_interrupt_handler(const struct device *uart_dev, void *user_data);
static int ps2_uart_read_err_check(const struct device *dev);
void ps2_uart_read_process_received_byte(const struct device *dev, uint8_t byte);
const char *ps2_uart_read_get_error_str(int err);

static void ps2_uart_interrupt_handler(const struct device *uart_dev, void *user_data) {
    // const struct device* dev = (const struct device*)user_data;
    int err;

    err = uart_irq_update(uart_dev);
    if (err != 1) {
        LOG_ERR("uart_irq_update returned: %d", err);
        return;
    }

    while (uart_irq_rx_ready(uart_dev)) {
        ps2_uart_read_interrupt_handler(uart_dev, user_data);
    }
}

void ps2_uart_read_interrupt_handler(const struct device *uart_dev, void *user_data) {
    const struct device* dev = (const struct device*)user_data;
    uint8_t byte;

    int byte_len = uart_fifo_read(uart_dev, &byte, 1);
    if (byte_len < 1) {
        LOG_ERR("UART read failed with error: %d", byte_len);
        return;
    }

    ps2_uart_read_process_received_byte(dev, byte);
}

static int ps2_uart_read_err_check(const struct device *dev) {
    // TOOD: Make this function only work if nrf52 is used
    int err = uart_err_check(dev);

    // In the config we enabled even parity, because nrf52 does
    // not support odd parity.
    // But PS/2 uses odd parity. This should generate a parity error on
    // every reception.
    // If the parity error doesn't happen, it means the transfer had an
    // actual even parity, which we consider an error.
    if ((err & NRF_UARTE_ERROR_PARITY_MASK) == 0) {
        err = UART_ERROR_PARITY;
    } else if (err & NRF_UARTE_ERROR_OVERRUN_MASK) {
        err = UART_ERROR_OVERRUN;
    } else if (err & NRF_UARTE_ERROR_FRAMING_MASK) {
        err = UART_ERROR_FRAMING;
    } else if (err & NRF_UARTE_ERROR_BREAK_MASK) {
        err = UART_BREAK;
    } else { // No errors
        err = 0;
    }

    return err;
}

void ps2_uart_read_process_received_byte(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;

    int err;

    LOG_DBG("UART Received: 0x%x", byte);

    err = ps2_uart_read_err_check(config->uart_dev);
    if (err != 0) {
        const char *err_str = ps2_uart_read_get_error_str(err);

        // Framing errors
        if (byte == 0xfa && err == UART_ERROR_FRAMING) {
            // Ignore, because it is not a real error and happens frequently
        } else {
            LOG_WRN("UART RX detected error for byte 0x%x: %s (%d)", byte, err_str, err);
        }
    }

    LOG_DBG("Received byte: 0x%x", byte);

    // If write_byte_await_response() is waiting, we notify
    // the blocked write process of whether it was a success or not.
    if (data->write_awaits_resp) {
        data->write_awaits_resp_byte = byte;
        data->write_awaits_resp = false;
        k_sem_give(&data->write_awaits_resp_sem);

        // Don't send ack and err responses to the callback and read
        // data queue.
        // If it's an ack, the write process will return success.
        // If it's an error, the write process will return failure.
        if (byte == PS2_UART_RESP_ACK || byte == PS2_UART_RESP_RESEND ||
            byte == PS2_UART_RESP_FAILURE) {

            return;
        }
    }

    // If no callback is set, we add the data to a fifo queue
    // that can be read later with the read using `ps2_read`
    if (data->callback_isr != NULL && data->callback_enabled) {

        // Call callback from a worker to make sure the callback
        // doesn't block the interrupt.
        // Will call ps2_uart_read_callback_work_handler
        data->callback_byte = byte;
        k_work_submit_to_queue(&ps2_uart_work_queue_cb, &data->callback_work);
    } else {
        ps2_uart_data_queue_add(dev, byte);
    }
}

const char *ps2_uart_read_get_error_str(int err) {
    switch (err) {
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

void ps2_uart_read_callback_work_handler(struct k_work *work) {

    // struct k_work_delayable *work_delayable = (struct k_work_delayable *)item;
    struct ps2_uart_data *data = CONTAINER_OF(work,
                                              struct ps2_uart_data,
                                              callback_work);
    // const struct device *dev = data->dev;

    data->callback_isr(data->dev, data->callback_byte);
    data->callback_byte = 0x0;
}

/*
 * Writing PS2 data
 */

int ps2_uart_write_byte_await_response(const struct device *dev, uint8_t byte);
int ps2_uart_write_byte_blocking(const struct device *dev, uint8_t byte);
int ps2_uart_write_byte_start(const struct device *dev, uint8_t byte);
void ps2_uart_write_scl_interrupt_handler(const struct device *dev, struct gpio_callback *cb,
                                          uint32_t pins);
void ps2_uart_write_scl_timeout(struct k_work *item);
void ps2_uart_write_finish(const struct device *dev, bool successful, char *descr);

// Returned when there was an error writing to the PS2 device, such
// as not getting a clock from the device or receiving an invalid
// ack bit.
#define PS2_UART_E_WRITE_TRANSMIT 1

// Returned when the semaphore times out. Theoretically this shouldn't be
// happening. But it can happen if the same thread is used for both the
// semaphore wait and the inhibition timeout.
#define PS2_UART_E_WRITE_SEM_TIMEOUT 2

// Returned when the write finished seemingly successful, but the
// device didn't send a response in time.
#define PS2_UART_E_WRITE_RESPONSE 3

// Returned when the write finished seemingly successful, but the
// device responded with 0xfe (request to resend) and we ran out of
// retry attempts.
#define PS2_UART_E_WRITE_RESEND 4

// Returned when the write finished seemingly successful, but the
// device responded with 0xfc (failure / cancel).
#define PS2_UART_E_WRITE_FAILURE 5

K_MUTEX_DEFINE(ps2_uart_write_mutex);

int ps2_uart_write_byte_debug(const struct device *dev, uint8_t byte) {
    int err;

    LOG_WRN("DEBUG WRITE STARTED for byte 0x%x", byte);

    LOG_WRN("Setting Write mode");
    err = ps2_uart_set_mode_write(dev);
    if (err != 0) {
        LOG_ERR("Could not configure driver for write mode: %d", err);
        return err;
    }
    // k_sleep(K_MSEC(1000));
    // err = ps2_uart_set_mode_write(dev);
    // if (err != 0) {
    //  LOG_ERR("Could not configure driver for write mode: %d", err);
    //  return err;
    // }
    LOG_WRN("Setting Write mode: Done");

    // Inhibit the line by setting clock low and data high for 100us
    LOG_INF("Setting low");
    ps2_uart_set_scl(dev, 0);
    ps2_uart_set_sda(dev, 0);
    k_sleep(K_MSEC(100));

    LOG_INF("Setting high");
    ps2_uart_set_scl(dev, 1);
    ps2_uart_set_sda(dev, 1);
    k_sleep(K_MSEC(100));

    LOG_INF("Setting low");
    ps2_uart_set_scl(dev, 0);
    ps2_uart_set_sda(dev, 0);
    k_sleep(K_MSEC(100));

    LOG_INF("Setting high");
    ps2_uart_set_scl(dev, 1);
    ps2_uart_set_sda(dev, 1);
    k_sleep(K_MSEC(100));

    LOG_INF("Setting low");
    ps2_uart_set_scl(dev, 0);
    ps2_uart_set_sda(dev, 0);
    k_sleep(K_MSEC(100));

    LOG_INF("Setting high");
    ps2_uart_set_scl(dev, 1);
    ps2_uart_set_sda(dev, 1);
    k_sleep(K_MSEC(100));

    LOG_INF("Setting low");
    ps2_uart_set_scl(dev, 0);
    ps2_uart_set_sda(dev, 0);
    k_sleep(K_MSEC(100));

    LOG_WRN("Enabling interrupt callback");
    ps2_uart_set_scl_callback_enabled(dev, true);

    LOG_WRN("Setting SCL input");
    ps2_uart_configure_pin_scl_input(dev);

    k_sleep(K_MSEC(300));

    LOG_WRN("Switching back to mode read");
    err = ps2_uart_set_mode_read(dev);
    if (err != 0) {
        LOG_ERR("Could not configure driver for write mode: %d", err);
        return err;
    }

    LOG_WRN("Finished Debug write");

    return -1;
}

int ps2_uart_write_byte(const struct device *dev, uint8_t byte) {
    int err;

    LOG_DBG("\n");
    LOG_DBG("Writing: 0x%x", byte);

    k_mutex_lock(&ps2_uart_write_mutex, K_FOREVER);

    for (int i = 0; i < PS2_UART_WRITE_MAX_RETRY; i++) {
        if (i > 0) {
            LOG_WRN("Attempting write re-try #%d of %d...", i + 1, PS2_UART_WRITE_MAX_RETRY);
        }

        err = ps2_uart_write_byte_await_response(dev, byte);

        if (err == 0) {
            if (i > 0) {
                LOG_WRN("Successfully wrote 0x%x on try #%d of %d...", byte, i + 1,
                        PS2_UART_WRITE_MAX_RETRY);
            }
            break;
        } else if (err == PS2_UART_E_WRITE_FAILURE) {
            // Write failed and the device requested to stop trying
            // to resend.
            break;
        }
    }

    LOG_DBG("END WRITE: 0x%x\n", byte);
    k_mutex_unlock(&ps2_uart_write_mutex);

    return err;
}

// Writes the byte and blocks execution until we read the
// response byte.
// Returns failure if the write fails or the response is 0xfe/0xfc (error)
// Returns success if the response is 0xfa (ack) or any value except of
// 0xfe.
// 0xfe, 0xfc and 0xfa are not passed on to the read data queue or callback.
int ps2_uart_write_byte_await_response(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    int err;

    err = ps2_uart_write_byte_blocking(dev, byte);
    if (err) {
        return err;
    }

    data->write_awaits_resp = true;

    err = k_sem_take(&data->write_awaits_resp_sem, PS2_UART_TIMEOUT_WRITE_AWAIT_RESPONSE);

    uint8_t resp_byte = data->write_awaits_resp_byte;
    data->write_awaits_resp_byte = 0x0;
    data->write_awaits_resp = false;

    if (err) {
        LOG_WRN("Write response didn't arrive in time for byte "
                "0x%x. Considering send a failure.",
                byte);

        return PS2_UART_E_WRITE_RESPONSE;
    }

    if (resp_byte == PS2_UART_RESP_RESEND || resp_byte == PS2_UART_RESP_FAILURE) {
        LOG_WRN("Write of 0x%x received error response: 0x%x", byte, resp_byte);
    } else {
        LOG_DBG("Write for byte 0x%x received response: 0x%x", byte, resp_byte);
    }

    // We fail the write since we got an error response
    if (resp_byte == PS2_UART_RESP_RESEND) {

        return PS2_UART_E_WRITE_RESEND;
    } else if (resp_byte == PS2_UART_RESP_FAILURE) {

        return PS2_UART_E_WRITE_FAILURE;
    }

    // Most of the time when a write was successful the device
    // responds with an 0xfa (ack), but for some commands it doesn't.
    // So we consider all non-0xfe and 0xfc responses as successful.
    return 0;
}

int ps2_uart_write_byte_blocking(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    int err;

    // LOG_DBG("ps2_uart_write_byte_blocking called with byte=0x%x", byte);

    err = ps2_uart_write_byte_start(dev, byte);
    if (err) {
        LOG_ERR("Could not initiate writing of byte.");
        return PS2_UART_E_WRITE_TRANSMIT;
    }

    // The async `write_byte_start` function takes the only available semaphor.
    // This causes the `k_sem_take` call below to block until
    // `ps2_uart_write_finish` gives it back.
    err = k_sem_take(&data->write_lock, PS2_UART_TIMEOUT_WRITE_BLOCKING);
    if (err) {

        // This usually means the controller is busy with other interrupts,
        // timed out processing the interrupts and even the scl timeout
        // delayable wasn't called due to the delay.
        //
        // So we abort the write and try again.
        LOG_ERR("Blocking write failed due to semaphore timeout for byte "
                "0x%x: %d",
                byte, err);

        return PS2_UART_E_WRITE_SEM_TIMEOUT;
    }

    if (data->cur_write_status == PS2_UART_WRITE_STATUS_SUCCESS) {
        // LOG_DBG("Blocking write finished successfully for byte 0x%x", byte);
        err = 0;
    } else {
        LOG_ERR("Blocking write finished with failure for byte 0x%x status: %d", byte,
                data->cur_write_status);
        err = -data->cur_write_status;
    }

    data->cur_write_status = PS2_UART_WRITE_STATUS_INACTIVE;

    return err;
}

int ps2_uart_write_byte_start(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    int err;

    // Take semaphore so that when `ps2_uart_write_byte_blocking` attempts
    // taking it, the process gets blocked.
    err = k_sem_take(&data->write_lock, K_NO_WAIT);
    if (err != 0 && err != -EBUSY) {
        LOG_ERR("ps2_uart_write_byte_start could not take semaphore: %d", err);

        return err;
    }

    err = ps2_uart_set_mode_write(dev);
    if (err != 0) {
        LOG_ERR("Could not configure driver for write mode: %d", err);
        return err;
    }

    // Set the write byte so it can be used in
    // the downstream write function that is called
    // from the SCL interrupt
    data->cur_write_byte = byte;
    data->cur_write_pos = PS2_UART_POS_START;

    // Inhibit the line by setting clock low and data high for 100us
    ps2_uart_set_scl(dev, 0);
    ps2_uart_set_sda(dev, 1);
    k_busy_wait(PS2_UART_TIMING_SCL_INHIBITION);

    // Set data to value of start bit
    ps2_uart_set_sda(dev, 0);
    k_busy_wait(PS2_UART_TIMING_SCL_INHIBITION);

    // The start bit was sent by setting sda to low
    // So the next scl interrupt will be for the first
    // data bit.
    data->cur_write_pos += 1;

    // Release the clock line and configure it as input
    // This let's the device take control of the clock again
    ps2_uart_set_scl(dev, 1);
    ps2_uart_configure_pin_scl_input(dev);

    // We need to wait for the first SCL clock
    // Execution continues once it arrives in
    // `ps2_uart_write_scl_interrupt_handler`
    ps2_uart_set_scl_callback_enabled(dev, true);

    // And if the PS/2 device doesn't start the clock, we want to
    // handle that error...
    k_work_schedule_for_queue(&ps2_uart_work_queue, &data->write_scl_timout,
                              PS2_UART_TIMEOUT_WRITE_SCL_START);

    k_mutex_unlock(&ps2_uart_write_mutex);

    return 0;
}

void ps2_uart_write_scl_timeout(struct k_work *item) {

    struct k_work_delayable *work_delayable = (struct k_work_delayable *)item;
    struct ps2_uart_data *data = CONTAINER_OF(work_delayable,
                                              struct ps2_uart_data,
                                              write_scl_timout);
    const struct device *dev = data->dev;

    // Once we start a transmission we expect the device to
    // to send a new clock/interrupt within
    // PS2_UART_TIMEOUT_WRITE_SCL_START us.
    // If we don't receive the next interrupt within that timeframe,
    // we abort the write.

    ps2_uart_write_finish(dev, false, "scl timeout");
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
void ps2_uart_write_scl_interrupt_handler_blocking(struct ps2_uart_data *data,
                                                   const struct device *gpio_dev,
                                                   struct gpio_callback *cb, 
                                                   uint32_t pins) {
    LOG_INF("Inside ps2_uart_write_scl_interrupt_handler_blocking");

    // Cancel the SCL timeout
    k_work_cancel_delayable(&data->write_scl_timout);

    // Disable the SCL interrupt again.
    // From here we will just use time delays.
    ps2_uart_set_scl_callback_enabled(data->dev, false);

    for (int i = PS2_UART_POS_DATA_FIRST; i <= PS2_UART_POS_STOP; i++) {

        if (i >= PS2_UART_POS_DATA_FIRST && i <= PS2_UART_POS_DATA_LAST) {

            int data_pos = i - PS2_UART_POS_DATA_FIRST;
            bool data_bit = PS2_UART_GET_BIT(data->cur_write_byte, data_pos);

            ps2_uart_set_sda(data->dev, data_bit);
        } else if (i == PS2_UART_POS_PARITY) {

            bool byte_parity = ps2_uart_get_byte_parity(data->cur_write_byte);

            ps2_uart_set_sda(data->dev, byte_parity);
        } else if (i == PS2_UART_POS_STOP) {

            ps2_uart_set_sda(data->dev, 1);

            // Give control over data pin back to device after sending
            // the stop bit so that we can receive the ack bit from the
            // device
            ps2_uart_configure_pin_sda_input(data->dev);
        } else {
            LOG_ERR("UART unknown TX bit number: %d", i);
        }

        // Sleep for the cycle length
        k_busy_wait(PS2_UART_TIMING_SCL_CYCLE_LEN);
    }

    // Check Ack
    int ack_val = ps2_uart_get_sda(data->dev);

    if (ack_val == 0) {
        ps2_uart_write_finish(data->dev, true, "successful ack");
    } else {
        // TODO: Properly handle write ack errors
        LOG_WRN("Ack bit was invalid for write of 0x%x", data->cur_write_byte);
        ps2_uart_write_finish(data->dev, true, "failed ack");
    }
}

void ps2_uart_write_scl_interrupt_handler_async(struct ps2_uart_data *data,
                                                const struct device *gpio_dev,
                                                struct gpio_callback *cb,
                                                uint32_t pins) {
    k_work_cancel_delayable(&data->write_scl_timout);

    if (data->cur_write_pos == PS2_UART_POS_START) {
        // This should not be happening, because the PS2_UART_POS_START bit
        // is sent in ps2_uart_write_byte_start during inhibition
        return;
    } else if (data->cur_write_pos >= PS2_UART_POS_DATA_FIRST &&
               data->cur_write_pos <= PS2_UART_POS_DATA_LAST) {

        int data_pos = data->cur_write_pos - PS2_UART_POS_DATA_FIRST;
        bool data_bit = PS2_UART_GET_BIT(data->cur_write_byte, data_pos);

        ps2_uart_set_sda(data->dev, data_bit);
    } else if (data->cur_write_pos == PS2_UART_POS_PARITY) {

        bool byte_parity = ps2_uart_get_byte_parity(data->cur_write_byte);

        ps2_uart_set_sda(data->dev, byte_parity);
    } else if (data->cur_write_pos == PS2_UART_POS_STOP) {

        ps2_uart_set_sda(data->dev, 1);

        // Give control over data pin back to device after sending
        // the stop bit so that we can receive the ack bit from the
        // device
        ps2_uart_configure_pin_sda_input(data->dev);
    } else if (data->cur_write_pos == PS2_UART_POS_ACK) {

        int ack_val = ps2_uart_get_sda(data->dev);

        if (ack_val == 0) {
            ps2_uart_write_finish(data->dev, true, "successful ack");
        } else {
            // TODO: Properly handle write ack errors
            LOG_WRN("Ack bit was invalid for write of 0x%x", data->cur_write_byte);
            ps2_uart_write_finish(data->dev, true, "failed ack");
        }
    } else {
        LOG_ERR("UART unknown TX bit number: %d", data->cur_write_pos);
    }

    if (data->cur_write_pos < PS2_UART_POS_ACK) {
        k_work_schedule_for_queue(&ps2_uart_work_queue, &data->write_scl_timout,
                                  PS2_UART_TIMEOUT_WRITE_SCL);
    }

    data->cur_write_pos += 1;
}

void ps2_uart_write_finish(const struct device *dev, bool successful, char *descr) {
    struct ps2_uart_data *data = dev->data;
    int err;

    k_work_cancel_delayable(&data->write_scl_timout);

    if (successful) {
        LOG_DBG("Successfully wrote value 0x%x", data->cur_write_byte);
        data->cur_write_status = PS2_UART_WRITE_STATUS_SUCCESS;
    } else { // Failure
        LOG_ERR("Failed to write value 0x%x: %s", data->cur_write_byte, descr);

        data->cur_write_status = PS2_UART_WRITE_STATUS_FAILURE;
    }

    err = ps2_uart_set_mode_read(dev);
    if (err != 0) {
        LOG_ERR("Could not configure driver for read mode: %d", err);
        return;
    }

    LOG_DBG("END WRITE: 0x%x\n", data->cur_write_byte);

    data->cur_write_byte = 0x0;

    // Give the semaphore to allow write_byte_blocking to continue
    k_sem_give(&data->write_lock);

    k_mutex_unlock(&ps2_uart_write_mutex);
}

/*
 * Zephyr PS/2 driver interface
 */
static int ps2_uart_enable_callback(const struct device *dev);

#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)

static int ps2_uart_configure(const struct device *dev, ps2_callback_t callback_isr,
                              ps2_resend_callback_t resend_callback_isr) {
    struct ps2_uart_data *data = dev->data;

    if (!callback_isr && !resend_callback_isr) {
        return -EINVAL;
    }

    if (callback_isr) {
        data->callback_isr = callback_isr;
        ps2_uart_enable_callback(dev);
    }

    if (resend_callback_isr) {
        data->resend_callback_isr = resend_callback_isr;
    }

    return 0;
}

#else

static int ps2_uart_configure(const struct device *dev, ps2_callback_t callback_isr) {
    struct ps2_uart_data *data = dev->data;

    if (!callback_isr) {
        return -EINVAL;
    }

    data->callback_isr = callback_isr;
    ps2_uart_enable_callback(dev);

    return 0;
}

#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

int ps2_uart_read(const struct device *dev, uint8_t *value) {
    uint8_t queue_byte;
    int err = ps2_uart_data_queue_get_next(dev, &queue_byte, PS2_UART_TIMEOUT_READ);
    if (err) { // Timeout due to no data to read in data queue
        // LOG_DBG("ps2_uart_read: Fifo timed out...");

        return -ETIMEDOUT;
    }

    // LOG_DBG("ps2_uart_read: Returning 0x%x", queue_byte);
    *value = queue_byte;

    return 0;
}

static int ps2_uart_write(const struct device *dev, uint8_t value) {
    int ret = ps2_uart_write_byte(dev, value);

    return ret;
}

static int ps2_uart_disable_callback(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;

    // Make sure there are no stale items in the data queue
    // from before the callback was disabled.
    ps2_uart_data_queue_empty(dev);

    data->callback_enabled = false;

    // LOG_DBG("Disabled PS2 callback.");

    return 0;
}

static int ps2_uart_enable_callback(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;
    data->callback_enabled = true;

    // LOG_DBG("Enabled PS2 callback.");

    ps2_uart_data_queue_empty(dev);

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
static int ps2_uart_init_uart(const struct device *dev);
static int ps2_uart_init_gpio(const struct device *dev);

static int ps2_uart_init(const struct device *dev) {
    int err;
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;

    // Set the ps2 device so we can retrieve it later for
    // the ps2 callback
    data->dev = dev;

    data->callback_byte = 0x0;
    data->callback_isr = NULL;
    data->callback_enabled = false;
    data->cur_write_status = PS2_UART_WRITE_STATUS_INACTIVE;
    data->cur_write_byte = 0x0;
    data->cur_write_pos = 0;
    data->write_awaits_resp = false;
    data->write_awaits_resp_byte = 0x0;

#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)
    data->resend_callback_isr = NULL;
#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

    // Get the P0.08 pin notation of the configured UART pinctrl pin.
    // The UART TX pin is the second (index 1) of the pins
    int pinctrl_port = -1;
    int pinctrl_pin = -1;
    ps2_uart_get_pinctrl_port_pin(config->pcfg, PINCTRL_STATE_DEFAULT, 1, &pinctrl_port,
                                  &pinctrl_pin);

    LOG_INF("Initializing ps2_uart driver with pins... SCL: P%d.%02d; SDA: P%d.%02d; SDA Pinctrl: "
            "P%d.%02d",
            config->scl_gpio_port_num, config->scl_gpio.pin, config->sda_gpio_port_num,
            config->sda_gpio.pin, pinctrl_port, pinctrl_pin);

    // Init data queue for synchronous read operations
    k_msgq_init(&data->data_queue, data->data_queue_buffer, sizeof(struct ps2_uart_data_queue_item),
                PS2_UART_DATA_QUEUE_SIZE);

    if (config->ps2_uart_idx == 0) {
        // Custom queue for background PS/2 processing work at high priority
        k_work_queue_start(&ps2_uart_work_queue, ps2_uart_work_queue_stack_area,
                        K_THREAD_STACK_SIZEOF(ps2_uart_work_queue_stack_area),
                        PS2_UART_WORK_QUEUE_PRIORITY, NULL);

        // Custom queue for calling the zephyr ps/2 callback at lower priority
        k_work_queue_start(&ps2_uart_work_queue_cb, ps2_uart_work_queue_cb_stack_area,
                        K_THREAD_STACK_SIZEOF(ps2_uart_work_queue_cb_stack_area),
                        PS2_UART_WORK_QUEUE_CB_PRIORITY, NULL);
    }

    k_work_init(&data->callback_work, ps2_uart_read_callback_work_handler);

    k_work_init_delayable(&data->write_scl_timout, ps2_uart_write_scl_timeout);

    // Init semaphore for blocking writes
    k_sem_init(&data->write_lock, 0, 1);

    // Init semaphore that waits for read after write
    k_sem_init(&data->write_awaits_resp_sem, 0, 1);

    err = ps2_uart_init_uart(dev);
    if (err != 0) {
        LOG_ERR("Could not init UART: %d", err);
        return err;
    }

    err = ps2_uart_init_gpio(dev);
    if (err != 0) {
        LOG_ERR("Could not init GPIO: %d", err);
        return err;
    }

    err = ps2_uart_set_mode_read(dev);
    if (err != 0) {
        LOG_ERR("Could not initialize in UART mode read: %d", err);
        return err;
    }

    return 0;
}

static int ps2_uart_init_uart(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;
    int err;

    if (!device_is_ready(config->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    } else {
        LOG_INF("UART device is ready");
    }

    struct uart_config uart_cfg;
    err = uart_config_get(config->uart_dev, &uart_cfg);
    if (err != 0) {
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
    if (err != 0) {
        LOG_ERR("Could not configure UART device: %d", err);
        return -EINVAL;
    }

    uart_irq_callback_user_data_set(config->uart_dev, ps2_uart_interrupt_handler,
                                    (void *)data->dev);

    uart_irq_rx_enable(config->uart_dev);
    uart_irq_err_enable(config->uart_dev);

    return 0;
}

static int ps2_uart_init_gpio(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;
    int err;

    // Interrupt for clock line
#if IS_ENABLED(CONFIG_PS2_UART_WRITE_MODE_BLOCKING)
    gpio_init_callback(&data->scl_cb_data, data->scl_rupt_blocking,
                       BIT(config->scl_gpio.pin));
#else /* IS_ENABLED(CONFIG_PS2_UART_WRITE_MODE_BLOCKING) */
    gpio_init_callback(&data->scl_cb_data, data->scl_rupt_async,
                       BIT(config->scl_gpio.pin));
#endif /* IS_ENABLED(CONFIG_PS2_UART_WRITE_MODE_BLOCKING) */

    err = gpio_add_callback(config->scl_gpio.port, &data->scl_cb_data);
    if (err) {
        LOG_ERR("failed to enable interrupt callback on "
                "SCL GPIO pin (err %d)",
                err);
    }

    LOG_INF("Disabling callback...");
    ps2_uart_set_scl_callback_enabled(dev, false);

    return err;
}


// Define wrapper function declaration for write_scl_interrupt_handler_*
// will assign to ps2_uart_data in below
#define PS2_UART_SCL_INTERRUPT_DEFINE(n)                                             \
    void ps2_uart_write_scl_interrupt_handler_blocking_##n(                          \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins);     \
    void ps2_uart_write_scl_interrupt_handler_async_##n(                             \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins);

DT_INST_FOREACH_STATUS_OKAY(PS2_UART_SCL_INTERRUPT_DEFINE)


#define PS2_UART_DEFINE(n)                                                           \
    PINCTRL_DT_DEFINE(DT_INST_BUS(n));                                               \
    static struct ps2_uart_data data##n = {                                          \
        .scl_rupt_blocking = &ps2_uart_write_scl_interrupt_handler_blocking_##n,     \
        .scl_rupt_async = &ps2_uart_write_scl_interrupt_handler_async_##n,           \
    };                                                                               \
    static const struct ps2_uart_config config##n = {                                \
        .ps2_uart_idx = n,                                                           \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                   \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                             \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                             \
        .pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_BUS(n)),                           \
        .scl_gpio_port_num = DT_PROP(DT_INST_PHANDLE(n, scl_gpios), port),           \
        .sda_gpio_port_num = DT_PROP(DT_INST_PHANDLE(n, sda_gpios), port),           \
    };                                                                               \
    DEVICE_DT_INST_DEFINE(n, &ps2_uart_init, NULL, &data##n, &config##n,             \
                          POST_KERNEL, 80,                                           \
                          &ps2_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_UART_DEFINE)


// Define wrapper function implementation for write_scl_interrupt_handler_*
// assigned to ps2_uart_data on above
#define PS2_UART_SCL_INTERRUPT_IMPL_DEFINE(n)                                        \
    void ps2_uart_write_scl_interrupt_handler_blocking_##n(                          \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins) {    \
        ps2_uart_write_scl_interrupt_handler_blocking(&data##n, gpio_dev, cb, pins); \
    }                                                                                \
    void ps2_uart_write_scl_interrupt_handler_async_##n(                             \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins) {    \
        ps2_uart_write_scl_interrupt_handler_async(&data##n, gpio_dev, cb, pins);    \
    }

DT_INST_FOREACH_STATUS_OKAY(PS2_UART_SCL_INTERRUPT_IMPL_DEFINE)
