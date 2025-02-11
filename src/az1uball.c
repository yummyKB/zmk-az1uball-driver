/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT palette_az1uball

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/activity_state_changed.h>
#include <math.h>
#include "az1uball.h"
LOG_MODULE_REGISTER(az1uball, LOG_LEVEL_DBG);

volatile uint8_t AZ1UBALL_MOUSE_MAX_SPEED = 25;
volatile uint8_t AZ1UBALL_MOUSE_MAX_TIME = 5;
volatile float AZ1UBALL_MOUSE_SMOOTHING_FACTOR = 1.3f;
volatile uint8_t AZ1UBALL_SCROLL_MAX_SPEED = 1;
volatile uint8_t AZ1UBALL_SCROLL_MAX_TIME = 1;
volatile float AZ1UBALL_SCROLL_SMOOTHING_FACTOR = 0.5f;
volatile float AZ1UBALL_HUE_INCREMENT_FACTOR = 0.3f;

enum az1uball_mode {
    AZ1UBALL_MODE_MOUSE,
    AZ1UBALL_MODE_SCROLL
};

static enum az1uball_mode current_mode = AZ1UBALL_MODE_MOUSE;

/* Forward declaration of functions */
static void activate_automouse_layer();
static void deactivate_automouse_layer(struct k_timer *timer);

static int previous_x = 0;
static int previous_y = 0;

void az1uball_enable_sleep(const struct device *dev) {
    struct palette_az1uball_data *data = dev->data;

    const struct palette_az1uball_config *config = data->dev->config;
    uint8_t ctrl_reg_value;

    // Read the current control register value
    if (i2c_reg_read_byte_dt(&config->i2c, MSK_CTRL_SLEEP, &ctrl_reg_value) != 0) {
        LOG_ERR("Failed to read AZ1UBALL control register");
        return;
    }

    ctrl_reg_value |= MSK_CTRL_SLEEP; // Set the SLEEP bit

    // Write the modified value back
    if (i2c_reg_write_byte_dt(&config->i2c, MSK_CTRL_SLEEP, ctrl_reg_value) != 0) {
        LOG_ERR("Failed to write AZ1UBALL control register");
        return;
    }

    LOG_DBG("AZ1UBALL sleep enabled");
}

void az1uball_disable_sleep(const struct device *dev) {
    struct palette_az1uball_data *data = dev->data;

    const struct palette_az1uball_config *config = data->dev->config;
    uint8_t ctrl_reg_value;

    // Read the current control register value
    if (i2c_reg_read_byte_dt(&config->i2c, MSK_CTRL_SLEEP, &ctrl_reg_value) != 0) {
        LOG_ERR("Failed to read AZ1UBALL control register");
        return;
    }

    ctrl_reg_value &= ~MSK_CTRL_SLEEP; // Clear the SLEEP bit

    // Write the modified value back
    if (i2c_reg_write_byte_dt(&config->i2c, MSK_CTRL_SLEEP, ctrl_reg_value) != 0) {
        LOG_ERR("Failed to write AZ1UBALL control register");
        return;
    }

    LOG_DBG("AZ1UBALL sleep disabled");
}

void az1uball_toggle_mode(void) {
    current_mode = (current_mode == AZ1UBALL_MODE_MOUSE) ? AZ1UBALL_MODE_SCROLL : AZ1UBALL_MODE_MOUSE;
    // Optional: Add logging or LED indication here to show the current mode
    LOG_DBG("AZ1UBALL mode switched to %s", (current_mode == AZ1UBALL_MODE_MOUSE) ? "MOUSE" : "SCROLL");
}

// Event handler for activity state changes
static int activity_state_changed_handler(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    // Get the device pointer
    const struct device *dev = DEVICE_DT_GET(DT_COMPAT(palette_az1uball);
    if (!device_is_ready(dev)) {
        LOG_ERR("AZ1UBALL device not ready");
        return -ENODEV;
    }

    if (ev->state == ZMK_ACTIVITY_IDLE) {
        az1uball_enable_sleep(dev);
    }

    if (ev->state != ZMK_ACTIVITY_IDLE) {
        az1uball_disable_sleep(dev);
    }

    return 0;
}

ZMK_LISTENER(idle_listener, activity_state_changed_handler);
ZMK_SUBSCRIPTION(idle_listener, zmk_activity_state_changed);

static void az1uball_process_movement(struct palette_az1uball_data *data, int delta_x, int delta_y, uint32_t time_between_interrupts, int max_speed, int max_time, float smoothing_factor) {
    float scaling_factor = 1.0f;
    if (time_between_interrupts < max_time) {
        // Exponential scaling calculation
        float exponent = -3.0f * (float)time_between_interrupts / max_time; // Adjust -3.0f for desired curve
        scaling_factor = 1.0f + (max_speed - 1.0f) * expf(exponent);
    }

    // Apply scaling based on mode
    if (current_mode == AZ1UBALL_MODE_SCROLL) {
        scaling_factor *= 2.5f; // Example: Increase scaling for scroll mode
    }

    /* Accumulate deltas atomically */
    atomic_add(&data->x_buffer, delta_x);
    atomic_add(&data->y_buffer, delta_y);

    int scaled_x_movement = (int)(delta_x * scaling_factor);
    int scaled_y_movement = (int)(delta_y * scaling_factor);

    // Apply smoothing
    data->smoothed_x = (int)(smoothing_factor * scaled_x_movement + (1.0f - smoothing_factor) * previous_x);
    data->smoothed_y = (int)(smoothing_factor * scaled_y_movement + (1.0f - smoothing_factor) * previous_y);

    data->previous_x = data->smoothed_x;
    data->previous_y = data->smoothed_y;
}

static void palette_az1uball_work_handler(struct k_work *work) {
    struct palette_az1uball_data *data = CONTAINER_OF(work, struct palette_az1uball_data, irq_work);
    const struct palette_az1uball_config *config = data->dev->config;
    const struct device *dev = data->dev;
    uint8_t buf[5];
    int ret;

    LOG_INF("AZ1UBALL work handler triggered");


    /* Read movement data and switch state */
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, 5);
    if (ret) {
        LOG_ERR("Failed to read movement data from AZ1UBALL: %d", ret);
        return;
    }

    uint32_t time_between_interrupts;

    k_mutex_lock(&data->data_lock, K_FOREVER);
    time_between_interrupts = data->last_interrupt_time - data->previous_interrupt_time;
    k_mutex_unlock(&data->data_lock);

    /* Calculate deltas */
    int16_t delta_x = (int16_t)buf[1] - (int16_t)buf[0]; // RIGHT - LEFT
    int16_t delta_y = (int16_t)buf[3] - (int16_t)buf[2]; // DOWN - UP

    /* Report movement immediately if non-zero */
    if (delta_x != 0 || delta_y != 0) {
        if (current_mode == AZ1UBALL_MODE_MOUSE) {
            az1uball_process_movement(data, delta_x, delta_y, time_between_interrupts, AZ1UBALL_MOUSE_MAX_SPEED, AZ1UBALL_MOUSE_MAX_TIME, AZ1UBALL_MOUSE_SMOOTHING_FACTOR);

            /* Report relative X movement */
            if (delta_x != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_X, data->smoothed_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", data->smoothed_x);
                }
            }

            /* Report relative Y movement */
            if (delta_y != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_Y, data->smoothed_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", data->smoothed_y);
                }
            }
        } else if (current_mode == AZ1UBALL_MODE_SCROLL) {
            az1uball_process_movement(data, delta_x, delta_y, time_between_interrupts, AZ1UBALL_SCROLL_MAX_SPEED, AZ1UBALL_SCROLL_MAX_TIME, AZ1UBALL_SCROLL_SMOOTHING_FACTOR);

            /* Report relative X movement */
            if (delta_x != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_WHEEL, data->smoothed_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", data->smoothed_x);
                }
            }

            /* Report relative Y movement */
            if (delta_y != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_HWHEEL, data->smoothed_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", data->smoothed_y);
                }
            }
        }
    }

    /* Update switch state */
    data->sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    /* Report switch state if it changed */
    if (data->sw_pressed != data->sw_pressed_prev) {
        ret = input_report_key(data->dev, INPUT_BTN_0, data->sw_pressed ? 1 : 0, true, K_NO_WAIT);
        if (ret) {
            LOG_ERR("Failed to report key");
        } else {
            LOG_DBG("Reported key");
        }

        LOG_DBG("Reported switch state: %d", data->sw_pressed);

        data->sw_pressed_prev = data->sw_pressed;
    }

    /* Clear movement registers */
    uint8_t zero = 0;
    i2c_reg_write_byte_dt(&config->i2c, REG_LEFT, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_RIGHT, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_UP, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_DOWN, zero);

}

/* Enable function */
static int palette_az1uball_enable(const struct device *dev) {
    const struct palette_az1uball_config *config = dev->config;
    struct palette_az1uball_data *data = dev->data;
    int ret;

    LOG_INF("palette_az1uball_enable called");


    LOG_INF("palette_az1uball enabled");

    return 0;
}

/* Disable function */
static int palette_az1uball_disable(const struct device *dev) {
    const struct palette_az1uball_config *config = dev->config;
    struct palette_az1uball_data *data = dev->data;
    int ret;

    LOG_INF("palette_az1uball_disable called");

    LOG_INF("palette_az1uball disabled");

    return 0;
}


/* Device initialization function */
static int palette_az1uball_init(const struct device *dev) {
    const struct palette_az1uball_config *config = dev->config;
    struct palette_az1uball_data *data = dev->data;
    int ret;

    LOG_INF("AZ1UBALL driver initializing");

    data->dev = dev;
    data->sw_pressed_prev = false;

    /* Check if the I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

    /* Read and log the chip ID */
    uint8_t chip_id_l, chip_id_h;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_CHIP_ID_L, &chip_id_l);
    if (ret) {
        LOG_ERR("Failed to read chip ID low byte");
        return ret;
    }

    ret = i2c_reg_read_byte_dt(&config->i2c, REG_CHIP_ID_H, &chip_id_h);
    if (ret) {
            LOG_ERR("Failed to read chip ID high byte");
            return ret;
        }

        uint16_t chip_id = ((uint16_t)chip_id_h << 8) | chip_id_l;
    LOG_INF("AZ1UBALL chip ID: 0x%04X", chip_id);

    /* Enable the Trackball */
    ret = palette_az1uball_enable(dev);
    if (ret) {
        LOG_ERR("Failed to enable AZ1UBALL");
        return ret;
    }

    k_work_init(&data->irq_work, palette_az1uball_work_handler);

    LOG_INF("AZ1UBALL driver initialized");

    return 0;
}

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
    struct k_timer automouse_layer_timer;
    static bool automouse_triggered = false;

    static void activate_automouse_layer() {
        automouse_triggered = true;
        zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
        k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_ZMK_AZ1UBALL_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
    }

    static void deactivate_automouse_layer(struct k_timer *timer) {
        automouse_triggered = false;
        zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
    }

    K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

static const struct palette_az1uball_config palette_az1uball_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
};

static struct palette_az1uball_data palette_az1uball_data;

/* Device initialization macro */
DEVICE_DT_INST_DEFINE(0, palette_az1uball_init, NULL, &palette_az1uball_data, &palette_az1uball_config,
                      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);