#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/mutex.h>

/* Bit Masks */
#define MSK_SWITCH_STATE    0b10000000

/* Mode definitions */
enum az1uball_mode {
    AZ1UBALL_MODE_MOUSE,
    AZ1UBALL_MODE_SCROLL
};

struct az1uball_config {
    struct i2c_dt_spec i2c;
    const char *default_mode;
    const char *sensitivity;

    bool flip_x;
    bool flip_y;
};

struct az1uball_data {
    const struct device *dev;
    struct k_work work;
    struct k_timer polling_timer;
    struct k_mutex data_lock;
    bool sw_pressed;
    bool sw_pressed_prev;
    atomic_t x_buffer;
    atomic_t y_buffer;
    uint32_t last_interrupt_time;
    uint32_t previous_interrupt_time;
    int previous_x;
    int previous_y;
    int smoothed_x;
    int smoothed_y;
    enum az1uball_mode current_mode;
    uint32_t last_activity_time;    // 最後の入力があった時間
    bool is_low_power_mode;         // 省電力モードフラグ
};

/* Public API */
void az1uball_toggle_mode(void);
