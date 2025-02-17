#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/mutex.h>

/* I2C address */
#define AZ1UBALL_I2C_ADDRESS  0x0A  // AZ1UBALLのI2Cアドレス（例）

/* Register Addresses */
#define REG_LEFT        0x04
#define REG_RIGHT       0x05
#define REG_UP          0x06
#define REG_DOWN        0x07
#define REG_SWITCH      0x08

/* Bit Masks */
#define MSK_SWITCH_STATE    0b10000000

struct az1uball_config {
    struct i2c_dt_spec i2c;
};

struct az1uball_data {
    const struct device *dev;
    struct k_work work;
    struct k_work_delayable periodic_work;
    struct k_timer polling_timer
    int16_t x;
    int16_t y;
    uint8_t buttons;
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
};
