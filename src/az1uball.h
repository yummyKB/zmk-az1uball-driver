#ifndef AZ1UBALL_H
#define AZ1UBALL_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/mutex.h>

/* Register Addresses */
#define REG_LEFT        0x04
#define REG_RIGHT       0x05
#define REG_UP          0x06
#define REG_DOWN        0x07
#define REG_SWITCH      0x08
#define REG_USER_FLASH  0xD0
#define REG_FLASH_PAGE  0xF0
#define REG_CHIP_ID_L   0xFA
#define REG_CHIP_ID_H   0xFB
#define REG_VERSION     0xFC
#define REG_I2C_ADDR    0xFD
#define REG_CTRL        0xFE

/* Bit Masks */
#define MSK_SWITCH_STATE    0b10000000

/* Sleep */
#define MSK_CTRL_SLEEP 0b00000001
#define MSK_CTRL_RESET 0b00000010

struct palette_az1uball_config {
    struct i2c_dt_spec i2c;
};

struct palette_az1uball_data {
    const struct device *dev;
    struct k_work_delayable periodic_work;
    struct k_work irq_work;     // Work item for handling interrupts
    struct k_mutex data_lock;   /* Existing mutex for data synchronization */
    struct k_mutex i2c_lock;    /* New mutex for I2C operations */
    float hue;
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

void az1uball_enable_sleep(const struct device *dev);
void az1uball_disable_sleep(const struct device *dev);
void az1uball_toggle_mode(void);


#endif /* AZ1UBALL_H */