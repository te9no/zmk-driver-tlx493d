#ifndef ZEPHYR_DRIVERS_SENSOR_TLX493D_H_
#define ZEPHYR_DRIVERS_SENSOR_TLX493D_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>

#define TLX493D_I2C_ADDR 0x5E

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    uint16_t evt_type;
    uint16_t x_input_code;
    uint16_t y_input_code;
    uint16_t z_input_code;
    uint16_t hysteresis;
    uint16_t center_threshold;
    uint16_t calibration_samples;
};

struct tlx493d_data {
    const struct device *dev;
    struct input_dev *input;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;
    float x_offset;
    float y_offset;
    float z_offset;
    float x_prev;
    float y_prev;
    float z_prev;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TLX493D_H_ */
