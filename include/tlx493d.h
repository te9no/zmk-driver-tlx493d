#ifndef ZEPHYR_DRIVERS_SENSOR_TLX493D_H_
#define ZEPHYR_DRIVERS_SENSOR_TLX493D_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    uint16_t x_input_code;
    uint16_t y_input_code;
    uint16_t center_threshold;
    uint16_t hysteresis;
};

struct tlx493d_data {
    struct input_dev *input;
    float x_offset;
    float y_offset;
    float z_offset;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TLX493D_H_ */
