#ifndef ZEPHYR_DRIVERS_SENSOR_TLX493D_H_
#define ZEPHYR_DRIVERS_SENSOR_TLX493D_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#define TLX493D_I2C_ADDR 0x0C

struct tlx493d_config {
    struct i2c_dt_spec i2c;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TLX493D_H_ */
