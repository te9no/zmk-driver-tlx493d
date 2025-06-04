#ifndef ZEPHYR_DRIVERS_SENSOR_TLX493D_H_
#define ZEPHYR_DRIVERS_SENSOR_TLX493D_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>

/* Register addresses */
#define TLV493D_R_BX1       0x00
#define TLV493D_R_BX2       0x04
#define TLV493D_R_BY1       0x01
#define TLV493D_R_BY2       0x04
#define TLV493D_R_BZ1       0x02
#define TLV493D_R_BZ2       0x05
#define TLV493D_R_TEMP1     0x03
#define TLV493D_R_TEMP2     0x06
#define TLV493D_W_ADDR      0x00
#define TLV493D_W_MOD1      0x11
#define TLV493D_W_MOD2      0x13

/* Operation modes */
#define TLV493D_MODE_POWER_DOWN    0x00
#define TLV493D_MODE_MCM           0x11
#define TLV493D_MODE_FAST          0x02
#define TLV493D_MODE_LOW_POWER     0x04

/* Configuration values */
#define TLV493D_CONFIG_TEMP_EN     0x80
#define TLV493D_CONFIG_INT_EN      0x04
#define TLV493D_CONFIG_FAST        0x02
#define TLV493D_CONFIG_LP          0x01

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    uint16_t evt_type;
    uint16_t x_input_code;
    uint16_t y_input_code;
    uint16_t z_input_code;
    uint16_t hysteresis;
    uint16_t center_threshold;
    uint16_t calibration_samples;
    uint16_t polling_interval_ms;
};

struct tlx493d_data {
    const struct device *dev;
    struct input_dev *input;
    struct k_work_delayable work;
    
    /* Sensor readings */
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t temp_raw;
    
    /* Calibration data */
    float x_offset;
    float y_offset;
    float z_offset;
    
    /* Previous values for hysteresis */
    float x_prev;
    float y_prev;
    float z_prev;
    
    /* Sensor state */
    uint8_t mode;
    bool temp_enabled;
    bool initialized;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TLX493D_H_ */
