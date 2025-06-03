#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_SENSOR_LOG_LEVEL);

/* Register addresses from sample code */
#define TLV493D_REG_B_X1      0x00    // X data MSB
#define TLV493D_REG_B_Y1      0x01    // Y data MSB
#define TLV493D_REG_B_Z1      0x02    // Z data MSB
#define TLV493D_REG_TEMP1     0x03    // Temperature data MSB
#define TLV493D_REG_B_X2      0x04    // X data LSB
#define TLV493D_REG_B_Y2      0x04    // Y data LSB (same as X2)
#define TLV493D_REG_B_Z2      0x05    // Z data LSB
#define TLV493D_REG_TEMP2     0x06    // Temperature data LSB
#define TLV493D_REG_FRAME     0x03    // Frame counter
/* Access and configuration registers */
#define TLV493D_REG_ACCESS    0x00    // Access control register
#define TLV493D_REG_CONFIG    0x10    // Configuration register
#define TLV493D_REG_MOD1      0x11    // Mode1 register
#define TLV493D_REG_MOD2      0x13    // Mode2 register

/* Configuration values */
#define TLV493D_ACCESS_MASTER 0x11    // Master controlled mode access value
#define TLV493D_CONFIG_MASTER 0x00    // Master controlled mode config
#define TLV493D_MOD1_MASTER   0x20    // Master controlled mode MOD1 value
#define TLV493D_MOD2_TEMP_EN  0x80    // Temperature measurement enable

/* Calibration settings */
#define TLX493D_CAL_SAMPLES    300
#define TLX493D_HYSTERESIS     0.1f    // 10% hysteresis
#define TLX493D_CENTER_THRESH  0.4f    // Center deadzone threshold

/* Event types */
#define EVT_TYPE_DEFAULT    0
#define EVT_TYPE_ABSOLUTE   1
#define EVT_TYPE_RELATIVE   2

struct tlx493d_data {
    const struct device *i2c_dev;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;
    /* Calibration offsets */
    float x_offset;
    float y_offset;
    float z_offset;
    /* Previous values for hysteresis */
    float x_prev;
    float y_prev;
    float z_prev;
};

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    uint8_t evt_type;
    uint16_t x_input_code;
    uint16_t y_input_code;
    uint16_t z_input_code;
};

#define TLX493D_I2C_ADDR    0x5E
#define TLX493D_I2C_RETRIES 3    // 通信リトライ回数

static int tlx493d_read_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t buf[7];
    int ret;

    /* Wait for measurement to complete (from sample) */
    k_msleep(TLV493D_MEASUREMENT_DELAY);

    ret = i2c_burst_read_dt(&config->i2c, TLV493D_REG_B_X1, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data");
        return ret;
    }

    /* Convert raw data using sample code format */
    data->x = ((int16_t)buf[0] << 8) | ((buf[4] & 0xF0) >> 4);
    data->y = ((int16_t)buf[1] << 8) | (buf[4] & 0x0F);
    data->z = ((int16_t)buf[2] << 8) | ((buf[5] & 0x0F));
    
    /* Apply conversion factor from sample */
    data->x = (float)data->x * TLV493D_B_MULT;
    data->y = (float)data->y * TLV493D_B_MULT;
    data->z = (float)data->z * TLV493D_B_MULT;

    LOG_DBG("Raw data - X: %.2f, Y: %.2f, Z: %.2f", 
            (double)data->x, (double)data->y, (double)data->z);
    return 0;
}

static int tlx493d_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    if (chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    return tlx493d_read_data(dev);
}

static int tlx493d_calibrate(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    float x_sum = 0, y_sum = 0, z_sum = 0;
    int ret;

    LOG_INF("Starting calibration process (%d samples)", TLX493D_CAL_SAMPLES);

    /* Collect samples for calibration */
    for (int i = 0; i < TLX493D_CAL_SAMPLES; i++) {
        ret = tlx493d_read_data(dev);
        if (ret != 0) {
            LOG_ERR("Calibration failed at sample %d", i);
            return ret;
        }
        x_sum += data->x;
        y_sum += data->y;
        z_sum += data->z;
        k_msleep(10);
        
        if (i % 50 == 0) {  // Log progress every 50 samples
            LOG_INF("Calibration progress: %d%%", (i * 100) / TLX493D_CAL_SAMPLES);
        }
    }

    /* Calculate offsets */
    data->x_offset = x_sum / TLX493D_CAL_SAMPLES;
    data->y_offset = y_sum / TLX493D_CAL_SAMPLES;
    data->z_offset = z_sum / TLX493D_CAL_SAMPLES;

    LOG_INF("Calibration complete - Offsets X: %.2f, Y: %.2f, Z: %.2f",
            (double)data->x_offset, (double)data->y_offset, (double)data->z_offset);

    return 0;
}

static float apply_hysteresis(float current, float previous, float threshold)
{
    if (fabsf(current - previous) < threshold) {
        return previous;
    }
    return current;
}

static int tlx493d_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct tlx493d_data *data = dev->data;
    float value, calibrated_value;

    switch (chan) {
    case SENSOR_CHAN_MAGN_X:
        calibrated_value = (data->x - data->x_offset) * TLV493D_B_MULT;
        value = apply_hysteresis(calibrated_value, data->x_prev, TLX493D_HYSTERESIS);
        data->x_prev = value;
        if (fabsf(value) < TLX493D_CENTER_THRESH) {
            value = 0;
        }
        break;
    case SENSOR_CHAN_MAGN_Y:
        calibrated_value = (data->y - data->y_offset) * TLV493D_B_MULT;
        value = apply_hysteresis(calibrated_value, data->y_prev, TLX493D_HYSTERESIS);
        data->y_prev = value;
        if (fabsf(value) < TLX493D_CENTER_THRESH) {
            value = 0;
        }
        break;
    case SENSOR_CHAN_MAGN_Z:
        calibrated_value = (data->z - data->z_offset) * TLV493D_B_MULT;
        value = apply_hysteresis(calibrated_value, data->z_prev, TLX493D_HYSTERESIS);
        data->z_prev = value;
        if (fabsf(value) < TLX493D_CENTER_THRESH) {
            value = 0;
        }
        break;
    case SENSOR_CHAN_DIE_TEMP:
        value = (data->temp * TLV493D_T_MULT) + TLV493D_T_OFFSET;
        break;
    default:
        return -ENOTSUP;
    }

    return sensor_value_from_float(val, value);
}

static int tlx493d_i2c_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct tlx493d_config *config = dev->config;
    int ret;

    for (int i = 0; i < TLX493D_I2C_RETRIES; i++) {
        ret = i2c_reg_read_byte_dt(&config->i2c, reg, val);
        if (ret == 0) {
            LOG_DBG("I2C read success - reg: 0x%02x, val: 0x%02x", reg, *val);
            return 0;
        }
        LOG_ERR("I2C read failed (attempt %d) - reg: 0x%02x, err: %d", i + 1, reg, ret);
        k_msleep(10);
    }
    return -EIO;
}

/* Timer settings */
#define LOG_INTERVAL_MS 1000
static struct k_work_delayable sensor_timer;

static void sensor_timer_handler(struct k_work *work)
{
    const struct device *dev = DEVICE_DT_GET_ANY(infineon_tlx493d);
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    struct sensor_value val;
    
    if (tlx493d_read_data(dev) == 0) {
        tlx493d_channel_get(dev, SENSOR_CHAN_MAGN_X, &val);
        float x = sensor_value_to_double(&val);
        tlx493d_channel_get(dev, SENSOR_CHAN_MAGN_Y, &val);
        float y = sensor_value_to_double(&val);
        tlx493d_channel_get(dev, SENSOR_CHAN_MAGN_Z, &val);
        float z = sensor_value_to_double(&val);
        tlx493d_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &val);
        float t = sensor_value_to_double(&val);

        // 各軸の入力コードに応じた処理
        if (config->x_input_code) {
            LOG_INF("X[%d]: %d.%03d", config->x_input_code, 
                    (int)x, (int)(x * 1000) % 1000);
        }
        if (config->y_input_code) {
            LOG_INF("Y[%d]: %d.%03d", config->y_input_code,
                    (int)y, (int)(y * 1000) % 1000);
        }
        if (config->z_input_code) {
            LOG_INF("Z[%d]: %d.%03d", config->z_input_code,
                    (int)z, (int)(z * 1000) % 1000);
        }

        // 従来のログ出力は evt_type に応じて表示
        switch (config->evt_type) {
        case EVT_TYPE_ABSOLUTE:
            LOG_INF("ABS: X: %d.%03d Y: %d.%03d Z: %d.%03d T: %d.%d",
                    (int)x, (int)(x * 1000) % 1000,
                    (int)y, (int)(y * 1000) % 1000,
                    (int)z, (int)(z * 1000) % 1000,
                    (int)t, (int)(t * 10) % 10);
            break;
        case EVT_TYPE_RELATIVE:
            LOG_INF("REL: X: %+d.%03d Y: %+d.%03d Z: %+d.%03d T: %d.%d",
                    (int)x, (int)(x * 1000) % 1000,
                    (int)y, (int)(y * 1000) % 1000,
                    (int)z, (int)(z * 1000) % 1000,
                    (int)t, (int)(t * 10) % 10);
            break;
        default:
            LOG_INF("Sensor values - X: %d.%03d mT, Y: %d.%03d mT, Z: %d.%03d mT, Temp: %d.%d C",
                    (int)x, (int)(x * 1000) % 1000,
                    (int)y, (int)(y * 1000) % 1000,
                    (int)z, (int)(z * 1000) % 1000,
                    (int)t, (int)(t * 10) % 10);
        }
    } else {
        LOG_ERR("Failed to read sensor data");
    }

    // Schedule next update
    k_work_schedule(&sensor_timer, K_MSEC(LOG_INTERVAL_MS));
}

static int tlx493d_init(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;

    LOG_INF("Initializing TLX493D sensor...");

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Initial delay */
    k_msleep(100);

    /* Set access mode */
    if (i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_ACCESS, TLV493D_ACCESS_MASTER)) {
        LOG_ERR("Failed to set access mode");
        return -EIO;
    }
    k_msleep(10);

    /* Set master controlled mode */
    if (i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_CONFIG, TLV493D_CONFIG_MASTER)) {
        LOG_ERR("Failed to set master controlled mode");
        return -EIO;
    }
    k_msleep(10);

    /* Enable temperature measurement */
    if (i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_MOD2, TLV493D_MOD2_TEMP_EN)) {
        LOG_ERR("Failed to enable temperature measurement");
        return -EIO;
    }

    /* Wait for configuration to take effect */
    k_msleep(50);

    /* Initialize state */
    data->x_prev = 0;
    data->y_prev = 0;
    data->z_prev = 0;

    /* Perform initial calibration */
    return tlx493d_calibrate(dev);
}

static const struct sensor_driver_api tlx493d_api = {
    .sample_fetch = tlx493d_sample_fetch,
    .channel_get = tlx493d_channel_get,
};

#define TLX493D_INIT(inst)                                               \
    static struct tlx493d_data tlx493d_data_##inst;                     \
    static const struct tlx493d_config tlx493d_config_##inst = {        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                             \
        .evt_type = DT_INST_PROP_OR(inst, evt_type, EVT_TYPE_DEFAULT), \
        .x_input_code = DT_INST_PROP_OR(inst, x_input_code, 0),        \
        .y_input_code = DT_INST_PROP_OR(inst, y_input_code, 0),        \
        .z_input_code = DT_INST_PROP_OR(inst, z_input_code, 0),        \
    };                                                                   \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                         tlx493d_init,                                   \
                         NULL,                                           \
                         &tlx493d_data_##inst,                          \
                         &tlx493d_config_##inst,                         \
                         POST_KERNEL,                                    \
                         CONFIG_SENSOR_INIT_PRIORITY,                    \
                         &tlx493d_api);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INIT)
