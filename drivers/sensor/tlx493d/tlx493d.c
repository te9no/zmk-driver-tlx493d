#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_SENSOR_LOG_LEVEL);

/* Register addresses */
#define TLX493D_REG_BX       0x00
#define TLX493D_REG_BY       0x01
#define TLX493D_REG_BZ       0x02
#define TLX493D_REG_TEMP     0x03
#define TLX493D_REG_BX2      0x04
#define TLX493D_REG_FACTSET1 0x05
#define TLX493D_REG_FACTSET2 0x06
#define TLX493D_REG_FACTSET3 0x07
#define TLX493D_REG_MOD1     0x11
#define TLX493D_REG_VERS     0x0C
#define TLX493D_REG_MOD2     0x13

/* Configuration values */
#define TLX493D_MOD1_DEFAULT     0x20
#define TLX493D_MOD1_LOW_POWER   0x00
#define TLX493D_MOD1_MASTER      0x20
#define TLX493D_MOD2_TEMP_EN     0x80

/* Conversion factors */
#define TLX493D_CONV_XY          0.13f
#define TLX493D_CONV_Z           0.13f
#define TLX493D_CONV_TEMP        0.24f

/* Calibration settings */
#define TLX493D_CAL_SAMPLES    300
#define TLX493D_HYSTERESIS     0.1f    // 10% hysteresis
#define TLX493D_CENTER_THRESH  0.4f    // Center deadzone threshold

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
};

static int tlx493d_read_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t buf[7];

    if (i2c_burst_read_dt(&config->i2c, TLX493D_REG_BX, buf, sizeof(buf))) {
        LOG_ERR("Failed to read sensor data");
        return -EIO;
    }

    /* Convert raw data to 12-bit signed values */
    data->x = ((int16_t)buf[0] << 4) | (buf[4] >> 4);
    data->y = ((int16_t)buf[1] << 4) | (buf[4] & 0x0F);
    data->z = ((int16_t)buf[2] << 4) | (buf[5] >> 4);
    data->temp = (int16_t)buf[3];

    LOG_DBG("Raw data - X: %d, Y: %d, Z: %d", data->x, data->y, data->z);
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
    if (fabs(current - previous) < threshold) {
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
        calibrated_value = (data->x - data->x_offset) * TLX493D_CONV_XY;
        value = apply_hysteresis(calibrated_value, data->x_prev, TLX493D_HYSTERESIS);
        data->x_prev = value;
        if (fabs(value) < TLX493D_CENTER_THRESH) {
            value = 0;
        }
        break;
    case SENSOR_CHAN_MAGN_Y:
        calibrated_value = (data->y - data->y_offset) * TLX493D_CONV_XY;
        value = apply_hysteresis(calibrated_value, data->y_prev, TLX493D_HYSTERESIS);
        data->y_prev = value;
        if (fabs(value) < TLX493D_CENTER_THRESH) {
            value = 0;
        }
        break;
    case SENSOR_CHAN_MAGN_Z:
        calibrated_value = (data->z - data->z_offset) * TLX493D_CONV_Z;
        value = apply_hysteresis(calibrated_value, data->z_prev, TLX493D_HYSTERESIS);
        data->z_prev = value;
        if (fabs(value) < TLX493D_CENTER_THRESH) {
            value = 0;
        }
        break;
    case SENSOR_CHAN_DIE_TEMP:
        value = data->temp * TLX493D_CONV_TEMP;
        break;
    default:
        return -ENOTSUP;
    }

    return sensor_value_from_float(val, value);
}

static int tlx493d_init(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    uint8_t id, mod1, mod2;

    LOG_INF("Initializing TLX493D sensor...");

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    /* Read and verify chip ID */
    if (i2c_reg_read_byte_dt(&config->i2c, TLX493D_REG_VERS, &id)) {
        LOG_ERR("Failed to read chip ID");
        return -EIO;
    }
    LOG_INF("Chip ID: 0x%02x", id);

    /* Initial delay for sensor stabilization */
    k_msleep(10);

    /* Configure sensor */
    if (i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_MOD1, TLX493D_MOD1_MASTER)) {
        LOG_ERR("Failed to write MOD1 register");
        return -EIO;
    }

    if (i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_MOD2, TLX493D_MOD2_TEMP_EN)) {
        LOG_ERR("Failed to write MOD2 register");
        return -EIO;
    }

    /* Verify configuration */
    if (i2c_reg_read_byte_dt(&config->i2c, TLX493D_REG_MOD1, &mod1) ||
        i2c_reg_read_byte_dt(&config->i2c, TLX493D_REG_MOD2, &mod2)) {
        LOG_ERR("Failed to verify configuration");
        return -EIO;
    }
    LOG_INF("Configuration verified - MOD1: 0x%02x, MOD2: 0x%02x", mod1, mod2);

    /* Initialize state */
    data->x_prev = 0;
    data->y_prev = 0;
    data->z_prev = 0;

    /* Wait for sensor to stabilize */
    k_msleep(50);

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
