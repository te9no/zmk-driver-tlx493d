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

struct tlx493d_data {
    const struct device *i2c_dev;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;
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
        return -EIO;
    }

    /* Convert raw data to 12-bit signed values */
    data->x = ((int16_t)buf[0] << 4) | (buf[4] >> 4);
    data->y = ((int16_t)buf[1] << 4) | (buf[4] & 0x0F);
    data->z = ((int16_t)buf[2] << 4) | (buf[5] >> 4);
    data->temp = (int16_t)buf[3];

    return 0;
}

static int tlx493d_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    if (chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    return tlx493d_read_data(dev);
}

static int tlx493d_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct tlx493d_data *data = dev->data;
    float value;

    switch (chan) {
    case SENSOR_CHAN_MAGN_X:
        value = data->x * TLX493D_CONV_XY;
        break;
    case SENSOR_CHAN_MAGN_Y:
        value = data->y * TLX493D_CONV_XY;
        break;
    case SENSOR_CHAN_MAGN_Z:
        value = data->z * TLX493D_CONV_Z;
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
    uint8_t id;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Read chip ID */
    if (i2c_reg_read_byte_dt(&config->i2c, TLX493D_REG_VERS, &id)) {
        LOG_ERR("Failed to read chip ID");
        return -EIO;
    }

    /* Configure sensor */
    if (i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_MOD1, TLX493D_MOD1_MASTER) ||
        i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_MOD2, TLX493D_MOD2_TEMP_EN)) {
        LOG_ERR("Failed to configure sensor");
        return -EIO;
    }

    return 0;
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
                         &tlx493d_config_##inst,                        \
                         POST_KERNEL,                                    \
                         CONFIG_SENSOR_INIT_PRIORITY,                    \
                         &tlx493d_api);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INIT)
