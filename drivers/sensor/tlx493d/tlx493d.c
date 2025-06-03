#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_SENSOR_LOG_LEVEL);

/* Register addresses */
#define TLV493D_REG_BX      0x00
#define TLV493D_REG_BY      0x01
#define TLV493D_REG_BZ      0x02
#define TLV493D_REG_TEMP    0x03
#define TLV493D_REG_BXYZ2   0x04
#define TLV493D_REG_MOD1    0x11
#define TLV493D_REG_MOD2    0x13

/* Configuration values from sample */
#define TLV493D_B_MULT      0.098f
#define TLV493D_TEMP_MULT   1.1f
#define TLV493D_TEMP_OFF    315

static K_THREAD_STACK_DEFINE(tlx493d_thread_stack, CONFIG_TLX493D_THREAD_STACK_SIZE);
static struct k_thread tlx493d_thread;

static void tlx493d_thread_main(void *p1, void *p2, void *p3)
{
    const struct device *dev = p1;
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t buf[7];
    int16_t x, y, z;
    float x_val, y_val, z_val;

    while (1) {
        /* Read sensor data */
        if (i2c_burst_read_dt(&config->i2c, TLV493D_REG_BX, buf, sizeof(buf)) == 0) {
            /* Convert raw data */
            x = ((int16_t)buf[0] << 4) | (buf[4] >> 4);
            y = ((int16_t)buf[1] << 4) | (buf[4] & 0x0F);
            z = ((int16_t)buf[2] << 4) | (buf[5] >> 4);

            /* Apply calibration and scaling */
            x_val = (x - data->x_offset) * TLV493D_B_MULT;
            y_val = (y - data->y_offset) * TLV493D_B_MULT;
            z_val = (z - data->z_offset) * TLV493D_B_MULT;

            /* Apply hysteresis and deadzone */
            if (fabs(x_val) > config->center_threshold / 1000.0f) {
                input_report_rel(data->input, config->x_input_code, 
                               (int)(x_val * 127.0f));
            }
            if (fabs(y_val) > config->center_threshold / 1000.0f) {
                input_report_rel(data->input, config->y_input_code, 
                               (int)(y_val * 127.0f));
            }

            input_sync(data->input);
        }

        k_msleep(10);  // 100Hz sampling rate
    }
}

static int tlx493d_init(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Register input device */
    data->input = input_device_register(dev->name, true);
    if (!data->input) {
        LOG_ERR("Failed to register input device");
        return -EINVAL;
    }

    /* Configure sensor in master controlled mode */
    uint8_t cfg[] = {
        0x00,  // Power down mode
        0x11   // Master controlled mode
    };

    if (i2c_write_dt(&config->i2c, cfg, sizeof(cfg))) {
        LOG_ERR("Failed to configure sensor");
        return -EIO;
    }

    /* Start sampling thread */
    k_thread_create(&tlx493d_thread,
                   tlx493d_thread_stack,
                   CONFIG_TLX493D_THREAD_STACK_SIZE,
                   tlx493d_thread_main,
                   (void *)dev, NULL, NULL,
                   CONFIG_TLX493D_THREAD_PRIORITY,
                   0, K_NO_WAIT);

    return 0;
}

/* Device instantiation */
#define TLX493D_INIT(inst)                                               \
    static struct tlx493d_data tlx493d_data_##inst;                     \
    static const struct tlx493d_config tlx493d_config_##inst = {        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                             \
        .x_input_code = DT_INST_PROP_OR(inst, x_input_code, 0),        \
        .y_input_code = DT_INST_PROP_OR(inst, y_input_code, 0),        \
        .center_threshold = DT_INST_PROP_OR(inst, center_threshold, 400), \
        .hysteresis = DT_INST_PROP_OR(inst, hysteresis, 100),          \
    };                                                                   \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                         tlx493d_init,                                   \
                         NULL,                                           \
                         &tlx493d_data_##inst,                          \
                         &tlx493d_config_##inst,                         \
                         POST_KERNEL,                                    \
                         CONFIG_INPUT_INIT_PRIORITY,                     \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INIT)
