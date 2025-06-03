#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_SENSOR_LOG_LEVEL);

/* Register definitions from TLx493D_W2BW.c */
#define TLV493D_R_BX1       0x00
#define TLV493D_R_BX2       0x04
#define TLV493D_R_BY1       0x01
#define TLV493D_R_BY2       0x04
#define TLV493D_R_BZ1       0x02
#define TLV493D_R_BZ2       0x05
#define TLV493D_R_TEMP1     0x03
#define TLV493D_R_TEMP2     0x06

/* Access registers */
#define TLV493D_W_ADDR      0x00
#define TLV493D_W_MOD1      0x11
#define TLV493D_W_MOD2      0x13
#define TLV493D_W_TEST      0x14

/* Configuration values */
#define TLV493D_MOD1_INT        0x04
#define TLV493D_MOD1_FAST       0x02
#define TLV493D_MOD1_LOW        0x01
#define TLV493D_MOD2_T_EN       0x80
#define TLV493D_MOD2_LP_PERIOD  0x40

static K_THREAD_STACK_DEFINE(tlx493d_thread_stack, CONFIG_TLX493D_THREAD_STACK_SIZE);
static struct k_thread tlx493d_thread;

static int tlx493d_read_regs(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t buf[7];
    int ret;

    ret = i2c_burst_read_dt(&config->i2c, TLV493D_R_BX1, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    /* Convert using official library format */
    data->x = ((int16_t)buf[0] << 8) | ((buf[4] & 0xF0) << 0);
    data->y = ((int16_t)buf[1] << 8) | ((buf[4] & 0x0F) << 4);
    data->z = ((int16_t)buf[2] << 8) | ((buf[5] & 0x0F) << 4);
    data->temp = ((int16_t)buf[3] << 8) | ((buf[6] & 0xF0) << 0);

    return 0;
}

static void tlx493d_thread_main(void *p1, void *p2, void *p3)
{
    const struct device *dev = p1;
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    float x_val, y_val, z_val;

    while (1) {
        if (tlx493d_read_regs(dev) == 0) {
            /* Convert to mT using official conversion factors */
            x_val = data->x * 0.098f;
            y_val = data->y * 0.098f;
            z_val = data->z * 0.098f;

            /* Apply calibration offsets */
            x_val -= data->x_offset;
            y_val -= data->y_offset;
            z_val -= data->z_offset;

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
        k_msleep(10);
    }
}

static int tlx493d_init(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    int ret;

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

    /* Initial configuration sequence from TLx493D_W2BW.c */
    const uint8_t init_seq[] = {
        0x00,  // Reset
        0x00, 
        0x00,
    };
    ret = i2c_write_dt(&config->i2c, init_seq, sizeof(init_seq));
    if (ret < 0) {
        LOG_ERR("Failed to reset sensor");
        return ret;
    }
    k_msleep(40);  // Wait for reset

    /* Configure master controlled mode */
    const uint8_t config_seq[] = {
        TLV493D_MOD1_FAST,                    // Fast mode
        0x00,                                  // Reserved
        TLV493D_MOD2_T_EN | TLV493D_MOD2_LP_PERIOD  // Enable temp + LP period
    };
    ret = i2c_write_dt(&config->i2c, config_seq, sizeof(config_seq));
    if (ret < 0) {
        LOG_ERR("Failed to configure sensor");
        return ret;
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
