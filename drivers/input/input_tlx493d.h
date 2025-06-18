/**
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_INPUT_TLX493D_H_
#define ZEPHYR_DRIVERS_INPUT_TLX493D_H_

#include <zephyr/types.h>

#define TLX493D_REG_BX_MSB              0x00
#define TLX493D_REG_BX_LSB              0x01
#define TLX493D_REG_BY_MSB              0x02
#define TLX493D_REG_BY_LSB              0x03
#define TLX493D_REG_BZ_MSB              0x04
#define TLX493D_REG_BZ_LSB              0x05
#define TLX493D_REG_TEMP_MSB            0x06
#define TLX493D_REG_TEMP_LSB            0x07
#define TLX493D_REG_CONFIG              0x08
#define TLX493D_REG_MOD1                0x09
#define TLX493D_REG_MOD2                0x0A
#define TLX493D_REG_RESERVED            0x0B
#define TLX493D_REG_DIAG                0x0C
#define TLX493D_REG_XL_MSB              0x0D
#define TLX493D_REG_XL_LSB              0x0E
#define TLX493D_REG_XH_MSB              0x0F
#define TLX493D_REG_XH_LSB              0x10
#define TLX493D_REG_YL_MSB              0x11
#define TLX493D_REG_YL_LSB              0x12
#define TLX493D_REG_YH_MSB              0x13
#define TLX493D_REG_YH_LSB              0x14
#define TLX493D_REG_ZL_MSB              0x15
#define TLX493D_REG_ZL_LSB              0x16
#define TLX493D_REG_ZH_MSB              0x17
#define TLX493D_REG_ZH_LSB              0x18

#define TLX493D_REG_FACTORY_START       0x07
#define TLX493D_FACTORY_DATA_SIZE       3

#define TLX493D_CONFIG_DEFAULT          0x00
#define TLX493D_CONFIG_MEASUREMENT_X    0x01
#define TLX493D_CONFIG_MEASUREMENT_Y    0x02
#define TLX493D_CONFIG_MEASUREMENT_Z    0x04
#define TLX493D_CONFIG_MEASUREMENT_T    0x08
#define TLX493D_CONFIG_MEASUREMENT_ALL  0x0F

#define TLX493D_MOD1_DEFAULT            0x00
#define TLX493D_MOD1_FAST               0x01
#define TLX493D_MOD1_LOW_POWER          0x02
#define TLX493D_MOD1_SLEEP              0x04
#define TLX493D_MOD1_INT_ENABLE         0x08
#define TLX493D_MOD1_COLLISION_AVOID    0x10
#define TLX493D_MOD1_1BYTE_READ         0x20
#define TLX493D_MOD1_ADDR_A0            0x40
#define TLX493D_MOD1_ADDR_A1            0x80

#define TLX493D_MOD2_DEFAULT            0x00
#define TLX493D_MOD2_LP_PERIOD_100MS    0x00
#define TLX493D_MOD2_LP_PERIOD_12MS     0x01
#define TLX493D_MOD2_TRIGGER_READ       0x02
#define TLX493D_MOD2_SHORT_RANGE        0x04
#define TLX493D_MOD2_TEMPERATURE_COMP   0x08

#define TLX493D_DIAG_VALID_DATA         0x01
#define TLX493D_DIAG_FRAME_COUNTER      0x02
#define TLX493D_DIAG_CHANNEL            0x04
#define TLX493D_DIAG_FUSE_PARITY        0x08
#define TLX493D_DIAG_BUS_PARITY         0x10
#define TLX493D_DIAG_ADC_TEST           0x20
#define TLX493D_DIAG_POWER_DOWN         0x40

#define TLX493D_I2C_ADDR_DEFAULT        0x35
#define TLX493D_I2C_ADDR_ALT1           0x22
#define TLX493D_I2C_ADDR_ALT2           0x78
#define TLX493D_I2C_ADDR_ALT3           0x44

#define TLX493D_DATA_SIZE               6
#define TLX493D_REG_MAP_SIZE            25

/* Status and diagnostic bits */
#define TLX493D_DIAG_T_BIT              0x01

#define TLX493D_POWER_CYCLE_DELAY_MS    10
#define TLX493D_RECOVERY_DELAY_MS       10
#define TLX493D_RESET_DELAY_MS          10
#define TLX493D_WAKEUP_DELAY_MS         10

#define TLX493D_CALIBRATION_SAMPLES     100
#define TLX493D_MIN_CALIBRATION_SAMPLES 50
#define TLX493D_CALIBRATION_SAMPLE_DELAY_MS  5

#define TLX493D_MAX_ERROR_COUNT         5
#define TLX493D_MAX_RECOVERY_ATTEMPTS   5
#define MAX_CONSECUTIVE_ERRORS          5

#define AUTO_RECALIBRATION_TIMEOUT_MS   30000
#define HYSTERESIS_THRESHOLD            20

#define TLX493D_DEAD_ZONE_XY            3
#define TLX493D_DEAD_ZONE_Z             5
#define TLX493D_SCALE_FACTOR            10

#ifndef CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS
#define CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS    10
#endif

#ifndef CONFIG_INPUT_TLX493D_Z_THRESHOLD
#define CONFIG_INPUT_TLX493D_Z_THRESHOLD            5
#endif

#ifndef CONFIG_INPUT_TLX493D_ROTATION_SCALER
#define CONFIG_INPUT_TLX493D_ROTATION_SCALER        20
#endif

#endif /* ZEPHYR_DRIVERS_INPUT_TLX493D_H_ */