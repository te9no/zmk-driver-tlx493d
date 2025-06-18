/**
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <errno.h>
#include <math.h>

#include "input_tlx493d.h"

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_TLX493D_LOG_LEVEL);

/* TLX493D sensor registers - based on Infineon library */
typedef struct {
	uint8_t address;
	uint8_t mask;
	uint8_t offset;
} tlx493d_register_t;

/* TLX493D sensor structure - simplified from Infineon library */
typedef struct {
	uint8_t reg_map[TLX493D_REG_MAP_SIZE];
	bool initialized;
	uint8_t i2c_address;
} tlx493d_sensor_t;

struct tlx493d_config {
	struct i2c_dt_spec i2c;
	uint32_t polling_interval_ms;
};

struct tlx493d_data {
	struct k_work_delayable work;
	const struct device *dev;
	struct k_sem lock;
	
	/* Infineon library based sensor */
	tlx493d_sensor_t sensor;
	
	/* Sensor state */
	uint32_t error_count;
	uint32_t consecutive_errors;
	bool in_recovery;
	
	/* Calibration data */
	int16_t offset_x;
	int16_t offset_y;
	int16_t offset_z;
	
	/* Previous values for relative calculation */
	int16_t prev_x;
	int16_t prev_y;
	int16_t prev_z;
	
	/* Auto-calibration */
	uint32_t last_movement_time;
	bool auto_cal_needed;
	
	/* Hysteresis */
	int16_t last_reported_x;
	int16_t last_reported_y;
	int16_t last_reported_z;
};

/* Forward declarations */
static int tlx493d_calibrate(const struct device *dev);

/* Infineon library inspired register definitions for TLV493D-A1B6 */
static const tlx493d_register_t tlx493d_regs[] = {
	/* Measurement registers */
	{TLX493D_REG_BX_MSB, 0xFF, 0},    /* BX[11:4] */
	{TLX493D_REG_BY_MSB, 0xFF, 0},    /* BY[11:4] */
	{TLX493D_REG_BZ_MSB, 0xFF, 0},    /* BZ[11:4] */
	{TLX493D_REG_TEMP_MSB, 0xF0, 4},  /* TEMP[11:8] */
	{TLX493D_REG_BX_LSB, 0xF0, 4},    /* BX[3:0] */
	{TLX493D_REG_BZ_LSB, 0x0F, 0},    /* BZ[3:0] */
	{TLX493D_REG_BY_LSB, 0x0F, 0},    /* BY[3:0] */
	{TLX493D_REG_TEMP_LSB, 0x0F, 0},  /* TEMP[7:4] */
};

/* Function to write a single register */
static int tlx493d_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct tlx493d_config *cfg = dev->config;
	uint8_t tx_buf[2] = {reg, value};
	
	return i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
}

/* Function to read multiple registers */
static int tlx493d_read_regs(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
	const struct tlx493d_config *cfg = dev->config;
	
	return i2c_write_read_dt(&cfg->i2c, &reg, 1, data, len);
}

/* Infineon library inspired functions */
static bool tlx493d_has_not_only_0xff(const uint8_t *reg_map, uint8_t size)
{
	/* Skip address 0 - seems to be ok even if other entries are not */
	for (uint8_t i = 1; i < size; ++i) {
		if (reg_map[i] != 0xFF) {
			return true;
		}
	}
	return false;
}

static int tlx493d_read_registers(const struct device *dev, tlx493d_sensor_t *sensor)
{
	int ret = tlx493d_read_regs(dev, 0x00, sensor->reg_map, TLX493D_REG_MAP_SIZE);
	if (ret != 0) {
		return ret;
	}
	
	if (!tlx493d_has_not_only_0xff(sensor->reg_map, TLX493D_REG_MAP_SIZE)) {
		return -EIO;
	}
	
	return 0;
}

static void tlx493d_calculate_raw_magnetic_field(const tlx493d_sensor_t *sensor, 
						 int16_t *x, int16_t *y, int16_t *z)
{
	const uint8_t *reg_map = sensor->reg_map;
	
	/* Extract 12-bit values from registers based on Infineon library logic */
	*x = ((int16_t)reg_map[TLX493D_REG_BX_MSB] << 4) | 
	     ((reg_map[TLX493D_REG_BX_LSB] & 0xF0) >> 4);
	     
	*y = ((int16_t)reg_map[TLX493D_REG_BY_MSB] << 4) | 
	     (reg_map[TLX493D_REG_BY_LSB] & 0x0F);
	     
	*z = ((int16_t)reg_map[TLX493D_REG_BZ_MSB] << 4) | 
	     (reg_map[TLX493D_REG_BZ_LSB] & 0x0F);

	/* Sign extend 12-bit to 16-bit */
	if (*x & 0x800) *x |= 0xF000;
	if (*y & 0x800) *y |= 0xF000;
	if (*z & 0x800) *z |= 0xF000;
}

static bool tlx493d_has_valid_data(const tlx493d_sensor_t *sensor)
{
	/* Check if data is valid based on status bits */
	/* This is a simplified check - full implementation would check parity bits */
	return (sensor->reg_map[TLX493D_REG_DIAG] & TLX493D_DIAG_T_BIT) == 0;
}

/* I2C bus recovery using exponential backoff */
static int tlx493d_bus_recovery(const struct device *dev)
{
	const struct tlx493d_config *cfg = dev->config;
	int ret;
	
	LOG_DBG("Attempting I2C bus recovery");
	
	/* Send recovery frame (0xFF) - based on Infineon library */
	uint8_t recovery_frame = 0xFF;
	ret = i2c_write_dt(&cfg->i2c, &recovery_frame, 1);
	if (ret < 0) {
		LOG_WRN("Recovery frame failed: %d", ret);
	}
	
	k_msleep(10);
	
	/* Send reset command (0x00) - based on Infineon library */
	uint8_t reset_cmd = 0x00;
	ret = i2c_write_dt(&cfg->i2c, &reset_cmd, 1);
	if (ret < 0) {
		LOG_WRN("Reset command failed: %d", ret);
	}
	
	k_msleep(50);
	
	return 0;
}

/* Sensor initialization following Infineon library approach */
static int tlx493d_init_sensor(const struct device *dev)
{
	struct tlx493d_data *data = dev->data;
	const struct tlx493d_config *cfg = dev->config;
	int ret;
	
	LOG_DBG("Initializing TLX493D sensor using Infineon library approach");
	
	/* Initialize sensor based on Infineon library approach */
	data->sensor.i2c_address = cfg->i2c.addr;
	
	/* Send recovery frame (0xFF) */
	uint8_t recovery_frame = 0xFF;
	ret = i2c_write_dt(&cfg->i2c, &recovery_frame, 1);
	if (ret) {
		LOG_WRN("Recovery frame failed: %d", ret);
	}
	
	k_msleep(1);
	
	/* Send reset command (0x00) */
	uint8_t reset_cmd = 0x00;
	ret = i2c_write_dt(&cfg->i2c, &reset_cmd, 1);
	if (ret) {
		LOG_WRN("Reset command failed: %d", ret);
	}
	
	k_msleep(10);
	
	/* Read initial register map */
	ret = tlx493d_read_registers(dev, &data->sensor);
	if (ret) {
		LOG_ERR("Failed to read initial registers: %d", ret);
		return ret;
	}
	
	/* Configure default settings - 1-byte read mode, interrupt off */
	ret = tlx493d_write_reg(dev, TLX493D_REG_MOD1, TLX493D_MOD1_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to configure MOD1: %d", ret);
		return ret;
	}
	
	ret = tlx493d_write_reg(dev, TLX493D_REG_MOD2, TLX493D_MOD2_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to configure MOD2: %d", ret);
		return ret;
	}
	
	/* Perform initial calibration */
	ret = tlx493d_calibrate(dev);
	if (ret) {
		LOG_ERR("Initial calibration failed: %d", ret);
		return ret;
	}
	
	data->sensor.initialized = true;
	
	LOG_INF("TLX493D sensor initialized successfully");
	return 0;
}

/* Error recovery with exponential backoff */
static int tlx493d_error_recovery(const struct device *dev)
{
	struct tlx493d_data *data = dev->data;
	int ret;
	int retry_delay_ms = 50;
	
	if (data->in_recovery) {
		LOG_DBG("Recovery already in progress");
		return -EBUSY;
	}
	
	data->in_recovery = true;
	
	for (int i = 0; i < TLX493D_MAX_RECOVERY_ATTEMPTS; i++) {
		LOG_INF("Recovery attempt %d/%d", i + 1, TLX493D_MAX_RECOVERY_ATTEMPTS);
		
		ret = tlx493d_bus_recovery(dev);
		if (ret == 0) {
			ret = tlx493d_init_sensor(dev);
			if (ret == 0) {
				LOG_INF("Recovery successful on attempt %d", i + 1);
				data->in_recovery = false;
				data->error_count = 0;
				data->consecutive_errors = 0;
				return 0;
			}
		}
		
		/* Exponential backoff */
		k_msleep(retry_delay_ms);
		retry_delay_ms *= 2;
		if (retry_delay_ms > 1000) {
			retry_delay_ms = 1000;
		}
	}
	
	LOG_ERR("Recovery failed after %d attempts", TLX493D_MAX_RECOVERY_ATTEMPTS);
	data->in_recovery = false;
	return -EIO;
}

/* Calibration using moving average */
static int tlx493d_calibrate(const struct device *dev)
{
	struct tlx493d_data *data = dev->data;
	int32_t sum_x = 0, sum_y = 0, sum_z = 0;
	int16_t x, y, z;
	int ret;
	int valid_samples = 0;
	
	LOG_DBG("Starting sensor calibration");
	
	for (int i = 0; i < TLX493D_CALIBRATION_SAMPLES && valid_samples < TLX493D_CALIBRATION_SAMPLES; i++) {
		ret = tlx493d_read_registers(dev, &data->sensor);
		if (ret) {
			LOG_WRN("Calibration read %d failed: %d", i, ret);
			continue;
		}
		
		if (!tlx493d_has_valid_data(&data->sensor)) {
			LOG_WRN("Invalid data during calibration sample %d", i);
			continue;
		}
		
		tlx493d_calculate_raw_magnetic_field(&data->sensor, &x, &y, &z);
		
		sum_x += x;
		sum_y += y;
		sum_z += z;
		valid_samples++;
		
		k_msleep(TLX493D_CALIBRATION_SAMPLE_DELAY_MS);
	}
	
	if (valid_samples < TLX493D_MIN_CALIBRATION_SAMPLES) {
		LOG_ERR("Insufficient valid samples for calibration: %d", valid_samples);
		return -EIO;
	}
	
	data->offset_x = sum_x / valid_samples;
	data->offset_y = sum_y / valid_samples;
	data->offset_z = sum_z / valid_samples;
	
	LOG_INF("Calibration complete: offset X=%d, Y=%d, Z=%d (%d samples)", 
		data->offset_x, data->offset_y, data->offset_z, valid_samples);
	
	return 0;
}

/* Main work handler */
static void tlx493d_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tlx493d_data *data = CONTAINER_OF(dwork, struct tlx493d_data, work);
	const struct device *dev = data->dev;
	const struct tlx493d_config *cfg = dev->config;
	int16_t raw_x, raw_y, raw_z;
	int ret;
	
	k_sem_take(&data->lock, K_FOREVER);
	
	/* Read sensor registers using Infineon library approach */
	ret = tlx493d_read_registers(dev, &data->sensor);
	if (ret) {
		data->consecutive_errors++;
		data->error_count++;
		
		if (data->consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
			LOG_WRN("Too many consecutive errors (%u), triggering recovery", 
				data->consecutive_errors);
			ret = tlx493d_error_recovery(dev);
			if (ret) {
				LOG_ERR("Error recovery failed: %d", ret);
			}
			data->consecutive_errors = 0;
		}
		goto reschedule;
	}
	
	/* Check if data is valid */
	if (!tlx493d_has_valid_data(&data->sensor)) {
		LOG_DBG("Invalid sensor data");
		goto reschedule;
	}
	
	/* Calculate raw magnetic field using Infineon library method */
	tlx493d_calculate_raw_magnetic_field(&data->sensor, &raw_x, &raw_y, &raw_z);
	
	/* Reset consecutive error count on successful read */
	data->consecutive_errors = 0;
	
	/* Apply calibration offset */
	int16_t cal_x = raw_x - data->offset_x;
	int16_t cal_y = raw_y - data->offset_y;
	int16_t cal_z = raw_z - data->offset_z;
	
	/* Calculate relative movement from previous values */
	int16_t rel_x = cal_x - data->prev_x;
	int16_t rel_y = cal_y - data->prev_y;
	int16_t rel_z = cal_z - data->prev_z;
	
	/* Apply deadzone filtering */
	if (abs(rel_x) > TLX493D_DEAD_ZONE_XY) {
		rel_x /= TLX493D_SCALE_FACTOR;
	} else {
		rel_x = 0;
	}
	
	if (abs(rel_y) > TLX493D_DEAD_ZONE_XY) {
		rel_y /= TLX493D_SCALE_FACTOR;
	} else {
		rel_y = 0;
	}
	
	if (abs(rel_z) > TLX493D_DEAD_ZONE_Z) {
		rel_z /= CONFIG_INPUT_TLX493D_ROTATION_SCALER;
	} else {
		rel_z = 0;
	}
	
	/* Apply hysteresis to reduce jitter */
	bool movement_detected = false;
	
	if (abs(rel_x - data->last_reported_x) > HYSTERESIS_THRESHOLD) {
		data->last_reported_x = rel_x;
		movement_detected = true;
	} else {
		rel_x = 0;
	}
	
	if (abs(rel_y - data->last_reported_y) > HYSTERESIS_THRESHOLD) {
		data->last_reported_y = rel_y;
		movement_detected = true;
	} else {
		rel_y = 0;
	}
	
	if (abs(rel_z - data->last_reported_z) > HYSTERESIS_THRESHOLD) {
		data->last_reported_z = rel_z;
		movement_detected = true;
	} else {
		rel_z = 0;
	}
	
	/* Update previous values for next iteration */
	data->prev_x = cal_x;
	data->prev_y = cal_y;
	data->prev_z = cal_z;
	
	/* Update movement tracking for auto-calibration */
	if (movement_detected) {
		data->last_movement_time = k_uptime_get_32();
		data->auto_cal_needed = false;
	} else {
		uint32_t time_since_movement = k_uptime_get_32() - data->last_movement_time;
		if (time_since_movement > AUTO_RECALIBRATION_TIMEOUT_MS) {
			data->auto_cal_needed = true;
		}
	}
	
	/* Perform auto-calibration if needed */
	if (data->auto_cal_needed) {
		LOG_INF("Auto-calibration triggered after %u ms of no movement", 
			k_uptime_get_32() - data->last_movement_time);
		ret = tlx493d_calibrate(dev);
		if (ret == 0) {
			data->auto_cal_needed = false;
			data->last_movement_time = k_uptime_get_32();
		}
	}
	
	k_sem_give(&data->lock);
	
	/* Report input events */
	if (rel_x != 0) {
		input_report_rel(dev, INPUT_REL_X, rel_x, true, K_FOREVER);
	}
	if (rel_y != 0) {
		input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
	}
	if (abs(rel_z) >= CONFIG_INPUT_TLX493D_Z_THRESHOLD) {
		input_report_rel(dev, INPUT_REL_WHEEL, rel_z, true, K_FOREVER);
	}
	
	/* Schedule next work */
	k_work_schedule(&data->work, K_MSEC(cfg->polling_interval_ms));
	return;

reschedule:
	k_sem_give(&data->lock);
	k_work_schedule(&data->work, K_MSEC(cfg->polling_interval_ms));
}

/* Device initialization */
static int tlx493d_init(const struct device *dev)
{
	struct tlx493d_data *data = dev->data;
	const struct tlx493d_config *cfg = dev->config;
	int ret;
	
	LOG_INF("Initializing TLX493D device");
	
	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	
	k_sem_init(&data->lock, 1, 1);
	data->dev = dev;
	data->error_count = 0;
	data->consecutive_errors = 0;
	data->in_recovery = false;
	data->last_movement_time = k_uptime_get_32();
	data->auto_cal_needed = false;
	
	/* Initialize work */
	k_work_init_delayable(&data->work, tlx493d_work_handler);
	
	/* Initialize sensor hardware */
	ret = tlx493d_init_sensor(dev);
	if (ret) {
		LOG_ERR("Sensor initialization failed: %d", ret);
		return ret;
	}
	
	/* Start periodic work */
	k_work_schedule(&data->work, K_MSEC(cfg->polling_interval_ms));
	
	LOG_INF("TLX493D device initialized successfully");
	return 0;
}

#define TLX493D_INIT(index)						\
	static struct tlx493d_data tlx493d_data_##index;		\
									\
	static const struct tlx493d_config tlx493d_config_##index = {	\
		.i2c = I2C_DT_SPEC_INST_GET(index),			\
		.polling_interval_ms = DT_INST_PROP_OR(index,		\
			polling_interval_ms,				\
			CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(index, tlx493d_init, NULL,		\
			      &tlx493d_data_##index,			\
			      &tlx493d_config_##index,			\
			      POST_KERNEL,				\
			      CONFIG_INPUT_INIT_PRIORITY,		\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INIT)