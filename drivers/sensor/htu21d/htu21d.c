/* htu21d.c - Driver for TE-Connectivity HTU21D temperature and humidity sensor */

/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Datasheet:
 * https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FHPC199_6%7FA6%7Fpdf%7FEnglish%7FENG_DS_HPC199_6_A6.pdf%7FCAT-HSC0004
 */

#define DT_DRV_COMPAT te_connectivity_htu21d

#include <math.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>

#include "htu21d.h"

LOG_MODULE_REGISTER(HTU21D, CONFIG_SENSOR_LOG_LEVEL);

/* HTU21D commands and register addresses */
#define HTU21D_CMD_TRIGGER_TEMP_MEASURE_HOLD       0xE3
#define HTU21D_CMD_TRIGGER_HUMIDITY_MEASURE_HOLD   0xE5
#define HTU21D_CMD_TRIGGER_TEMP_MEASURE_NOHOLD     0xF3
#define HTU21D_CMD_TRIGGER_HUMIDITY_MEASURE_NOHOLD 0xF5
#define HTU21D_CMD_WRITE_USER_REG                  0xE6
#define HTU21D_CMD_READ_USER_REG                   0xE7
#define HTU21D_CMD_SOFT_RESET                      0xFE

/* HTU21D CRC characteristics */
#define HTU21D_CRC_POLY     0x31 /* CRC-8, Maximal 8-bit polynomial X8+X5+X4+1 */
#define HTU21D_CRC_INIT     0x00
#define HTU21D_CRC_REVERSED false

struct htu21d_config {
	struct i2c_dt_spec i2c;
};

struct htu21d_data {
	uint16_t humidity;
	uint16_t temp;
};

static int htu21d_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct htu21d_data *dev_data = dev->data;
	const struct htu21d_config *dev_config = dev->config;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	uint8_t buf[3];
	uint8_t crc;
	int ret;
	int size = 3;

	ret = i2c_burst_read_dt(&dev_config->i2c, HTU21D_CMD_TRIGGER_TEMP_MEASURE_HOLD, buf, size);
	if (!ret) {
		crc = crc8(buf, 2, HTU21D_CRC_POLY, HTU21D_CRC_INIT, HTU21D_CRC_REVERSED);
		if (crc != buf[2]) {
			LOG_ERR("CRC error");
			return -EIO;
		}
		dev_data->temp = ((buf[0] << 8) | buf[1]) & 0xFFFC; /* Clear status bits */
	}

	ret = i2c_burst_read_dt(&dev_config->i2c, HTU21D_CMD_TRIGGER_HUMIDITY_MEASURE_HOLD, buf,
				size);
	if (!ret) {
		crc = crc8(buf, 2, HTU21D_CRC_POLY, HTU21D_CRC_INIT, HTU21D_CRC_REVERSED);
		if (crc != buf[2]) {
			LOG_ERR("CRC error");
			return -EIO;
		}
		dev_data->humidity = ((buf[0] << 8) | buf[1]) & 0xFFFC; /* Clear status bits */
	}

	return ret;
}

static int htu21d_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct htu21d_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/* Temperature conversion in degrees Celsius
		 * T = -46.85 + 175.72 * (sample/2^16)
		 * LSB = 0.0026812744 degrees Celsius
		 * Everything is multiplied by 10^7 to maximize precision.
		 * Final result is scaled back to 10^6.
		 */
		val->val1 = (-468500000 + (data->temp * 26813)) / 10000000;
		val->val2 = ((-468500000 + (data->temp * 26813)) % 10000000) / 10;
		break;
	case SENSOR_CHAN_HUMIDITY:
		/* Relative humidity conversion in percent
		 * RH = -6 + 125 * (sample/2^16)
		 * LSB = 0.0019073486 percent
		 * Everything is multiplied by 10^7 to maximize precision.
		 * Final result is scaled back to 10^6.
		 */
		val->val1 = (-60000000 + (data->humidity * 19073)) / 10000000;
		val->val2 = ((-60000000 + (data->humidity * 19073)) % 10000000) / 10;
		break;
	case HTU21D_CHAN_DEW_POINT_TEMP:
		/* Dew point temperature conversion in degrees Celsius
		 * First, compute partial pressure from temperature:
		 * PP = 10^(8.1332 - 1762.39 / (T + 235.66))
		 * Then, compute dew point temperature:
		 * Td = -(1762.39 / (log10(RH * PP / 100) - 8.1332) + 235.66)
		 */
		double temperature = -46.85 + 175.72 * (data->temp / 65536.0);
		double humidity = -6 + 125 * (data->humidity / 65536.0);
		double partial_pressure = pow(10, (8.1332 - 1762.39 / (temperature + 235.66)));
		double dew_point =
			-(1762.39 / (log10(humidity * partial_pressure / 100) - 8.1332) + 235.66);
		LOG_DBG("T: %f, RH: %f, PP: %f, Td: %f", temperature, humidity, partial_pressure,
			dew_point);
		sensor_value_from_double(val, dew_point);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int htu21d_init(const struct device *dev)
{
	const struct htu21d_config *config = dev->config;
	int err;

	err = i2c_is_ready_dt(&config->i2c);
	if (err < 0) {
		LOG_ERR("I2C bus %s not ready: %d", config->i2c.bus->name, err);
		return err;
	}

	/* Soft reset */
	char buf[1] = {HTU21D_CMD_SOFT_RESET};
	err = i2c_write_dt(&config->i2c, buf, 1);
	if (err < 0) {
		LOG_ERR("Failed to reset sensor: %d", err);
		return err;
	}
	k_sleep(K_MSEC(15)); /* Wait for sensor to reset, cf. datasheet, page 12 */

	return 0;
}

static const struct sensor_driver_api htu21d_driver_api = {
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

#define HTU21D_INIT(n)                                                                             \
	static struct htu21d_config htu21d_config_##n = {                                          \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
	static struct htu21d_data htu21d_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, htu21d_init, NULL, &htu21d_data_##n, &htu21d_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &htu21d_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HTU21D_INIT)
