/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <htu21d.h>

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(te_connectivity_htu21d);
	struct sensor_value temp;
	struct sensor_value humidity;
	struct sensor_value dew_point;

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	while (1) {
		sensor_sample_fetch(dev);

		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev, HTU21D_CHAN_DEW_POINT_TEMP, &dew_point);

		printk("Temperature: %d.%06d °C\tHumidity: %d.%06d %%\tDew point: %d.%06d °C\n",
		       temp.val1, temp.val2, humidity.val1, humidity.val2, dew_point.val1,
		       dew_point.val2);

		k_sleep(K_MSEC(1000));
	}
}
