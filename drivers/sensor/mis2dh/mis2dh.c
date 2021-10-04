/* ST Microelectronics MIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2021 SKA Polska
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/mis2dh.pdf
 */

#define DT_DRV_COMPAT st_mis2dh

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "mis2dh.h"

LOG_MODULE_REGISTER(MIS2DH, CONFIG_SENSOR_LOG_LEVEL);

static struct mis2dh_data mis2dh_data;

static struct mis2dh_config mis2dh_config = {
	.comm_master_dev_name = DT_INST_BUS_LABEL(0),
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	.bus_init = mis2dh_spi_init,
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	.bus_init = mis2dh_i2c_init,
#else
#error "BUS MACRO NOT DEFINED IN DTS"
#endif
#ifdef CONFIG_MIS2DH_TRIGGER
	.irq_port	= DT_INST_GPIO_LABEL(0, irq_gpios),
	.irq_pin	= DT_INST_GPIO_PIN(0, irq_gpios),
	.irq_flags	= DT_INST_GPIO_FLAGS(0, irq_gpios),
#endif
};

#if defined(MIS2DH_ODR_RUNTIME)
/* page 33 Table 28 */
static const uint16_t mis2dh_hr_odr_map[] = {0, 1, 10, 25, 50, 100, 200, 400};

static int mis2dh_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(mis2dh_hr_odr_map); i++) {
		if (freq == mis2dh_hr_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int mis2dh_accel_odr_set(const struct device *dev, uint16_t freq)
{
	struct mis2dh_data *data = dev->data;
	int odr;

	odr = mis2dh_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (data->hw_tf->update_reg(data,
				    LIS2DS12_REG_CTRL1,
				    LIS2DS12_MASK_CTRL1_ODR,
				    odr << LIS2DS12_SHIFT_CTRL1_ODR) < 0) {
		LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	return 0;
}
#endif

#ifdef MIS2DH_FS_RUNTIME
/* page 35 Table 36 */
static const uint16_t mis2dh_accel_fs_map[] = {2, 4, 8, 16};
static const float mis2dh_accel_fs_sens[] = {MIS2DH_2G_GAIN, MIS2DH_4G_GAIN, MIS2DH_8G_GAIN, MIS2DH_16G_GAIN};

static int mis2dh_accel_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(mis2dh_accel_fs_map); i++) {
		if (range == mis2dh_accel_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int mis2dh_accel_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct mis2dh_data *data = dev->data;

	fs = mis2dh_accel_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (data->hw_tf->update_reg(data,
				    MIS2DH_REG_CTRL4,
				    MIS2DH_MASK_CTRL4_FS,
				    fs << MIS2DH_SHIFT_CTRL4_FS) < 0) {
		LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	data->gain = mis2dh_accel_fs_sens[fs];
	return 0;
}
#endif

static int mis2dh_accel_config(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	switch (attr) {
#ifdef MIS2DH_FS_RUNTIME
	case SENSOR_ATTR_FULL_SCALE:
		return mis2dh_accel_range_set(dev, sensor_ms2_to_g(val));
#endif
#ifdef MIS2DH_ODR_RUNTIME
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return mis2dh_accel_odr_set(dev, val->val1);
#endif
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int mis2dh_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return mis2dh_accel_config(dev, chan, attr, val);
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int mis2dh_sample_fetch_accel(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;
	uint8_t buf[6];

	if (data->hw_tf->read_data(data, MIS2DH_REG_OUTX_L,
				   buf, sizeof(buf)) < 0) {
		LOG_DBG("failed to read sample");
		return -EIO;
	}

	data->sample_x = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
	data->sample_y = (int16_t)((uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8));
	data->sample_z = (int16_t)((uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8));

	return 0;
}

static int mis2dh_sample_fetch_temp(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;
	uint8_t buf[2];

	if (data->hw_tf->read_data(data, MIS2DH_REG_OUT_TEMP_L,
				buf, sizeof(buf)) < 0) {
		LOG_DBG("failed to read temperature");
		return -EIO;
	}

	data->sample_t = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
}

static int mis2dh_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		mis2dh_sample_fetch_accel(dev);
		break;
#if defined(CONFIG_MIS2DH_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		mis2dh_sample_fetch_temp(dev)
		break;
#endif
	case SENSOR_CHAN_ALL:
		lis2ds12_sample_fetch_accel(dev);
#if defined(CONFIG_MIS2DH_ENABLE_TEMP)
		lis2ds12_sample_fetch_temp(dev)
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void mis2dh_convert(struct sensor_value *val, int raw_val,
				    float gain)
{
	int64_t dval;

	/* Gain is in mg/LSB */
	/* Convert to m/s^2 */
	dval = ((int64_t)raw_val * gain * SENSOR_G) / 1000;
	val->val1 = dval / 1000000LL;
	val->val2 = dval % 1000000LL;
}

static inline int mis2dh_get_channel(enum sensor_channel chan,
					     struct sensor_value *val,
					     struct mis2dh_data *data,
					     float gain)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		mis2dh_convert(val, data->sample_x, gain);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		mis2dh_convert(val, data->sample_y, gain);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		mis2dh_convert(val, data->sample_z, gain);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		mis2dh_convert(val, data->sample_x, gain);
		mis2dh_convert(val + 1, data->sample_y, gain);
		mis2dh_convert(val + 2, data->sample_z, gain);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mis2dh_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct mis2dh_data *data = dev->data;

	return mis2dh_get_channel(chan, val, data, data->gain);
}

static const struct sensor_driver_api mis2dh_api_funcs = {
	.attr_set = mis2dh_attr_set,
#if defined(CONFIG_MIS2DH_TRIGGER)
	.trigger_set = mis2dh_trigger_set,
#endif
	.sample_fetch = mis2dh_sample_fetch,
	.channel_get = mis2dh_channel_get,
};

static int mis2dh_init(const struct device *dev)
{
	const struct mis2dh_config * const config = dev->config;
	struct mis2dh_data *data = dev->data;
	uint8_t chip_id;

	data->comm_master = device_get_binding(config->comm_master_dev_name);
	if (!data->comm_master) {
		LOG_DBG("master not found: %s",
			    config->comm_master_dev_name);
		return -EINVAL;
	}

	config->bus_init(dev);

	/* s/w reset the sensor */
	if (data->hw_tf->update_reg(data,
                                    MIS2DH_REG_CTRL5,
				    MIS2DH_MASK_CTRL5_BOOT,
				    MIS2DH_CTRL5_BOOT) < 0) {
		LOG_DBG("s/w reset assert fail");
		return -EIO;
	}

	k_sleep(K_MSEC(10));

	if (data->hw_tf->update_reg(data,
                                    MIS2DH_REG_CTRL5,
				    MIS2DH_MASK_CTRL5_BOOT,
				    ~MIS2DH_CTRL5_BOOT) < 0) {
		LOG_DBG("s/w reset deassert fail");
		return -EIO;
	}

	if (data->hw_tf->read_reg(data, MIS2DH_REG_WHO_AM_I, &chip_id) < 0) {
		LOG_DBG("failed reading chip id");
		return -EIO;
	}

	if (chip_id != MIS2DH_VAL_WHO_AM_I) {
		LOG_DBG("invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	LOG_DBG("chip id 0x%x", chip_id);

	/* XYZ axis enable */
	if (data->hw_tf->update_reg(data,
				    MIS2DH_REG_CTRL1,
				    (MIS2DH_MASK_CTRL1_ZEN |
				     MIS2DH_MASK_CTRL1_YEN |
				     MIS2DH_MASK_CTRL1_XEN),
				    MIS2DH_CTRL1_XYZEN) < 0) {
		LOG_DBG("failed enabling axis");
		return -EIO;
	}

#ifdef CONFIG_MIS2DH_TRIGGER
	if (mis2dh_trigger_init(dev) < 0) {
		LOG_ERR("Failed to initialize triggers.");
		return -EIO;
	}
#endif

	/* set sensor default odr */
	if (data->hw_tf->update_reg(data,
				    MIS2DH_REG_CTRL1,
				    MIS2DH_MASK_CTRL1_ODR,
				    MIS2DH_DEFAULT_ODR) < 0) {
		LOG_DBG("failed setting odr");
		return -EIO;
	}

	/* set sensor default scale */
	if (data->hw_tf->update_reg(data,
				    MSI2DH_REG_CTRL4,
				    MIS2DH_MASK_CTRL4_FS,
				    MIS2DH_DEFAULT_FS) < 0) {
		LOG_DBG("failed setting scale");
		return -EIO;
	}
	data->gain = MIS2DH_DEFAULT_GAIN;

	return 0;
}

DEVICE_DT_INST_DEFINE(0, mis2dh_init, NULL,
		    &mis2dh_data, &mis2dh_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &mis2dh_api_funcs);
