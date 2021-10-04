/* ST Microelectronics LIS2DS12 3-axis accelerometer driver
 *
 * Copyright (c) 2019 SKA Polska
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/mis2dh.pdf
 */

#define DT_DRV_COMPAT st_mis2dh

#include <string.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "mis2dh.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static uint16_t mis2dh_i2c_slave_addr = DT_INST_REG_ADDR(0);

LOG_MODULE_DECLARE(MIS2D, CONFIG_SENSOR_LOG_LEVEL);

static int mis2dh_i2c_read_data(struct mis2dh_data *data, uint8_t reg_addr,
				 uint8_t *value, uint8_t len)
{
	return i2c_burst_read(data->comm_master, mis2dh_i2c_slave_addr,
			      reg_addr, value, len);
}

static int mis2dh_i2c_write_data(struct mis2dh_data *data, uint8_t reg_addr,
				  uint8_t *value, uint8_t len)
{
	return i2c_burst_write(data->comm_master, mis2dh_i2c_slave_addr,
			       reg_addr, value, len);
}

static int mis2dh_i2c_read_reg(struct mis2dh_data *data, uint8_t reg_addr,
				uint8_t *value)
{
	return i2c_reg_read_byte(data->comm_master, mis2dh_i2c_slave_addr,
				 reg_addr, value);
}

static int mis2dh_i2c_write_reg(struct mis2dh_data *data, uint8_t reg_addr,
				uint8_t value)
{
	return i2c_reg_write_byte(data->comm_master, mis2dh_i2c_slave_addr,
				 reg_addr, value);
}

static int mis2dh_i2c_update_reg(struct mis2dh_data *data, uint8_t reg_addr,
				  uint8_t mask, uint8_t value)
{
	return i2c_reg_update_byte(data->comm_master, mis2dh_i2c_slave_addr,
				   reg_addr, mask, value);
}

static const struct mis2dh_transfer_function mis2dh_i2c_transfer_fn = {
	.read_data = mis2dh_i2c_read_data,
	.write_data = mis2dh_i2c_write_data,
	.read_reg  = mis2dh_i2c_read_reg,
	.write_reg  = mis2dh_i2c_write_reg,
	.update_reg = mis2dh_i2c_update_reg,
};

int mis2dh_i2c_init(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;

	data->hw_tf = &mis2dh_i2c_transfer_fn;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
