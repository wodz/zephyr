/* ST Microelectronics MIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2021 SKA Polska
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/mis2dh.pdf
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MIS2DH_MIS2DH_H_
#define ZEPHYR_DRIVERS_SENSOR_MIS2DH_MIS2DH_H_

#include <zephyr/types.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>

#define MIS2DH_REG_OUT_TEMP_L			0x0C
#define MIS2DH_REG_OUT_TEMP_H			0x0D

#define MIS2DH_REG_WHO_AM_I			0x0F
#define MIS2DH_VAL_WHO_AM_I			0x33

#define MIS2DH_REG_TEMP_CFG			0x1F
#define MIS2DH_MASK_TEMP_CFG_TEMP_EN		(BIT(7) | BIT(6))
#define MIS2DH_TEMP_EN				(BIT(7) | BIT(6))

#define MIS2DH_REG_CTRL1			0x20
#define MIS2DH_MASK_CTRL1_ODR			(BIT(7) | BIT(6) | \
						 BIT(5) | BIT(4))
#define MIS2DH_SHIFT_CTRL1_ODR			4
#define MIS2DH_MASK_CTRL1_LPEN			BIT(3)
#define MIS2DH_MASK_CTRL1_ZEN			BIT(2)
#define MIS2DH_MASK_CTRL1_YEN			BIT(1)
#define MIS2DH_MASK_CTRL1_XEN			BIT(0)
#define MIS2DH_CTRL1_XYZEN			(BIT(2) | BIT(1) | BIT(0))

#define MIS2DH_REG_CTRL2			0x21
#define MIS2DH_MASK_CTRL2_HPM			(BIT(7) | BIT(6))
#define MIS2DH_SHIFT_CTRL2_HPM			6
#define MIS2DH_MASK_CTRL2_HPCF			(BIT(5) | BIT(4))
#define MIS2DH_SHIFT_CTRL2_HPCF			4
#define MIS2DH_CTRL2_FDS			BIT(3)
#define MIS2DH_CTRL2_HPCLICK			BIT(2)
#define MIS2DH_MASK_CTRL2_HPIS			(BIT(1) | BIT(0))
#define MIS2DH_SHIFT_CTRL2_HPIS			0

#define MIS2DH_REG_CTRL3			0x22
#define MIS2DH_SHIFT_INT1_DRDY			4
#define MIS2DH_MASK_INT1_DRDY			BIT(4)

#define MIS2DH_REG_CTRL4			0x23
#define MIS2DH_MASK_CTRL4_FS			(BIT(5) | BIT(4))
#define MIS2DH_SHIFT_CTRL4_FS			4

#define MIS2DH_REG_CTRL5			0x24
#define MIS2DH_CTRL5_BOOT			BIT(7)
#define MIS2DH_MASK_CTRL5_BOOT			BIT(7)
#define MIS2DH_MASK_CTRL5_LIR_INT1		BIT(3)
#define MIS2DH_SHIFT_CTRL5_LIR_INT1		3

#define MIS2DH_REG_CTRL6			0x25

#define MIS2DH_REG_OUT_T			0x26

#define MIS2DH_REG_STATUS			0x27
#define MIS2DH_INT_DRDY				BIT(3)

#define MIS2DH_REG_OUTX_L			0x28
#define MIS2DH_REG_OUTX_H			0x29
#define MIS2DH_REG_OUTY_L			0x2A
#define MIS2DH_REG_OUTY_H			0x2B
#define MIS2DH_REG_OUTZ_L			0x2C
#define MIS2DH_REG_OUTZ_H			0x2D


#if (CONFIG_MIS2DH_ODR == 0)
#define MIS2DH_ODR_RUNTIME			1
#define MIS2DH_DEFAULT_ODR			0
#else
#define MIS2DH_DEFAULT_ODR			(CONFIG_MIS2DH_ODR << MIS2DH_SHIFT_CTRL1_ODR)
#endif

/* Accel sensor sensitivity unit  mg/LSB */
/* page 10 Table 2.1 typical values */
#define MIS2DH_2G_GAIN	(98LL / 1000.0)
#define MIS2DH_4G_GAIN	(195LL / 1000.0)
#define MIS2DH_8G_GAIN	(391LL / 1000.0)
#define MIS2DH_16G_GAIN	(1172LL / 1000.0)

#if CONFIG_MIS2DH_FS == 0
#define MIS2DH_FS_RUNTIME 1
#define MIS2DH_DEFAULT_FS			(0 << MIS2DH_SHIFT_CTRL4_FS)
#define MIS2DH_DEFAULT_GAIN			MIS2DH_2G_GAIN
#elif CONFIG_MIS2DH_FS == 2
#define MIS2DH_DEFAULT_FS			(0 << MIS2DH_SHIFT_CTRL4_FS)
#define MIS2DH_DEFAULT_GAIN			MIS2DH_2G_GAIN
#elif CONFIG_MIS2DH_FS == 4
#define MIS2DH_DEFAULT_FS			(1 << MIS2DH_SHIFT_CTRL4_FS)
#define MIS2DH_DEFAULT_GAIN			MIS2DH_4G_GAIN
#elif CONFIG_MIS2DH_FS == 8
#define MIS2DH_DEFAULT_FS			(2 << MIS2DH_SHIFT_CTRL4_FS)
#define MIS2DH_DEFAULT_GAIN			MIS2DH_8G_GAIN
#elif CONFIG_MIS2DH_FS == 16
#define MIS2DH_DEFAULT_FS			(3 << MIS2DH_SHIFT_CTRL4_FS)
#define MIS2DH_DEFAULT_GAIN			MIS2DH_16G_GAIN
#else
#error "Bad MIS2DH FS value (should be 0, 2, 4, 8, 16)"
#endif

struct mis2dh_config {
	char *comm_master_dev_name;
	int (*bus_init)(const struct device *dev);
#ifdef CONFIG_MIS2DH_TRIGGER
	const char *irq_port;
	gpio_pin_t irq_pin;
	gpio_dt_flags_t irq_flags;
#endif
};

struct mis2dh_data;

struct mis2dh_transfer_function {
	int (*read_data)(struct mis2dh_data *data, uint8_t reg_addr,
			 uint8_t *value, uint8_t len);
	int (*write_data)(struct mis2dh_data *data, uint8_t reg_addr,
			  uint8_t *value, uint8_t len);
	int (*read_reg)(struct mis2dh_data *data, uint8_t reg_addr,
			uint8_t *value);
	int (*write_reg)(struct mis2dh_data *data, uint8_t reg_addr,
			uint8_t value);
	int (*update_reg)(struct mis2dh_data *data, uint8_t reg_addr,
			  uint8_t mask, uint8_t value);
};

struct mis2dh_data {
	const struct device *comm_master;
	int sample_t;
	int sample_x;
	int sample_y;
	int sample_z;
	float gain;
	const struct mis2dh_transfer_function *hw_tf;

#ifdef CONFIG_MIS2DH_TRIGGER
	const struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;
	const struct device *dev;

#if defined(CONFIG_MIS2DH_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MIS2DH_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#elif defined(CONFIG_MIS2DH_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_MIS2DH_TRIGGER */
};

int mis2dh_spi_init(const struct device *dev);
int mis2dh_i2c_init(const struct device *dev);

#ifdef CONFIG_MIS2DH_TRIGGER
int mis2dh_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int mis2dh_trigger_init(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_MIS2DH_MIS2DH_H_ */
