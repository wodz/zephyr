/* ST Microelectronics MIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2021 SKA Polska
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/mis2dh.pdf
 */

#include <device.h>
#include <drivers/i2c.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include "mis2dh.h"

LOG_MODULE_DECLARE(MIS2DH, CONFIG_SENSOR_LOG_LEVEL);

static void mis2dh_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct mis2dh_data *data =
		CONTAINER_OF(cb, struct mis2dh_data, gpio_cb);
	const struct mis2dh_config *cfg = data->dev->config;

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure(data->gpio, cfg->irq_pin,
				     GPIO_INT_DISABLE);

#if defined(CONFIG_MIS2DH_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#elif defined(CONFIG_MIS2DH_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

static void mis2dh_handle_drdy_int(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;

	if (data->data_ready_handler != NULL) {
		data->data_ready_handler(dev, &data->data_ready_trigger);
	}
}

static void mis2dh_handle_int(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;
	const struct mis2dh_config *cfg = dev->config;
	uint8_t status;

	if (data->hw_tf->read_reg(data, MIS2DH_REG_STATUS, &status) < 0) {
		LOG_ERR("status reading error");
		return;
	}

	if (status & MIS2DH_INT_DRDY) {
		mis2dh_handle_drdy_int(dev);
	}

	gpio_pin_interrupt_configure(data->gpio, cfg->irq_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);
}

#ifdef CONFIG_MIS2DH_TRIGGER_OWN_THREAD
static void mis2dh_thread(struct lis2ds12_data *data)
{
	while (1) {
		k_sem_take(&data->trig_sem, K_FOREVER);
		mis2dh_handle_int(data->dev);
	}
}
#endif

#ifdef CONFIG_MIS2DH_TRIGGER_GLOBAL_THREAD
static void mis2dh_work_cb(struct k_work *work)
{
	struct mis2dh_data *data =
		CONTAINER_OF(work, struct mis2dh_data, work);

	mis2dh_handle_int(data->dev);
}
#endif

static int mis2dh_init_interrupt(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;

	/* Enable latched mode on INT1 */
	if (data->hw_tf->update_reg(data,
				    MIS2DH_REG_CTRL5,
				    MIS2DH_MASK_CTRL5_LIR_INT1,
				    (1 << MIS2DH_SHIFT_CTRL5_LIR_INT1)) < 0) {
		LOG_ERR("Could not enable LIR mode.");
		return -EIO;
	}

	/* enable data-ready interrupt on INT1 */
	if (data->hw_tf->update_reg(data,
				    MIS2DH_REG_CTRL3,
				    MIS2DH_MASK_INT1_DRDY,
				    (1 << MIS2DH_SHIFT_INT1_DRDY)) < 0) {
		LOG_ERR("Could not enable data-ready interrupt.");
		return -EIO;
	}

	return 0;
}

int mis2dh_trigger_init(const struct device *dev)
{
	struct mis2dh_data *data = dev->data;
	const struct mis2dh_config *cfg = dev->config;

	/* setup data ready gpio interrupt */
	data->gpio = device_get_binding(cfg->irq_port);
	if (data->gpio == NULL) {
		LOG_ERR("Cannot get pointer to %s device.", cfg->irq_port);
		return -EINVAL;
	}

	gpio_pin_configure(data->gpio, cfg->irq_pin,
			   GPIO_INPUT | cfg->irq_flags);

	gpio_init_callback(&data->gpio_cb,
			   lis2ds12_gpio_callback,
			   BIT(cfg->irq_pin));

	if (gpio_add_callback(data->gpio, &data->gpio_cb) < 0) {
		LOG_ERR("Could not set gpio callback.");
		return -EIO;
	}
	data->dev = dev;

#if defined(CONFIG_MIS2DH_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_MIS2DH_THREAD_STACK_SIZE,
			(k_thread_entry_t)mis2dh_thread,
			data, NULL, NULL,
			K_PRIO_COOP(CONFIG_LIS2DS12_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_MIS2DH_TRIGGER_GLOBAL_THREAD)
	data->work.handler = lis2ds12_work_cb;
#endif

	gpio_pin_interrupt_configure(data->gpio, cfg->irq_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

int mis2dh_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct mis2dh_data *data = dev->data;
	const struct lis2ds12_config *cfg = dev->config;
	uint8_t buf[6];

	__ASSERT_NO_MSG(trig->type == SENSOR_TRIG_DATA_READY);

	gpio_pin_interrupt_configure(data->gpio, cfg->irq_pin,
				     GPIO_INT_DISABLE);

	data->data_ready_handler = handler;
	if (handler == NULL) {
		LOG_WRN("mis2dh: no handler");
		return 0;
	}

	/* re-trigger lost interrupt */
	if (data->hw_tf->read_data(data, MIS2DH_REG_OUTX_L,
				   buf, sizeof(buf)) < 0) {
		LOG_ERR("status reading error");
		return -EIO;
	}

	data->data_ready_trigger = *trig;

	mis2dh_init_interrupt(dev);
	gpio_pin_interrupt_configure(data->gpio, cfg->irq_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}
