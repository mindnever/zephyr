/*
 * Copyright (c) 2026, ARS Embedded Systems LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_pwm);

/* Each color channel is represented by 8 bits. */
#define BITS_PER_COLOR_CHANNEL 8
#define TRAILING_BITS   42

#define WS2812_PWM_CALC_BUFSZ(num_px, num_colors) \
	((num_px) * (num_colors) * BITS_PER_COLOR_CHANNEL + TRAILING_BITS)

typedef uint16_t octype_t;

struct ws2812_pwm_cfg {
	const struct pwm_dt_spec pwm;

	TIM_TypeDef *timer_base;

	octype_t *oc_buf;

	size_t length;
	const uint8_t *color_mapping;
	uint8_t num_colors;
};

struct ws2812_pwm_data {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;

	// Not 100% sure about following? They are for DMA, 
	int fifo_threshold;
	
	struct dma_block_config dma_blk_cfg;

	struct k_sem tx_busy_sem;

	uint8_t bit0_oc_value;
	uint8_t bit1_oc_value;
};


#define PWM_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev) \
	.dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(index, dir)),			 \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),	 \
	.dma_cfg = {							\
		.dma_slot = STM32_DMA_SLOT(index, dir, slot),       \
		.channel_direction = STM32_DMA_CONFIG_DIRECTION(	\
					STM32_DMA_CHANNEL_CONFIG(index, dir)),  \
		.cyclic =  STM32_DMA_CONFIG_CYCLIC(			        \
				STM32_DMA_CHANNEL_CONFIG(index, dir)),      \
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(		\
				STM32_DMA_CHANNEL_CONFIG(index, dir)),	    \
		.source_data_size = sizeof(octype_t),               \
		.dest_data_size = sizeof(octype_t),                                \
		.source_burst_length = sizeof(octype_t), /* SINGLE transfer */		\
		.dest_burst_length = sizeof(octype_t),					\
		.block_count = 1,					\
		.dma_callback = pwm_stm32_dma_##dir##_cb,		\
		.complete_callback_en = true, \
		.error_callback_dis = true, \
	},								\
	.fifo_threshold = STM32_DMA_FEATURES_FIFO_THRESHOLD(		\
				STM32_DMA_FEATURES(index, dir))

#define PWM_TIMER_BASE(inst) \
    ((TIM_TypeDef *)DT_REG_ADDR(DT_PARENT(DT_PWMS_CTLR(DT_DRV_INST(inst)))))

static void pwm_stm32_dma_tx_cb(const struct device *dma_dev, void *user_data,
			       uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct ws2812_pwm_cfg *config = dev->config;
	struct ws2812_pwm_data *data = dev->data;

	pwm_disable_dma(config->pwm.dev, config->pwm.channel);

	k_sem_give(&data->tx_busy_sem);
}

static int ws2812_strip_update_rgb(const struct device *dev, struct led_rgb *pixels,
				   size_t num_pixels)
{
	struct ws2812_pwm_data *data = dev->data;
	const struct ws2812_pwm_cfg *config = dev->config;

	/* Wait for previous transfer to complete. */
	k_sem_take(&data->tx_busy_sem, K_FOREVER);

	if (num_pixels > config->length) {
		num_pixels = config->length;
	}

	// Fill buffer
	unsigned bit_index = 0;

	for (unsigned current_pixel = 0; current_pixel < num_pixels; ++current_pixel) {
		for (unsigned color = 0; color < config->num_colors; ++color) {
			uint8_t color_val;

			switch (config->color_mapping[color]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				color_val = 0;
				break;
			case LED_COLOR_ID_RED:
				color_val = pixels[current_pixel].r;
				break;
			case LED_COLOR_ID_GREEN:
				color_val = pixels[current_pixel].g;
				break;
			case LED_COLOR_ID_BLUE:
				color_val = pixels[current_pixel].b;
				break;
			default:
				LOG_ERR("Invalid color mapping");
				k_sem_give(&data->tx_busy_sem);
				return -EINVAL;
			}

			for (int b = BITS_PER_COLOR_CHANNEL - 1; b >= 0; --b) {
				config->oc_buf[bit_index++] = color_val & (1 << b) ? data->bit1_oc_value : data->bit0_oc_value;
			}
		}
	}

	dma_reload(data->dma_dev, data->dma_channel, data->dma_blk_cfg.source_address, data->dma_blk_cfg.dest_address, data->dma_blk_cfg.block_size);
	dma_start(data->dma_dev, data->dma_channel);
	pwm_enable_dma(config->pwm.dev, config->pwm.channel);

	return 0;
}

static size_t ws2812_strip_length(const struct device *dev)
{
	const struct ws2812_pwm_cfg *config = dev->config;

	return config->length;
}

static int ws2812_pwm_init(const struct device *dev)
{
	const struct ws2812_pwm_cfg *config = dev->config;
	struct ws2812_pwm_data *data = dev->data;

	if (!device_is_ready(config->pwm.dev)) {
		LOG_ERR("%s: PWM device %s not ready", dev->name, config->pwm.dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(data->dma_dev)) {
		LOG_ERR("%s: DMA device %s not ready", dev->name, data->dma_dev->name);
		return -ENODEV;
	}

	for (int i = 0; i < config->num_colors; i++) {
		switch (config->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping.", dev->name);
			return -EINVAL;
		}
	}

	k_sem_init(&data->tx_busy_sem, 1, 1);

	data->dma_cfg.user_data = (void *)dev;

	data->dma_cfg.head_block = &data->dma_blk_cfg;

	// block_size is number of bytes
	data->dma_blk_cfg.block_size = WS2812_PWM_CALC_BUFSZ(config->length, config->num_colors) * sizeof(octype_t);

	data->dma_blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_blk_cfg.dest_address = (uint32_t)(&config->timer_base->CCR1 + (config->pwm.channel - 1));

	data->dma_blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_blk_cfg.source_address = (uint32_t)config->oc_buf;

	if (dma_config(data->dma_dev, data->dma_channel, &data->dma_cfg) != 0) {
		return -EIO;
	}

	uint64_t pwm_freq;
	if (pwm_get_cycles_per_sec(config->pwm.dev, config->pwm.channel, &pwm_freq) != 0) {
		return -EIO;
	}

	uint32_t timer_period = ((uint64_t)pwm_freq * config->pwm.period) / 1000000000ULL - 1;

	data->bit0_oc_value = (timer_period * 30) / 100;
	data->bit1_oc_value = (timer_period * 60) / 100;

	pwm_set_pulse_dt(&config->pwm, 0);

	LOG_INF("ws2812_pwm_init done");
	return 0;
}

static const struct led_strip_driver_api ws2812_pwm_api = {
	.update_rgb = ws2812_strip_update_rgb,
	.length = ws2812_strip_length,
};

#define WS2812_NUM_PIXELS(idx)           (DT_INST_PROP(idx, chain_length))
#define WS2812_NUM_COLORS(idx)           (DT_INST_PROP_LEN(idx, color_mapping))

#define WS2812_PWM_BUFSZ(idx)  \
	WS2812_PWM_CALC_BUFSZ(WS2812_NUM_PIXELS(idx), WS2812_NUM_COLORS(idx))

#define WS2812_PWM_DEVICE(idx)                                                                    \
	static octype_t ws2812_pwm_##idx##_oc_buf[WS2812_PWM_BUFSZ(idx)] \
		IF_ENABLED(CONFIG_WS2812_STRIP_PWM_FORCE_NOCACHE, (__nocache));                         \
	static struct ws2812_pwm_data ws2812_pwm_##idx##_data = { \
		PWM_DMA_CHANNEL_INIT(idx, tx, TX, MEMORY, PERIPHERAL) \
	};                                   \
	static const uint8_t ws2812_pwm_##idx##_color_mapping[] =                                 \
		DT_INST_PROP(idx, color_mapping);                                                  \
	static const struct ws2812_pwm_cfg ws2812_pwm_##idx##_cfg = {                            \
		.pwm = PWM_DT_SPEC_INST_GET(idx),                                    \
		.timer_base = PWM_TIMER_BASE(idx), \
		.oc_buf = ws2812_pwm_##idx##_oc_buf,                                              \
		.num_colors = WS2812_NUM_COLORS(idx),                                              \
		.color_mapping = ws2812_pwm_##idx##_color_mapping,                                \
		.length = WS2812_NUM_PIXELS(idx),                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ws2812_pwm_init, NULL, &ws2812_pwm_##idx##_data,              \
			      &ws2812_pwm_##idx##_cfg, POST_KERNEL,                               \
			      CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEVICE)
