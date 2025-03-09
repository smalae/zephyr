/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>

#include "spi_context.h"

#define DT_DRV_COMPAT silabs_siwx91x_gspi

/* Structure for DMA configuration */
struct gspi_siwx91x_dma_config {
	const struct device *dma_dev; /* Pointer to the DMA device structure */
	uint32_t dma_channel;         /* DMA channel number */
};

struct gspi_siwx91x_config {
	GSPI0_Type *reg;
	/* Pointer to the clock device structure */
	const struct device *clock_dev;
	/* Clock control subsystem */
	clock_control_subsys_t clock_subsys;
	/* Pointer to the pin control device configuration */
	const struct pinctrl_dev_config *pcfg;
};

struct gspi_siwx91x_data {
	struct spi_context ctx;
	struct gspi_siwx91x_dma_config dma_rx; /* DMA configuration for RX */
	struct gspi_siwx91x_dma_config dma_tx; /* DMA configuration for TX */
};

static bool spi_siwx91x_is_dma_enabled_instance(const struct device *dev)
{
#ifdef CONFIG_SPI_SILABS_SIWX91X_GSPI_DMA
	struct gspi_siwx91x_data *data = dev->data;

	__ASSERT_NO_MSG(!!data->dma_tx.dma_dev == !!data->dma_rx.dma_dev);

	return data->dma_rx.dma_dev != NULL;
#else
	return false;
#endif /* CONFIG_SPI_SILABS_SIWX91X_GSPI_DMA */
}

static int gspi_siwx91x_config(const struct device *dev, const struct spi_config *config)
{

	return 0;
}

static int gspi_siwx91x_transceive_dma(const struct device *dev, bool sync)
{

	return 0;
}

static int gspi_siwx91x_transceive_polling_sync(const struct device *dev)
{

	return 0;
}

static int gspi_siwx91x_transceive(const struct device *dev, const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs)
{
	struct gspi_siwx91x_data *data = dev->data;
	int ret = 0;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);
	if (!spi_context_configured(&data->ctx, config)) {
		ret = gspi_siwx91x_config(dev, config);
		if(ret) {
			return ret;
		}
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, SPI_WORD_SIZE_GET(config->operation) / 8);

	spi_context_cs_control(&data->ctx, true);
	if (spi_siwx91x_is_dma_enabled_instance(dev)) {
		ret = gspi_siwx91x_transceive_dma(dev, true);
	} else {
		ret = gspi_siwx91x_transceive_polling_sync(dev);
	}
	spi_context_cs_control(&data->ctx, false);
	if(ret) {
		return ret;
	}
	
	spi_context_release(&data->ctx, ret);

	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int gspi_siwx91x_transceive_async(const struct device *dev,
					     const struct spi_config *config,
					     const struct spi_buf_set *tx_bufs,
					     const struct spi_buf_set *rx_bufs,
					     struct k_poll_signal *async)
{
	return 0;
}
#endif /* CONFIG_SPI_ASYNC */

static int gspi_siwx91x_release(const struct device *dev, const struct spi_config *config)
{
	struct gspi_siwx91x_data *data = dev->data;

	if (spi_context_configured(&data->ctx, config)) {
		spi_context_unlock_unconditionally(&data->ctx);
	}
	return 0;
}

static int gspi_siwx91x_init(const struct device *dev)
{
	const struct gspi_siwx91x_config *cfg = dev->config;
	struct gspi_siwx91x_data *data = dev->data;
	int ret;
	
	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
	
	ret = spi_context_cs_configure_all(&data->ctx);
	if(ret) {
		return ret;
	}

	ret = spi_context_unlock_unconditionally(&data->ctx);
	if(ret) {
		return ret;
	}
	
	sys_write32(0x01, (mem_addr_t)&cfg->reg->GSPI_BUS_MODE_b.SPI_HIGH_PERFORMANCE_EN);
	sys_write32(0x00, (mem_addr_t)&cfg->reg->GSPI_CONFIG1_b.GSPI_MANUAL_CSN);

#if defined(CONFIG_SPI_SILABS_SIWX91X_GSPI_DMA)
	if (spi_siwx91x_is_dma_enabled_instance(dev)) {
		if (!device_is_ready(data->dma_rx.dma_dev) ||
		    !device_is_ready(data->dma_tx.dma_dev)) {
			return -ENODEV;
		}
		data->dma_rx.dma_channel = dma_request_channel(data->dma_rx.dma_dev, NULL);
		data->dma_tx.dma_channel = dma_request_channel(data->dma_tx.dma_dev, NULL);

		if (data->dma_rx.dma_channel < 0 || data->dma_tx.dma_channel < 0) {
			dma_release_channel(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
			dma_release_channel(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
			return -EAGAIN;
		}
	}
#endif /* CONFIG_SPI_SILABS_SIWX91X_GSPI_DMA */
	
	return 0;	
}

static DEVICE_API(spi, gspi_siwx91x_driver_api) = {
	.transceive = gspi_siwx91x_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = gspi_siwx91x_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
	.release = gspi_siwx91x_release,
};

#define SIWX91X_GSPI_INIT(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct gspi_siwx91x_data gspi_data_##inst = {					\
		SPI_CONTEXT_INIT_LOCK(gspi_data_##inst, ctx),                                        \
		SPI_CONTEXT_INIT_SYNC(gspi_data_##inst, ctx),                                        \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx),                               \
		.dma_rx = COND_CODE_1(                                                     \
			CONFIG_SPI_SILABS_SIWX91X_GSPI_DMA,                                                     \
			(DEVICE_DT_GET_OR_NULL(DT_INST_DMAS_CTLR_BY_NAME(idx, rx))), (NULL)),      \
		.dma_tx = COND_CODE_1(                                                     \
			CONFIG_SPI_SILABS_SIWX91X_GSPI_DMA,                                                     \
			(DEVICE_DT_GET_OR_NULL(DT_INST_DMAS_CTLR_BY_NAME(idx, tx))), (NULL)),      \
	};                                    \
	static const struct gspi_siwx91x_config gspi_config_##inst = {                               \
		.reg = (GSPI0_Type *)DT_INST_REG_ADDR(inst),					\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_PHA(inst, clocks, clkid),          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &gspi_siwx91x_init, NULL, &gspi_data_##inst,             \
			      &gspi_config_##inst, PRE_KERNEL_1, CONFIG_SPI_INIT_PRIORITY,          \
			      &gspi_siwx91x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_GSPI_INIT)
