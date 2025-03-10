/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx91x_gspi
 
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include "clock_update.h"

LOG_MODULE_REGISTER(spi_max32, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"

#define GSPI_MAX_BAUDRATE_FOR_DYNAMIC_CLOCK	110000000
#define GSPI_STATIC_CLOCK_DIV_FACTOR		1
#define GSPI_MAX_BAUDRATE_FOR_POS_EDGE_SAMPLE	40000000

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

static int gspi_siwx91x_config(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct gspi_siwx91x_config *cfg = dev->config;
	struct gspi_siwx91x_data *data = dev->data;
	uint32_t bit_rate = spi_cfg->frequency;
	uint32_t clk_div_factor;

	if (spi_cfg->operation & (SPI_HALF_DUPLEX | SPI_CS_ACTIVE_HIGH | SPI_LOCK_ON |
				  SPI_TRANSFER_LSB | SPI_OP_MODE_SLAVE | SPI_MODE_LOOP)) {
		LOG_ERR("Unsupported configuration 0x%X!", spi_cfg->operation);
		return -ENOTSUP;
	}

	if(SPI_WORD_SIZE_GET(spi_cfg->operation) > 16) {
		LOG_ERR("Word size incorrect %d!", SPI_WORD_SIZE_GET(spi_cfg->operation));
		return -ENOTSUP;
	}

	if(IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
		   (spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only supports single mode!");
		return -ENOTSUP;
	}

	if (0 != (spi_cfg->operation & (SPI_MODE_CPOL | SPI_MODE_CPHA)) &&
		   ((SPI_MODE_CPOL | SPI_MODE_CPHA) !=
		    (spi_cfg->operation & (SPI_MODE_CPOL | SPI_MODE_CPHA)))) {
		LOG_ERR("Only SPI mode 0 and 3 supported!");
		return -ENOTSUP;
	} 

	if(bit_rate > GSPI_MAX_BAUDRATE_FOR_DYNAMIC_CLOCK) {
		clk_div_factor = GSPI_STATIC_CLOCK_DIV_FACTOR;
	} else {
		/* Get the clock using clock driver */
		clk_div_factor = ((RSI_CLK_GetBaseClock(M4_GSPI) / spi_cfg->frequency) / 2);
	}

	if (clk_div_factor < 1) {
		cfg->reg->GSPI_CLK_CONFIG_b.GSPI_CLK_EN = 0x1;
		cfg->reg->GSPI_CLK_CONFIG_b.GSPI_CLK_SYNC = 0x1;
	}
	
	if (bit_rate > GSPI_MAX_BAUDRATE_FOR_POS_EDGE_SAMPLE) {
		cfg->reg->GSPI_BUS_MODE_b.GSPI_DATA_SAMPLE_EDGE = ENABLE;
	}
	
	cfg->reg->GSPI_CLK_DIV_b.GSPI_CLK_DIV_FACTOR = (uint8_t)clk_div_factor;

	/*  Set SPI mode */
	if (0 == (spi_cfg->operation & (SPI_MODE_CPOL | SPI_MODE_CPHA))) {
		cfg->reg->GSPI_BUS_MODE_b.GSPI_CLK_MODE_CSN0 = 0;
	} else {
		cfg->reg->GSPI_BUS_MODE_b.GSPI_CLK_MODE_CSN0 = 1;
	}

	/*  Update the number of Data Bits */
	cfg->reg->GSPI_WRITE_DATA2_b.GSPI_MANUAL_WRITE_DATA2 =
		SPI_WORD_SIZE_GET(spi_cfg->operation);

	// Swap the read data inside the GSPI controller it-self.
	cfg->reg->GSPI_CONFIG2 &= ~BIT(4);

	cfg->reg->GSPI_CONFIG1_b.SPI_FULL_DUPLEX_EN = ENABLE;
	cfg->reg->GSPI_CONFIG1_b.GSPI_MANUAL_WR = ENABLE;
	cfg->reg->GSPI_CONFIG1_b.GSPI_MANUAL_RD = ENABLE;
	cfg->reg->GSPI_WRITE_DATA2_b.USE_PREV_LENGTH = true;

	data->ctx.config = spi_cfg;

	return 0;
}

static int gspi_siwx91x_transceive_dma(const struct device *dev, bool sync)
{
	return 0;
}

static uint32_t gspi_siwx91x_write_fifo(const struct device *dev, struct spi_context *ctx) {
	const struct gspi_siwx91x_config *cfg = dev->config;
	const uint32_t rd_size = MIN(ARRAY_SIZE(cfg->reg->GSPI_WRITE_FIFO), ctx->rx_len);
	const uint32_t wr_size = MIN(ARRAY_SIZE(cfg->reg->GSPI_WRITE_FIFO), ctx->tx_len);
	const uint32_t transfer_size = MAX(wr_size, rd_size);
	const uint8_t dfs = SPI_WORD_SIZE_GET(ctx->config->operation) / 8;

	for(int i = 0; i < transfer_size; i++) {
		uint32_t tx_data = ((i >= wr_size) || (!ctx->tx_buf))
								? 0
								: (1 == dfs)
									? UNALIGNED_GET((uint8_t *)(&ctx->tx_buf[i]))
									: UNALIGNED_GET((uint16_t *)(&ctx->tx_buf[i * 2]));
		cfg->reg->GSPI_WRITE_FIFO[i] = tx_data;
	}
	spi_context_update_tx(ctx, dfs, wr_size);
	return wr_size;
}

static uint32_t gspi_siwx91x_read_fifo(const struct device *dev, struct spi_context *ctx) {
	const struct gspi_siwx91x_config *cfg = dev->config;
	uint32_t dummy_buf;
	const uint32_t rd_size = MIN(ARRAY_SIZE(cfg->reg->GSPI_WRITE_FIFO), ctx->rx_len);
	const uint8_t dfs = SPI_WORD_SIZE_GET(ctx->config->operation) / 8;

	for (int i = 0; i < ARRAY_SIZE(cfg->reg->GSPI_WRITE_FIFO); i++) {
		void *rx_data = ((i >= ctx->rx_len) || (!ctx->rx_buf)) ? &dummy_buf : (void *)&ctx->rx_buf[i * dfs];
		if (1 == dfs) {
			UNALIGNED_PUT(cfg->reg->GSPI_READ_FIFO[i], (uint8_t *)rx_data);
		} else {
			UNALIGNED_PUT(cfg->reg->GSPI_READ_FIFO[i], (uint16_t *)rx_data);
		}
	}
	spi_context_update_rx(ctx, dfs, rd_size);
	return rd_size;
}

static int gspi_siwx91x_transceive_polling_sync(const struct device *dev, struct spi_context *ctx)
{
	const struct gspi_siwx91x_config *cfg = dev->config;

	while (ctx->tx_len || ctx->rx_len) {
		gspi_siwx91x_write_fifo(dev, ctx);
		while (cfg->reg->GSPI_STATUS_b.GSPI_BUSY)
			;
		gspi_siwx91x_read_fifo(dev, ctx);
	}

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
		printk("polling**********\n");
		ret = gspi_siwx91x_transceive_polling_sync(dev, &data->ctx);
	}
	spi_context_cs_control(&data->ctx, false);
	if(ret) {
		return ret;
	}
	
	spi_context_release(&data->ctx, ret);

	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int gspi_siwx91x_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs, spi_callback_t cb,
					void *userdata)
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

	spi_context_unlock_unconditionally(&data->ctx);
	
	cfg->reg->GSPI_BUS_MODE_b.SPI_HIGH_PERFORMANCE_EN = 0x01;
	cfg->reg->GSPI_CONFIG1_b.GSPI_MANUAL_CSN = 0x00;

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
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)                              \
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
