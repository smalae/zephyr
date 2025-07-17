/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/mem_blocks.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include "rsi_gpdma.h"
#include "rsi_rom_gpdma.h"

#define DT_DRV_COMPAT                silabs_siwx91x_gpdma
#define GPDMA_DESC_MAX_TRANSFER_SIZE 4096

LOG_MODULE_REGISTER(si91x_gpdma, CONFIG_DMA_LOG_LEVEL);

enum gpdma_xfer_dir {
	TRANSFER_MEM_TO_MEM,
	TRANSFER_MEM_TO_PER,
	TRANSFER_PER_TO_MEM,
	TRANSFER_DIR_INVALID,
};

enum gpdma_data_width {
	DATA_WIDTH_8,
	DATA_WIDTH_16,
	DATA_WIDTH_32,
	DATA_WIDTH_INVALID,
};

struct gpdma_siwx91x_channel_info {
	dma_callback_t dma_callback; /* User callback */
	void *cb_data;               /* User callback data */
	RSI_GPDMA_DESC_T *chan_base_desc;
	uint8_t desc_count;
	enum gpdma_xfer_dir xfer_direction; /* mem<->mem ot per<->mem */
};

struct gpdma_siwx91x_config {
	GPDMA_G_Type *reg;         /* GPDMA base address */
	GPDMA_C_Type *channel_reg; /* GPDMA channel registers address */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_configure)(void); /* IRQ configure function */
};

struct gpdma_siwx91x_data {
	struct dma_context dma_ctx;
	RSI_GPDMA_HANDLE_T gpdma_handle;
	struct sys_mem_blocks *gpdma_desc_pool;
	struct gpdma_siwx91x_channel_info *chan_info;
	uint8_t channel_xfer_status;
	uint8_t reload_compatible;
};

static GPDMA_DATACONTEXT_T gpdma_context __aligned(4);

static bool siwx91x_is_priority_valid(uint32_t channel_priority)
{
	switch (channel_priority) {
	case 0:
	case 1:
	case 2:
	case 3:
		return true;
	default:
		return false;
	}
}

static enum gpdma_xfer_dir siwx91x_transfer_direction(uint32_t dir)
{
	switch (dir) {
	case MEMORY_TO_MEMORY:
		return TRANSFER_MEM_TO_MEM;
	case MEMORY_TO_PERIPHERAL:
		return TRANSFER_MEM_TO_PER;
	case PERIPHERAL_TO_MEMORY:
		return TRANSFER_PER_TO_MEM;
	default:
		return TRANSFER_DIR_INVALID;
	}
}

static enum gpdma_data_width siwx91x_data_size(uint32_t data_size)
{
	switch (data_size) {
	case 1:
		return DATA_WIDTH_8;
	case 2:
		return DATA_WIDTH_16;
	case 4:
		return DATA_WIDTH_32;
	default:
		return DATA_WIDTH_INVALID;
	}
}

static int siwx91x_ahb_burst_size(uint8_t burst_size)
{
	if (burst_size < 4) {
		return AHBBURST_SIZE_1;	
	} else if (burst_size < 8) {
		return AHBBURST_SIZE_4;
	} else if (burst_size < 16) {
		return AHBBURST_SIZE_8;
	} else if (burst_size < 20) {
		return AHBBURST_SIZE_16;
	} else if (burst_size < 24) {
		return AHBBURST_SIZE_20;
	} else if (burst_size < 28) {
		return AHBBURST_SIZE_24;
	} else if (burst_size < 32) {
		return AHBBURST_SIZE_28;
	} else if (burst_size == 32) {
		return AHBBURST_SIZE_32;
	} else {
		return -EINVAL;
	}
}

static int siwx91x_gpdma_desc_config(struct gpdma_siwx91x_data *data,
				     const struct dma_config *config, RSI_GPDMA_DESC_T *xfer_cfg,
				     uint32_t channel)
{
	const struct dma_block_config *block_addr = config->head_block;
	RSI_GPDMA_DESC_T *pPrevDesc = NULL;
	void *gpdma_desc_addr = NULL;
	int block_alloc_count = 0;

	for (int i = 0; i < config->block_count; i++) {
		if (block_addr->block_size >
		    (GPDMA_DESC_MAX_TRANSFER_SIZE - config->source_data_size)) {
			return -EINVAL;
		}

		xfer_cfg->src = (void *)block_addr->source_address;
		xfer_cfg->dest = (void *)block_addr->dest_address;
		xfer_cfg->chnlCtrlConfig.transSize = block_addr->block_size;

		if (block_addr->dest_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) {
			xfer_cfg->chnlCtrlConfig.dstFifoMode = 1;
			xfer_cfg->chnlCtrlConfig.srcFifoMode = 0;
		}

		if (block_addr->source_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) {
			xfer_cfg->chnlCtrlConfig.srcFifoMode = 1;
			xfer_cfg->chnlCtrlConfig.dstFifoMode = 0;
		}
#if 0
		printf("xfer_cfg->src = %x\n",xfer_cfg->src);
		printf("xfer_cfg->dest = %x\n",xfer_cfg->dest);
		printf("xfer_cfg->chnlCtrlConfig.transSize = %d\n", xfer_cfg->chnlCtrlConfig.transSize);
#endif
		if (sys_mem_blocks_alloc_contiguous(data->gpdma_desc_pool, 1, &gpdma_desc_addr)) {
			if (pPrevDesc == NULL) {
				return -EINVAL;
			}
			goto free_desc;
		}
		block_alloc_count += 1;

		if (pPrevDesc == NULL) {
			data->chan_info[channel].chan_base_desc =
				(RSI_GPDMA_DESC_T *)gpdma_desc_addr;
		}

		memset(gpdma_desc_addr, 0, 32);
#if 0
		printf("gpdma_desc_addr = %x\n", gpdma_desc_addr);
#endif

		if (RSI_GPDMA_BuildDescriptors(data->gpdma_handle, xfer_cfg,
					       (RSI_GPDMA_DESC_T *)gpdma_desc_addr, pPrevDesc)) {
			goto free_desc;
		}

		pPrevDesc = (RSI_GPDMA_DESC_T *)gpdma_desc_addr;
		block_addr = block_addr->next_block;
	}

	if (block_addr != NULL) {
		/* next_block address for last block must be null */
		goto free_desc;
	}
	//printf("block count = %d\n", block_alloc_count);

	if (RSI_GPDMA_SetupChannelTransfer(data->gpdma_handle, channel,
					   data->chan_info[channel].chan_base_desc)) {
		goto free_desc;
	}

	data->chan_info[channel].desc_count = block_alloc_count;

	return 0;

free_desc:
	sys_mem_blocks_free_contiguous(data->gpdma_desc_pool,
				       (void *)data->chan_info[channel].chan_base_desc,
				       block_alloc_count);
	return -EINVAL;
}

static int siwx91x_gpdma_xfer_configure(const struct device *dev, const struct dma_config *config,
					uint32_t channel)
{
	const struct gpdma_siwx91x_config *cfg = dev->config;
	struct gpdma_siwx91x_data *data = dev->data;
	RSI_GPDMA_DESC_T xfer_cfg = {0};
	int ret;

	xfer_cfg.chnlCtrlConfig.transType = siwx91x_transfer_direction(config->channel_direction);
	if (xfer_cfg.chnlCtrlConfig.transType == TRANSFER_DIR_INVALID) {
		return -EINVAL;
	}

	data->chan_info[channel].xfer_direction = config->channel_direction;

	if (config->dest_data_size != config->source_data_size) {
		return -EINVAL;
	}

	if (config->dest_burst_length != config->source_burst_length) {
		return -EINVAL;
	}

	xfer_cfg.chnlCtrlConfig.srcDataWidth = siwx91x_data_size(config->source_data_size);
	if (xfer_cfg.chnlCtrlConfig.destDataWidth == DATA_WIDTH_INVALID) {
		return -EINVAL;
	}

	xfer_cfg.chnlCtrlConfig.destDataWidth = xfer_cfg.chnlCtrlConfig.srcDataWidth;

	if (config->block_count == 1) {
		xfer_cfg.chnlCtrlConfig.linkListOn = 0;
		data->reload_compatible = 1;
	} else {
		xfer_cfg.chnlCtrlConfig.linkListOn = 1;
		data->reload_compatible = 0;
	}
	xfer_cfg.chnlCtrlConfig.linkInterrupt = config->complete_callback_en;

	if (xfer_cfg.chnlCtrlConfig.transType == TRANSFER_MEM_TO_PER) {
		xfer_cfg.miscChnlCtrlConfig.destChannelId = config->dma_slot;
		//xfer_cfg.chnlCtrlConfig.mastrIfFetchSel = 0;
		//xfer_cfg.chnlCtrlConfig.mastrIfSendSel = 1;
	} else if (xfer_cfg.chnlCtrlConfig.transType == TRANSFER_PER_TO_MEM) {
		xfer_cfg.miscChnlCtrlConfig.srcChannelId = config->dma_slot;
		//xfer_cfg.chnlCtrlConfig.mastrIfFetchSel = 1;
		//xfer_cfg.chnlCtrlConfig.mastrIfSendSel = 0;
	}

#if 1
	xfer_cfg.miscChnlCtrlConfig.ahbBurstSize = siwx91x_ahb_burst_size(config->source_burst_length);
	xfer_cfg.miscChnlCtrlConfig.destDataBurst = config->dest_burst_length;
	xfer_cfg.miscChnlCtrlConfig.srcDataBurst = config->source_burst_length;
#endif

#if 0
	xfer_cfg.miscChnlCtrlConfig.ahbBurstSize = 1;
	xfer_cfg.miscChnlCtrlConfig.destDataBurst = 4;
	xfer_cfg.miscChnlCtrlConfig.srcDataBurst = 4;
#endif
#if 0
	printf("%d\n", xfer_cfg.miscChnlCtrlConfig.ahbBurstSize);
	printf("%d\n", xfer_cfg.miscChnlCtrlConfig.destDataBurst);
	printf("%d\n", xfer_cfg.miscChnlCtrlConfig.srcDataBurst);
#endif
	ret = siwx91x_gpdma_desc_config(data, config, &xfer_cfg, channel);
	if (ret) {
		return ret;
	}

	return 0;
}

static int siwx91x_gpdma_configure(const struct device *dev, uint32_t channel,
				   struct dma_config *config)
{
	const struct gpdma_siwx91x_config *cfg = dev->config;
	struct gpdma_siwx91x_data *data = dev->data;
	RSI_GPDMA_CHA_CFG_T gpdma_channel_cfg = {
		.descFetchDoneIntr = config->complete_callback_en,
		.hrespErr = 1,
		.gpdmacErr = 1,
		.xferDoneIntr = 1,
		.dmaCh = channel,
	};
	int ret;

	if (channel >= data->dma_ctx.dma_channels) {
		return -EINVAL;
	}

	if (config->cyclic) {
		return -ENOTSUP;
	}

	if (sys_test_bit((mem_addr_t)&data->channel_xfer_status, channel)) {
		/* Transfer in progress */
		return -EIO;
	}
#if 0
	if (channel == 0)
	printf("tx-----------\n");
	else
	printf("rx-----------\n");
#endif

	if (!siwx91x_is_priority_valid(config->channel_priority)) {
		return -EINVAL;
	}
	gpdma_channel_cfg.channelPrio = config->channel_priority;

	if (RSI_GPDMA_SetupChannel(data->gpdma_handle, &gpdma_channel_cfg)) {
		return -EIO;
	}

	ret = siwx91x_gpdma_xfer_configure(dev, config, channel);
	if (ret) {
		return -EINVAL;
	}

	//cfg->reg->GLOBAL.DMA_CHNL_SQUASH_REG = BIT(channel);
	cfg->channel_reg->CHANNEL_CONFIG[channel].FIFO_CONFIG_REGS = 0;
	if (channel) {
	cfg->channel_reg->CHANNEL_CONFIG[channel].FIFO_CONFIG_REGS_b.FIFO_SIZE = 8;
	cfg->channel_reg->CHANNEL_CONFIG[channel].FIFO_CONFIG_REGS_b.FIFO_STRT_ADDR = 0;
	} else {
	cfg->channel_reg->CHANNEL_CONFIG[channel].FIFO_CONFIG_REGS_b.FIFO_SIZE = 8;
	cfg->channel_reg->CHANNEL_CONFIG[channel].FIFO_CONFIG_REGS_b.FIFO_STRT_ADDR = 8;
	}
	//cfg->channel_reg->CHANNEL_CONFIG[channel].PRIORITY_CHNL_REGS_b.PRIORITY_CH = 0;

	data->chan_info[channel].dma_callback = config->dma_callback;
	data->chan_info[channel].cb_data = config->user_data;
	atomic_set_bit(data->dma_ctx.atomic, channel);

#if 0
	printf("GPDMA_G->GLOBAL.DMA_CHNL_ENABLE_REG = %x\n", GPDMA_G->GLOBAL.DMA_CHNL_ENABLE_REG);
	printf("GPDMA_G->GLOBAL.DMA_CHNL_LOCK_REG = %x\n", GPDMA_G->GLOBAL.DMA_CHNL_LOCK_REG);
	printf("GPDMA_G->GLOBAL.DMA_CHNL_SQUASH_REG = %x\n", GPDMA_G->GLOBAL.DMA_CHNL_SQUASH_REG);
	printf("GPDMA_G->GLOBAL.INTERRUPT_MASK_REG = %x\n", GPDMA_G->GLOBAL.INTERRUPT_MASK_REG);
	printf("GPDMA_G->GLOBAL.INTERRUPT_REG = %x\n", GPDMA_G->GLOBAL.INTERRUPT_REG);
	printf("GPDMA_G->GLOBAL.INTERRUPT_STAT_REG = %x\n", GPDMA_G->GLOBAL.INTERRUPT_STAT_REG);
	printf("\n");
	printf("GPDMA_C->CHANNEL_CONFIG[].CHANNEL_CTRL_REG_CHNL = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].CHANNEL_CTRL_REG_CHNL);
	printf("GPDMA_C->CHANNEL_CONFIG[].DEST_ADDR_REG_CHNL = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].DEST_ADDR_REG_CHNL);
	printf("GPDMA_C->CHANNEL_CONFIG[].SRC_ADDR_REG_CHNL = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].SRC_ADDR_REG_CHNL);
	printf("GPDMA_C->CHANNEL_CONFIG[].FIFO_CONFIG_REGS = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].FIFO_CONFIG_REGS);
	printf("GPDMA_C->CHANNEL_CONFIG[].LINK_LIST_PTR_REGS = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].LINK_LIST_PTR_REGS);
	printf("GPDMA_C->CHANNEL_CONFIG[].MISC_CHANNEL_CTRL_REG_CHNL = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].MISC_CHANNEL_CTRL_REG_CHNL);
	printf("GPDMA_C->CHANNEL_CONFIG[].PRIORITY_CHNL_REGS = %x\n", GPDMA_C->CHANNEL_CONFIG[channel].PRIORITY_CHNL_REGS);
#endif

	return 0;
}

static int siwx91x_gpdma_reload(const struct device *dev, uint32_t channel, uint32_t src,
				uint32_t dst, size_t size)
{
	const struct gpdma_siwx91x_config *cfg = dev->config;
	struct gpdma_siwx91x_data *data = dev->data;
	int data_size = siwx91x_data_size(
		cfg->channel_reg->CHANNEL_CONFIG[channel].CHANNEL_CTRL_REG_CHNL_b.SRC_DATA_WIDTH);

	if (channel >= data->dma_ctx.dma_channels) {
		return -EINVAL;
	}

	if (!data->reload_compatible) {
		return -ENOTSUP;
	}

	if (sys_test_bit((mem_addr_t)&data->channel_xfer_status, channel)) {
		/* Transfer in progress */
		return -EIO;
	}

	if (data_size < 0) {
		return -EINVAL;
	}

	if (size > (GPDMA_DESC_MAX_TRANSFER_SIZE - data_size)) {
		return -EINVAL;
	}

	cfg->channel_reg->CHANNEL_CONFIG[channel].SRC_ADDR_REG_CHNL = src;
	cfg->channel_reg->CHANNEL_CONFIG[channel].DEST_ADDR_REG_CHNL = dst;
	cfg->channel_reg->CHANNEL_CONFIG[channel].CHANNEL_CTRL_REG_CHNL_b.DMA_BLK_SIZE = size;

	return 0;
}

static int siwx91x_gpdma_start(const struct device *dev, uint32_t channel)
{
	struct gpdma_siwx91x_data *data = dev->data;

	if (channel >= data->dma_ctx.dma_channels) {
		return -EINVAL;
	}

	if (!atomic_test_bit(data->dma_ctx.atomic, channel)) {
		return -EINVAL;
	}

	if (sys_test_bit((mem_addr_t)&data->channel_xfer_status, channel)) {
		/* Transfer in progress */
		return -EIO;
	}

	if (RSI_GPDMA_DMAChannelTrigger(data->gpdma_handle, channel)) {
		return -EINVAL;
	}

	sys_set_bit((mem_addr_t)&data->channel_xfer_status, channel);

	return 0;
}

static int siwx91x_gpdma_stop(const struct device *dev, uint32_t channel)
{
	struct gpdma_siwx91x_data *data = dev->data;

	if (channel >= data->dma_ctx.dma_channels) {
		return -EINVAL;
	}

	if (!atomic_test_bit(data->dma_ctx.atomic, channel)) {
		return -EINVAL;
	}

	if (RSI_GPDMA_AbortChannel(data->gpdma_handle, channel)) {
		return -EINVAL;
	}

	sys_clear_bit((mem_addr_t)&data->channel_xfer_status, channel);

	return 0;
}

static int siwx91x_gpdma_get_status(const struct device *dev, uint32_t channel,
				    struct dma_status *stat)
{
	struct gpdma_siwx91x_data *data = dev->data;

	if (channel >= data->dma_ctx.dma_channels) {
		return -EINVAL;
	}

	if (!atomic_test_bit(data->dma_ctx.atomic, channel)) {
		return -EINVAL;
	}

	stat->busy = sys_test_bit((mem_addr_t)&data->channel_xfer_status, channel);
	stat->dir = data->chan_info[channel].xfer_direction;

	return 0;
}

bool siwx91x_gpdma_chan_filter(const struct device *dev, int channel, void *filter_param)
{
	ARG_UNUSED(dev);

	if (!filter_param) {
		return false;
	}

	if (*(int *)filter_param == channel) {
		return true;
	} else {
		return false;
	}
}

static int siwx91x_gpdma_init(const struct device *dev)
{
	const struct gpdma_siwx91x_config *cfg = dev->config;
	struct gpdma_siwx91x_data *data = dev->data;
	RSI_GPDMA_INIT_T gpdma_init = {
		.pUserData = NULL,
		.baseG = (uint32_t)cfg->reg,
		.baseC = (uint32_t)cfg->channel_reg,
		.sramBase = (uint32_t)data->gpdma_desc_pool->buffer,
	};
	int ret;

	data->gpdma_handle = RSI_GPDMA_Init(&gpdma_context, &gpdma_init);
	if (data->gpdma_handle == NULL) {
		return -EIO;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret) {
		return ret;
	}

	cfg->irq_configure();

	return 0;
}

static void siwx91x_gpdma_isr(const struct device *dev)
{
	const struct gpdma_siwx91x_config *cfg = dev->config;
	struct gpdma_siwx91x_data *data = dev->data;
	volatile uint32_t channel = find_lsb_set(cfg->reg->GLOBAL.INTERRUPT_REG) - 1;
	volatile uint32_t channel_int_status = cfg->reg->GLOBAL.INTERRUPT_STAT_REG;
	uint32_t channel_shift = channel * 4;
	uint32_t abort_mask = (BIT(0) | BIT(3)) << channel_shift;
	uint32_t desc_fetch_mask = BIT(1) << channel_shift;
	uint32_t done_mask = BIT(2) << channel_shift;

	//printf("ISR***\n");
	if (channel_int_status & abort_mask) {
		RSI_GPDMA_AbortChannel(data->gpdma_handle, channel);
		cfg->reg->GLOBAL.INTERRUPT_STAT_REG = abort_mask;
		//printf("abort***\n");
	}

	if (channel_int_status & desc_fetch_mask) {
		cfg->reg->GLOBAL.INTERRUPT_STAT_REG = desc_fetch_mask;
		if (data->chan_info[channel].dma_callback) {
			data->chan_info[channel].dma_callback(dev, data->chan_info[channel].cb_data,
							      channel, 0);
		}
		//printf("desc fetch***\n");
	}

	if (channel_int_status & done_mask) {
		sys_mem_blocks_free_contiguous(data->gpdma_desc_pool,
					       (void *)data->chan_info[channel].chan_base_desc,
					       data->chan_info[channel].desc_count);
		data->chan_info[channel].chan_base_desc = NULL;
		data->chan_info[channel].desc_count = 0;
		cfg->reg->GLOBAL.INTERRUPT_STAT_REG = done_mask;
		if (data->chan_info[channel].dma_callback) {
			data->chan_info[channel].dma_callback(dev, data->chan_info[channel].cb_data,
							      channel, 0);
		}
		//printf("done***\n");
	}

	sys_clear_bit((mem_addr_t)&data->channel_xfer_status, channel);
}

static DEVICE_API(dma, siwx91x_gpdma_api) = {
	.config = siwx91x_gpdma_configure,
	.reload = siwx91x_gpdma_reload,
	.start = siwx91x_gpdma_start,
	.stop = siwx91x_gpdma_stop,
	.get_status = siwx91x_gpdma_get_status,
	.chan_filter = siwx91x_gpdma_chan_filter,
};

#define SIWX91X_GPDMA_INIT(inst)                                                                   \
	static ATOMIC_DEFINE(dma_channels_atomic_##inst,                                           \
			     DT_INST_PROP(inst, silabs_dma_channel_count));                        \
	SYS_MEM_BLOCKS_DEFINE_STATIC(desc_pool_##inst, 32,                                         \
				     CONFIG_GPDMA_SILABS_SIWX91X_DESCRIPTOR_COUNT, 512);           \
	static struct gpdma_siwx91x_channel_info                                                   \
		chan_info_##inst[DT_INST_PROP(inst, silabs_dma_channel_count)];                    \
	static struct gpdma_siwx91x_data gpdma_data_##inst = {                                     \
		.dma_ctx.magic = DMA_MAGIC,                                                        \
		.dma_ctx.dma_channels = DT_INST_PROP(inst, silabs_dma_channel_count),              \
		.dma_ctx.atomic = dma_channels_atomic_##inst,                                      \
		.gpdma_desc_pool = &desc_pool_##inst,                                              \
		.chan_info = chan_info_##inst,                                                     \
	};                                                                                         \
	static void siwx91x_gpdma_irq_configure_##inst(void)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(inst, irq), DT_INST_IRQ(inst, priority),                   \
			    siwx91x_gpdma_isr, DEVICE_DT_INST_GET(inst), 0);                       \
		irq_enable(DT_INST_IRQ(inst, irq));                                                \
	}                                                                                          \
                                                                                                   \
	static const struct gpdma_siwx91x_config gpdma_cfg_##inst = {                              \
		.reg = (GPDMA_G_Type *)DT_INST_REG_ADDR(inst),                                     \
		.channel_reg = (GPDMA_C_Type *)DT_INST_PROP(inst, silabs_channel_reg_base),        \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_PHA(inst, clocks, clkid),          \
		.irq_configure = siwx91x_gpdma_irq_configure_##inst,                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_gpdma_init, NULL, &gpdma_data_##inst,                  \
			      &gpdma_cfg_##inst, POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,            \
			      &siwx91x_gpdma_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_GPDMA_INIT)
