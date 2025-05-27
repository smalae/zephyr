/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>

#define RESOLUTION	32U

#define SAMPLE_NO	1024

#if (RESOLUTION == 16)
#define SIZE	SAMPLE_NO * 2
#else
#define SIZE	SAMPLE_NO * 4
#endif

#define NUM_BLOCKS 5

K_MEM_SLAB_DEFINE(tx_mem_slab, SIZE, NUM_BLOCKS, 32);
K_MEM_SLAB_DEFINE(rx_mem_slab, SIZE, NUM_BLOCKS, 32);

void *tx_block[NUM_BLOCKS];

#if (RESOLUTION == 16)
uint16_t rx_block[SAMPLE_NO];
#else
uint32_t rx_block[SAMPLE_NO];
#endif

#if (RESOLUTION == 16)
static void fill_test_buf(uint16_t *tx_block)
#else
static void fill_test_buf(uint32_t *tx_block)
#endif
{

	for (int i = 0; i < SAMPLE_NO; i++) {
		tx_block[i] = i + 1;
	}
}

int i2s_record(const struct device *i2s_dev, void *buf)
{
	int ret;
	void *mem_block = NULL;
	size_t mem_block_size = 0;

	ret = i2s_read(i2s_dev, &mem_block, &mem_block_size);
	if (ret) {
		printf("I2S read error\n");
		return ret;
	}
	
	memcpy(buf, mem_block, mem_block_size);
	k_mem_slab_free(&rx_mem_slab, mem_block);
	
	return 0;
}

int main(void)
{
	struct i2s_config i2s_cfg;
	int ret;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2s_tx));
	uint32_t tx_idx;
	int i;

	if (!device_is_ready(dev_i2s)) {
		printf("I2S device not ready\n");
		return -ENODEV;
	}
	/* Configure I2S stream */
	i2s_cfg.word_size = RESOLUTION;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = SIZE;
	i2s_cfg.timeout = 2000;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
			| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_mem_slab;

	for (tx_idx = 0; tx_idx < NUM_BLOCKS; tx_idx++) {
		ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block[tx_idx],
				       K_FOREVER);
		if (ret < 0) {
			printf("Failed to allocate TX block\n");
			return ret;
		}
#if (RESOLUTION == 16)
		fill_test_buf((uint16_t *)tx_block[tx_idx]);
#else
		fill_test_buf((uint32_t *)tx_block[tx_idx]);
#endif
	}

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S Tx stream\n");
		return ret;
	}

	i2s_cfg.mem_slab = &rx_mem_slab;
	ret = i2s_configure(dev_i2s, I2S_DIR_RX, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S Rx stream\n");
		return ret;
	}

	/* Trigger the I2S transmission */
	ret = i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("Could not trigger I2S rx\n");
		return ret;
	}

	tx_idx = 0;
	ret = i2s_write(dev_i2s, tx_block[tx_idx++], SIZE);

	/* Trigger the I2S transmission */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("Could not trigger I2S tx\n");
		return ret;
	}

	for (; tx_idx < NUM_BLOCKS; ) {
		ret = i2s_write(dev_i2s, tx_block[tx_idx++], SIZE);
		if (ret < 0) {
			printf("Could not write TX buffer %d\n", tx_idx);
			return ret;
		}
	}

	/* Drain TX queue */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		printf("Could not trigger I2S tx\n");
		return ret;
	}

	printf("All I2S blocks written\n");

	ret = i2s_record(dev_i2s, rx_block);
	if (ret) {
		return ret;
	}

	for (i = 0;i < SAMPLE_NO; i++) {
		if(rx_block[i] != 0)
		break;
	}

	if (i != SAMPLE_NO) {
		printf("Read test Fail\n");
		printf("i = %d\n", i);
		return -1;		
	}

	printf("Read test pass\n");

	return 0;
}
