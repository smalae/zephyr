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

#define SAMPLE_NO 	1024

#if (RESOLUTION == 16)
	uint16_t test_buff[SAMPLE_NO];
#else
	uint32_t test_buff[SAMPLE_NO];
#endif

#define NUM_BLOCKS 3
#if (RESOLUTION == 16)
#define BLOCK_SIZE (SAMPLE_NO * 2)
#else
#define BLOCK_SIZE (SAMPLE_NO * 4)
#endif

#if (RESOLUTION == 16)
uint16_t rx_block[SAMPLE_NO];
#else
uint32_t rx_block[SAMPLE_NO];
#endif

K_MEM_SLAB_DEFINE(rx_mem_slab, BLOCK_SIZE, NUM_BLOCKS, 32);

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
	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2s_rx));
	int i;

	k_thread_access_grant(k_current_get(), &rx_mem_slab);
	k_object_access_grant(dev_i2s, k_current_get());

	if (!device_is_ready(dev_i2s)) {
		printf("I2S device not ready\n");
		return -ENODEV;
	}
	/* Configure I2S stream */
	i2s_cfg.word_size = RESOLUTION;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
			| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &rx_mem_slab;

	for (int k = 0;k < SAMPLE_NO; k++) {
		test_buff[k] = k + 1;
	}

	ret = i2s_configure(dev_i2s, I2S_DIR_RX, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S stream\n");
		return ret;
	}

	/* Trigger the I2S transmission */
	ret = i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("Could not trigger I2S rx\n");
		return ret;
	}

#if 1
	ret = i2s_record(dev_i2s, rx_block);
	if (ret) {
		return ret;
	}

	for (i = 0;i < SAMPLE_NO; i++) {
		if(rx_block[i] != test_buff[i])
		//if(rx_block[i] != 0)
		break;
	}

	if (i != SAMPLE_NO) {
		printf("Read test Fail\n");
		printf("i = %d\n", i);
		return -1;		
	}
#endif

#if 1
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
#endif

	ret = i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_STOP);
	if (ret < 0) {
		printf("Could not stop I2S rx\n");
		return ret;
	}

#if 1
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
#endif

	printf("Read Test Pass\n");

	return 0;
}
