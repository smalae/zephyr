/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx91x_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>

#include "rsi_adc.h"
#include "rsi_bod.h"
#include "rsi_ipmu.h"
#include "rsi_system_config.h"
#include "aux_reference_volt_config.h"

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/* Number of channels available. */
#define SIWX91X_CHANNEL_COUNT            16
#define ADC_DEFAULT_CHANNEL              0
#define ADC_SINGLE_ENDED_IN              0
#define ADC_DIFFERENTIAL_IN              1
#define ADC_DEFAULT_SAMPLE_NO            1
#define ADC_DEFAULT_SAMPLING_RATE        100000
#define ADC_DEFAULT_CHANNEL_ENABLE_COUNT 1
#define MAX_IP_VOLT_SCDC                 2.4f

struct adc_siwx91x_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	int ref_voltage;
	void (*irq_configure)(void);
};

struct adc_siwx91x_data {
	const struct device *dev;
	struct adc_context ctx;
	int16_t *buffer;
	int16_t *repeat_buffer;
	uint32_t channels;
	adc_config_t adc_common_config;
	adc_ch_config_t adc_channel_config;
	uint8_t channel_init_status[SIWX91X_CHANNEL_COUNT];
};

static int adc_siwx91x_check_buffer_size(const struct adc_sequence *sequence,
					 uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		return -ENOMEM;
	}

	return 0;
}

static int start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_siwx91x_data *data = dev->data;
	uint8_t channel_count;
	uint32_t channels;
	int ret;
	int i;

	if (sequence->channels == 0) {
		return -EINVAL;
	}

	if (sequence->oversampling) {
		return -ENOTSUP;
	}

	channels = sequence->channels;
	channel_count = 0;
	while (channels) {
		i = find_lsb_set(channels) - 1;

		if (i >= SIWX91X_CHANNEL_COUNT) {
			return -EINVAL;
		}

		if (!data->channel_init_status[i]) {
			return -EINVAL;
		}
		channel_count++;
		channels &= ~BIT(i);
	}

	ret = adc_siwx91x_check_buffer_size(sequence, channel_count);
	if (ret < 0) {
		return ret;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);
	ret = adc_context_wait_for_completion(&data->ctx);

	return ret;
}

static int adc_siwx91x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_siwx91x_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);
	ret = adc_siwx91x_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

static int adc_siwx91x_channel_setup(const struct device *dev,
				     const struct adc_channel_cfg *channel_cfg)
{
	struct adc_siwx91x_data *data = dev->data;

	data->channel_init_status[channel_cfg->channel_id] = false;

	if (channel_cfg->channel_id >= SIWX91X_CHANNEL_COUNT) {
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_VDD_1) {
		return -ENOTSUP;
	}

	data->adc_channel_config.num_of_samples[channel_cfg->channel_id] = ADC_DEFAULT_SAMPLE_NO;
	data->adc_channel_config.sampling_rate[channel_cfg->channel_id] = ADC_DEFAULT_SAMPLING_RATE;

	data->adc_channel_config.pos_inp_sel[channel_cfg->channel_id] = channel_cfg->input_positive;
	if (channel_cfg->differential) {
		data->adc_channel_config.neg_inp_sel[channel_cfg->channel_id] =
			channel_cfg->input_negative;
		data->adc_channel_config.input_type[channel_cfg->channel_id] = ADC_DIFFERENTIAL_IN;
	} else {
		data->adc_channel_config.input_type[channel_cfg->channel_id] = ADC_SINGLE_ENDED_IN;
	}

	data->channel_init_status[channel_cfg->channel_id] = true;

	return 0;
}

static int adc_siwx91x_init(const struct device *dev)
{
	const struct adc_siwx91x_config *cfg = dev->config;
	struct adc_siwx91x_data *data = dev->data;
	float max_ip_volt_scdc = 2.4;
	float chip_volt = 0;
	float ref_voltage = (float)cfg->ref_voltage / 1000;
	int ret;

	data->adc_common_config.operation_mode = ADC_STATICMODE_ENABLE;
	data->adc_common_config.num_of_channel_enable = ADC_DEFAULT_CHANNEL_ENABLE_COUNT;

	data->adc_channel_config.channel = ADC_DEFAULT_CHANNEL;
	data->adc_channel_config.input_type[ADC_DEFAULT_CHANNEL] = ADC_SINGLE_ENDED_IN;
	data->adc_channel_config.num_of_samples[ADC_DEFAULT_CHANNEL] = ADC_DEFAULT_SAMPLE_NO;
	data->adc_channel_config.sampling_rate[ADC_DEFAULT_CHANNEL] = ADC_DEFAULT_SAMPLING_RATE;

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = ADC_Per_Channel_Init(data->adc_channel_config, data->adc_common_config, NULL);
	if (ret) {
		return -EIO;
	}

	chip_volt = RSI_BOD_SoftTriggerGetBatteryStatus();
	if (chip_volt < MAX_IP_VOLT_SCDC) {
		RSI_IPMU_ProgramConfigData(hp_ldo_voltsel);
	}

	ret = RSI_AUX_RefVoltageConfig(ref_voltage, chip_voltage);
	if (ret) {
		return -EIO;
	}

	adc_context_unlock_unconditionally(&data->ctx);

	cfg->irq_configure();

	return 0;
}

static void adc_siwx91x_start_channel(const struct device *dev)
{
	struct adc_siwx91x_data *data = dev->data;

	data->adc_channel_config.channel = find_lsb_set(data->channels) - 1;

	ADC_Per_ChannelConfig(data->adc_channel_config, data->adc_common_config);
	RSI_ADC_Start(AUX_ADC_DAC_COMP, adcConfig.operation_mode);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_siwx91x_data *data = CONTAINER_OF(ctx, struct adc_siwx91x_data, ctx);

	data->channels = ctx->sequence.channels;
	data->adc_common_config.num_of_channel_enable = 1;
	adc_siwx91x_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_siwx91x_data *data = CONTAINER_OF(ctx, struct adc_siwx91x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_siwx91x_isr(const struct device *dev)
{
	struct adc_siwx91x_data *data = dev->data;
	uint32_t intr_status;
	uint8_t current_ch = data->adc_channel_config.channel;
	int16_t sample;

	intr_status =
		RSI_ADC_ChnlIntrStatus(AUX_ADC_DAC_COMP); // update AUX_ADC_DAC_COMP with reg_base
	if (intr_status & ADC_STATIC_MODE_INTR &&
	    AUX_ADC_DAC_COMP->INTR_MASK_REG_b.ADC_STATIC_MODE_DATA_INTR_MASK == 0) {
		RSI_ADC_ChnlIntrMask(AUX_ADC_DAC_COMP, 0, ADC_STATICMODE_ENABLE);
		sample = RSI_ADC_ReadDataStatic(AUX_ADC_DAC_COMP, 1,
						data->adc_channel_config.input_type[current_ch]);
		*data->buffer++ = sample;
		data->channels &= ~BIT(data->channel_id);
		if (data->channels) {
			++data->adc_common_config.num_of_channel_enable;
			adc_siwx91x_start_channel(dev);
		} else {
			adc_context_on_sampling_done(&data->ctx, dev);
			data->adc_common_config.num_of_channel_enable = 0;
		}
	} else {
		adc_context_complete(&data->ctx, -EIO);
		data->adc_common_config.num_of_channel_enable;
		= 0;
	}
}

static DEVICE_API(adc, adc_siwx91x_driver_api) = {
	.channel_setup = adc_siwx91x_channel_setup,
	.read = adc_siwx91x_read,
};

#define SIWX91X_ADC_INIT(inst)                                                                     \
	static struct adc_siwx91x_data adc_data_##inst = {                                         \
		ADC_CONTEXT_INIT_TIMER(adc_data_##inst, ctx),                                      \
		ADC_CONTEXT_INIT_LOCK(adc_data_##inst, ctx),                                       \
		ADC_CONTEXT_INIT_SYNC(adc_data_##inst, ctx),                                       \
	};                                                                                         \
                                                                                                   \
	static void siwx91x_adc_irq_configure_##inst(void)                                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(inst, irq), DT_INST_IRQ(inst, priority), adc_siwx91x_isr,  \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ(inst, irq));                                                \
	}                                                                                          \
                                                                                                   \
	static const struct adc_siwx91x_config adc_cfg_##inst = {                                  \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_PHA(inst, clocks, clkid),          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.ref_voltage = DT_INST_PROP(inst, silabs_adc_ref_voltage),                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, adc_siwx91x_init, NULL, &adc_data_##inst, &adc_cfg_##inst,     \
			      PRE_KERNEL_1, CONFIG_ADC_INIT_PRIORITY, &adc_siwx91x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_ADC_INIT)
