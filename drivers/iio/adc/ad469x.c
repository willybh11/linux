// SPDX-License-Identifier: GPL-2.0+
/*
 * AD4696 SPI ADC driver
 *
 * Copyright 2022 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/sysfs.h>
#include <linux/pwm.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

/* AD469x registers */
#define AD469x_REG_IF_CONFIG_A		0x000
#define AD469x_REG_IF_CONFIG_B		0x001
#define AD469x_REG_DEVICE_TYPE		0x003
#define AD469x_REG_DEVICE_ID_L		0x004
#define AD469x_REG_DEVICE_ID_H		0x005
#define AD469x_REG_SCRATCH_PAD		0x00A
#define AD469x_REG_VENDOR_L		0x00C
#define AD469x_REG_VENDOR_H		0x00D
#define AD469x_REG_LOOP_MODE		0x00E
#define AD469x_REG_IF_CONFIG_C		0x010
#define AD469x_REG_IF_STATUS		0x011
#define AD469x_REG_STATUS		0x014
#define AD469x_REG_ALERT_STATUS1	0x015
#define AD469x_REG_ALERT_STATUS2	0x016
#define AD469x_REG_ALERT_STATUS3	0x017
#define AD469x_REG_ALERT_STATUS4	0x018
#define AD469x_REG_CLAMP_STATUS1	0x01A
#define AD469x_REG_CLAMP_STATUS2	0x01B
#define AD469x_REG_SETUP		0x020
#define AD469x_REG_REF_CTRL		0x021
#define AD469x_REG_SEQ_CTRL		0x022
#define AD469x_REG_AC_CTRL		0x023
#define AD469x_REG_STD_SEQ_CONFIG	0x024
#define AD469x_REG_GPIO_CTRL		0x026
#define AD469x_REG_GP_MODE		0x027
#define AD469x_REG_GPIO_STATE		0x028
#define AD469x_REG_TEMP_CTRL		0x029
#define AD469x_REG_CONFIG_IN(x)		((x & 0x0F) | 0x30)
#define AD469x_REG_THRESHOLD_UB(x)  ((x << 1) | 0x40)
#define AD469x_REG_THRESHOLD_LB(x)  ((x << 1) | 0x60)
#define AD469x_REG_HYST_IN(x)		((x << 1) | 0x80)
#define AD469x_REG_OFFSET_IN(x)       ((x << 1) | 0x0A0)
#define AD469x_REG_GAIN_IN(x)       ((x << 1) | 0x0C0)
#define AD469x_REG_AS_SLOT(x)		((x & 0x7F) | 0x100)

/* 5-bit SDI Conversion Mode Commands */
#define AD469x_CMD_REG_CONFIG_MODE		(0x0A << 3)
#define AD469x_CMD_SEL_TEMP_SNSOR_CH		(0x0F << 3)
#define AD469x_CMD_CONFIG_CH_SEL(x)		((0x10 | (0x0F & x)) << 3)

/* Set instuction mode command */
#define AD469x_INST_MODE_MASK		(0x01 << 7)
#define AD469x_SET_INST_MODE(x)		(x << 7)

/* Software reset command */
#define AD469x_SW_RST_MASK		(0x01 << 7 | 0x01)
#define AD469x_SW_RST_CMD		(0x01 << 7 | 0x01)
/* Status Register Mask */
#define AD469x_REG_STATUS_RESET_MASK     (0x01 << 5)

/* AD469x_ SPI_CONFIG_B */
#define AD469x_ADDR_LEN_MASK			(0x01 << 3)
#define AD469x_SET_ADDR_LEN(x)			((x & 0x01) << 3)

/* AD469x_REG_SETUP */
#define AD469x_SETUP_IF_MODE_MASK		(0x01 << 2)
#define AD469x_SETUP_IF_MODE_CONV		(0x01 << 2)
#define AD469x_SETUP_CYC_CTRL_MASK		(0x01 << 1)
#define AD469x_SETUP_CYC_CTRL_SINGLE(x)		((x & 0x01) << 1)

/* AD469x_REG_REF_CTRL */
#define AD469x_REG_REF_VREF_SET_MASK		(0x07 << 2)
#define AD469x_REG_REF_VREF_SET(x)          ((x & 0x07) << 2)
#define AD469x_REG_REF_VREF_REFHIZ_MASK		(0x07 << 1)
#define AD469x_REG_REF_VREF_REFHIZ(x)		((x & 0x01) << 1)
#define AD469x_REG_REF_VREF_REFBUF_MASK		0x01
#define AD469x_REG_REF_VREF_REFBUF(x)		(x & 0x01)

/* AD469x_REG_GP_MODE */
#define AD469x_GP_MODE_BUSY_GP_EN_MASK		(0x01 << 1)
#define AD469x_GP_MODE_BUSY_GP_EN(x)		((x & 0x01) << 1)
#define AD469x_GP_MODE_BUSY_GP_SEL_MASK		(0x01 << 5)
#define AD469x_GP_MODE_BUSY_GP_SEL(x)		((x & 0x01) << 5)

/* AD469x_REG_SEQ_CTRL */
#define AD469x_SEQ_CTRL_STD_SEQ_EN_MASK		(0x01 << 7)
#define AD469x_SEQ_CTRL_STD_SEQ_EN(x)		((x & 0x01) << 7)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK	(0x7f << 0)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS(x)		((x & 0x7f) << 0)

/* AD469x_REG_TEMP_CTRL */
#define AD469x_REG_TEMP_CTRL_TEMP_EN_MASK	(0x01 << 0)
#define AD469x_REG_TEMP_CTRL_TEMP_EN(x)		((x & 0x01) << 0)

/* AD469x_REG_AS_SLOT */
#define AD469x_REG_AS_SLOT_INX(x)		((x & 0x0f) << 0)

/* AD469x_REG_IF_CONFIG_C */
#define AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK	(0x01 << 5)
#define AD469x_REG_IF_CONFIG_C_MB_STRICT(x)	((x & 0x01) << 5)

/* AD469x_REG_CONFIG_INn */
#define AD469x_REG_CONFIG_IN_OSR_MASK		(0x03 << 0)
#define AD469x_REG_CONFIG_IN_OSR(x)		((x & 0x03) << 0)
#define AD469x_REG_CONFIG_IN_HIZ_EN_MASK	(0x01 << 3)
#define AD469x_REG_CONFIG_IN_HIZ_EN(x)		((x & 0x01) << 3)
#define AD469x_REG_CONFIG_IN_PAIR_MASK		(0x03 << 4)
#define AD469x_REG_CONFIG_IN_PAIR(x)		((x & 0x03) << 4)
#define AD469x_REG_CONFIG_IN_MODE_MASK		(0x01 << 6)
#define AD469x_REG_CONFIG_IN_MODE(x)		((x & 0x01) << 6)
#define AD469x_REG_CONFIG_IN_TD_EN_MASK		(0x01 << 7)
#define AD469x_REG_CONFIG_IN_TD_EN(x)		((x & 0x01) << 7)

#define AD469x_CHANNEL(x)			(NO_OS_BIT(x) & 0xFFFF)
#define AD469x_CHANNEL_NO			16
#define AD469x_SLOTS_NO				0x80
#define AD469x_CHANNEL_TEMP			16

#define KHz 1000
#define MHz (1000 * KHz)

enum ad469x_ids {
	ID_AD4695,
	ID_AD4696,
	ID_AD4697,
	ID_AD4698,
};

enum ad469x_instr_mode {
	AD469x_STREAM_INSTR_MODE,
	AD469x_SINGLE_INSTR_MODE,
};

enum ad469x_reg_access {
	AD469x_BYTE_ACCESS,
	AD469x_WORD_ACCESS,
};

enum ad469x_addr_len {
	AD469x_15BIT_ADDR,
	AD469x_7BIT_ADDR,
};

enum ad469x_osr_ratios {
	AD469x_OSR_1,
	AD469x_OSR_4,
	AD469x_OSR_16,
	AD469x_OSR_64
};

enum ad469x_channel_sequencing {
	/** Single cycle read */
	AD469x_single_cycle,
	/** Two cycle read */
	AD469x_two_cycle,
	/** Sequence trough channels, standard mode */
	AD469x_standard_seq,
	/** Sequence trough channels, advanced mode */
	AD469x_advanced_seq,
};

enum ad469x_pin_pairing {
	AD469x_INx_REF_GND,
	AD469x_INx_COM,
	AD469x_INx_EVEN_ODD
};

enum ad469x_busy_gp_sel {
	/** Busy on gp0 */
	AD469x_busy_gp0 = 0,
	/** Busy on gp3 */
	AD469x_busy_gp3 = 1,
};

enum ad469x_mode {
	AD469x_REG_CONFIG_MODE = 0,
	AD469x_CONV_MODE = 1,
	AD469x_AUTOCYCLE_MODE = 2,
};

struct ad469x_chip_info {
	const char	*name;
	enum ad469x_ids dev_id;
	u8		num_channels;
};

struct ad469x_state {
	struct spi_device *spi;
	struct device *dev;
	const struct ad469x_chip_info *chip_info;
	struct mutex lock;
	struct regulator *vref;
	enum ad469x_osr_ratios adv_seq_osr_resol[AD469x_CHANNEL_NO];

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;

	uint8_t	data[3] ____cacheline_aligned;
	struct pwm_device *cnv;
	enum ad469x_mode mode;
	int sampling_freq;
	u16 gain[16];
	u16 offset[16];
	uint32_t conv_data[17] ____cacheline_aligned;

	unsigned long ref_clk_rate;
	struct clk *ref_clk;
};

const uint8_t ad469x_device_resol[] = {
	[AD469x_OSR_1] = 16,
	[AD469x_OSR_4] = 17,
	[AD469x_OSR_16] = 18,
	[AD469x_OSR_64] = 19
};

const unsigned int ad469x_device_max_freq[] = {
	[ID_AD4695] = 500 * KHz,
	[ID_AD4696] = MHz,
	[ID_AD4697] = 500 * KHz,
	[ID_AD4698] = MHz,
};

static int ad469x_set_sampling_freq(struct ad469x_state *st, int freq)
{
	int ret;
	struct pwm_state cnv_state;
	unsigned long long target, ref_clk_period_ps;

	target = DIV_ROUND_CLOSEST_ULL(st->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL,
			    st->ref_clk_rate);
	cnv_state.period = ref_clk_period_ps * target;
	cnv_state.duty_cycle = cnv_state.period / 2;
	cnv_state.time_unit = PWM_UNIT_PSEC;
	cnv_state.enabled = true;

	ret = pwm_apply_state(st->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	st->sampling_freq = DIV_ROUND_CLOSEST_ULL(st->ref_clk_rate, target);

	return 0;
}

static int ad469x_disable_cnv (struct ad469x_state *st)
{
	int ret;
	struct pwm_state cnv_state;
	cnv_state.enabled = true;
	ret = pwm_apply_state(st->cnv, &cnv_state);

	return ret;
}

static int ad469x_enable_cnv(struct ad469x_state *st)
{
	return ad469x_set_sampling_freq(st, st->sampling_freq);
}

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
			   struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}


static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int ad469x_spi_reg_write(struct ad469x_state *st, uint16_t reg_addr,
				uint8_t reg_data)
{

	struct spi_transfer t = {0};
	int ret;

	st->data[0] = ((reg_addr >> 8) & 0x7F);
	st->data[1] = 0xFF & reg_addr;
	st->data[2] = reg_data;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 3;
	t.bits_per_word = 8;

	ret = spi_sync_transfer(st->spi, &t, 1);

	return ret;
}

static int ad469x_spi_exit_conv(struct ad469x_state *st)
{
	return ad469x_spi_reg_write(st, 0x5000, 0x00);
}

static int ad469x_spi_reg_read(struct ad469x_state *st, uint16_t reg_addr,
			       uint8_t *reg_data)
{
	struct spi_transfer t = {0};
	int ret;

	st->data[0] = (1 << 7) | ((reg_addr >> 8) & 0x7F);
	st->data[1] = 0xFF & reg_addr;
	st->data[2] = 0xFF;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 3;
	t.bits_per_word = 8;

	ret = spi_sync_transfer(st->spi, &t, 1);
	*reg_data = st->data[2];

	return ret;
}

static int ad469x_spi_write_mask(struct ad469x_state *st, uint16_t reg_addr,
				 uint8_t mask, uint8_t data)
{
	uint8_t reg_data;
	int ret;

	ret = ad469x_spi_reg_read(st, reg_addr, &reg_data);
	if (ret != 0)
		return ret;

	reg_data &= ~mask;
	reg_data |= data;

	return ad469x_spi_reg_write(st, reg_addr, reg_data);
}

static int ad469x_set_reg_access_mode(struct ad469x_state *st,
				      enum ad469x_reg_access access)
{
	return ad469x_spi_write_mask(st, AD469x_REG_IF_CONFIG_C,
				     AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK,
				     AD469x_REG_IF_CONFIG_C_MB_STRICT(access));
}

static int ad469x_set_addr_len(struct ad469x_state *st,
			       enum ad469x_addr_len len)
{
	return ad469x_spi_write_mask(st, AD469x_REG_IF_CONFIG_B,
				     AD469x_ADDR_LEN_MASK,
				     AD469x_SET_ADDR_LEN(len));
}

static int ad469x_update_channels(struct ad469x_state *st, unsigned long mask)
{
	int ret;
	uint8_t reg_msb = (mask >> 8) & 0xFF;
	uint8_t reg_lsb = 0xFF & mask;

	if(st->mode != AD469x_REG_CONFIG_MODE)
		return -EBUSY;

	/* Update conversion channel 7-0 */
	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_STD_SEQ_CONFIG,
				   reg_lsb);
	if (ret)
		return ret;

	/* Update conversion channel 15-8 */
	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_STD_SEQ_CONFIG + 1,
				   reg_msb);
	if (ret)
		return ret;

	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_TEMP_CTRL,
				   AD469x_REG_TEMP_CTRL_TEMP_EN(mask >> 16));
	if (ret)
		return ret;

	return ret;
}

static int ad469x_set_busy(struct ad469x_state *st,
			   enum ad469x_busy_gp_sel gp_sel)
{
	int ret;

	ret = ad469x_spi_write_mask(st,
				    AD469x_REG_GP_MODE,
				    AD469x_GP_MODE_BUSY_GP_EN_MASK,
				    AD469x_GP_MODE_BUSY_GP_EN(1));
	if (ret != 0)
		return ret;


	ret = ad469x_spi_write_mask(st,
				    AD469x_REG_GP_MODE,
				    AD469x_GP_MODE_BUSY_GP_SEL_MASK,
				    AD469x_GP_MODE_BUSY_GP_SEL(gp_sel));
	if (ret != 0)
		return ret;


	return ret;
}

static int ad469x_enter_conversion_mode(struct ad469x_state *st)
{
	return ad469x_spi_write_mask(st,
				     AD469x_REG_SETUP,
				     AD469x_SETUP_IF_MODE_MASK,
				     AD469x_SETUP_IF_MODE_CONV);
}

static int ad469x_set_osr (struct ad469x_state *st, enum ad469x_osr_ratios osr,
			   uint16_t chn)
{
	int ret;

	ret = ad469x_spi_write_mask(st, AD469x_REG_CONFIG_IN(chn), GENMASK(1,0),
				    ad469x_device_resol[osr]);
	if (ret)
		return ret;
	st->adv_seq_osr_resol[chn] = osr;

	return 0;
}

static int ad469x_set_gain(struct ad469x_state *st, int chan_idx, u16 gain)
{
	int ret;

	if(chan_idx >= st->chip_info->num_channels)
		return -EINVAL;

	/* Update gain bits 7-0 */
	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_GAIN_IN(chan_idx),
				   gain & 0xFF);
	if (ret)
		return ret;

	/* Update gain bits 15-8 */
	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_GAIN_IN(chan_idx) + 1,
				   (gain >> 8) & 0xFF);
	if (ret)
		return ret;

	st->gain[chan_idx] = gain;

	return 0;
}

static int ad469x_set_offset(struct ad469x_state *st, int chan_idx, u16 offset)
{
	int ret;

	if(chan_idx >= st->chip_info->num_channels)
		return -EINVAL;

	/* Update offset bits 7-0 */
	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_OFFSET_IN(chan_idx),
				   offset & 0xFF);
	if (ret)
		return ret;

	/* Update offset bits 15-8 */
	ret = ad469x_spi_reg_write(st,
				   AD469x_REG_OFFSET_IN(chan_idx) + 1,
				   (offset >> 8) & 0xFF);
	if (ret)
		return ret;

	st->offset[chan_idx] = offset;

	return 0;
}

static int ad469x_set_ref_voltage (struct ad469x_state *st, int ref_voltage)
{
	uint8_t reg;

	if(ref_voltage >= 2400000 && ref_voltage <= 2750000)
		reg = 0;
	else if (ref_voltage > 2750000 && ref_voltage <= 3250000)
		reg = 1;
	else if (ref_voltage > 3250000 && ref_voltage <= 3750000)
		reg = 2;
	else if (ref_voltage > 3750000 && ref_voltage <= 4500000)
		reg = 3;
	else if (ref_voltage > 4500000 && ref_voltage <= 5100000)
		reg = 4;
	else
		return -EINVAL;

	return ad469x_spi_write_mask(st, AD469x_REG_IF_CONFIG_A,
				     AD469x_REG_REF_VREF_SET_MASK,
				     AD469x_REG_REF_VREF_SET(reg));
}

static int ad469x_reset (struct ad469x_state *st)
{
	int ret;
	ret = ad469x_spi_write_mask(st, AD469x_REG_IF_CONFIG_A,
				    AD469x_SW_RST_MASK,
				    AD469x_SW_RST_CMD);
	if (ret)
		return ret;

	/* Wait 310 microseconds after issuing a software reset */
	udelay(310);

	st->mode = AD469x_REG_CONFIG_MODE;
	return 0;
}

static int ad469x_set_instr_mode (struct ad469x_state *st,
				  enum ad469x_instr_mode mode)
{
	return  ad469x_spi_write_mask(st, AD469x_REG_IF_CONFIG_B,
				      AD469x_INST_MODE_MASK,
				      AD469x_SET_INST_MODE(mode));
}


#define AD469X_CHANNEL(_chan)         					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |	\
				BIT(IIO_CHAN_INFO_CALIBBIAS),		\
		.channel = _chan,	                                \
		.scan_index = _chan,					\
		.scan_type = {						\
			.sign = 'u',					\
			.storagebits = 32,				\
			.endianness = IIO_LE,				\
		},							\
	}								\


static struct iio_chan_spec ad469x_channels[] = {
	AD469X_CHANNEL(0),
	AD469X_CHANNEL(1),
	AD469X_CHANNEL(2),
	AD469X_CHANNEL(3),
	AD469X_CHANNEL(4),
	AD469X_CHANNEL(5),
	AD469X_CHANNEL(6),
	AD469X_CHANNEL(7),
	AD469X_CHANNEL(8),
	AD469X_CHANNEL(9),
	AD469X_CHANNEL(10),
	AD469X_CHANNEL(11),
	AD469X_CHANNEL(12),
	AD469X_CHANNEL(13),
	AD469X_CHANNEL(14),
	AD469X_CHANNEL(15),
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.info_mask_separate = 0,
		.channel = 0,
		.scan_index = 16,
		.scan_type = {
			.sign = 'u',
			.storagebits = 32,
			.endianness = IIO_LE,
		},
	},
};

static int ad469x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct ad469x_state *st = iio_priv(indio_dev);

	if(chan->type != IIO_VOLTAGE)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = st->gain[chan->channel];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = sign_extend32(st->offset[chan->channel], 15);
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int ad469x_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad469x_state *st = iio_priv(indio_dev);

	if(chan->type != IIO_VOLTAGE)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad469x_set_sampling_freq(st, val);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad469x_set_gain(st, chan->channel, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad469x_set_offset(st, chan->channel, (u16)val);
	default:
		return -EINVAL;
	}
}

static int ad469x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad469x_state *st = iio_priv(indio_dev);
	int ret;
	uint8_t data;

	if (readval)	{
		ret = ad469x_spi_reg_read(st, (uint16_t)reg, &data);
		*readval = data;

	} else {
		data = (uint8_t)writeval;
		ret = ad469x_spi_reg_write(st, (uint16_t)reg, data);
	}

	return ret;
}

static const struct iio_info ad469x_info = {
	.read_raw = &ad469x_read_raw,
	.write_raw = &ad469x_write_raw,
	.debugfs_reg_access = &ad469x_reg_access,
};

static void ad469x_regulator_disable(void *reg)
{
	regulator_disable(reg);
}


static void ad469x_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ad469x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad469x_state *st = iio_priv(indio_dev);
	int ret;


	ret = ad469x_update_channels(st, *indio_dev->active_scan_mask);
	if (ret)
		return ret;

	ret = ad469x_enter_conversion_mode(st);

	st->mode = AD469x_CONV_MODE;

	ad469x_enable_cnv(st);

	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = st->conv_data;
	st->spi_transfer.len = hweight32(*indio_dev->active_scan_mask);
	st->spi_transfer.bits_per_word = 32;

	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);

	spi_bus_lock(st->spi->master);

	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret < 0)
		return ret;

	spi_engine_offload_enable(st->spi, true);

	return 0;
}

static int ad469x_buffer_predisable(struct iio_dev *indio_dev)
{
	int ret;
	int i;
	struct ad469x_state *st = iio_priv(indio_dev);
	spi_engine_offload_enable(st->spi, false);

	ret = spi_bus_unlock(st->spi->master);
	if (ret)
		return ret;

	mdelay(1);
	ret =  ad469x_spi_exit_conv(st);
	if (ret) {
		printk (KERN_ERR "ad469 exit not working");
		return ret;
	}

	st->mode = AD469x_REG_CONFIG_MODE;

	return 0;
}

static const struct iio_buffer_setup_ops ad469x_buffer_setup_ops = {
	.postenable = &ad469x_buffer_postenable,
	.predisable = &ad469x_buffer_predisable,
};

static void ad469x_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static int ad469x_probe(struct spi_device *spi)
{
	struct ad469x_state *st;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	int ref_voltage;
	int ret;
	uint16_t i;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;
	st->dev = &spi->dev;
	st->chip_info = device_get_match_data(&spi->dev);
	if (!st->chip_info)
		st->chip_info = (const struct ad469x_chip_info *)spi_get_device_id(
					spi)->driver_data;

	mutex_init(&st->lock);

	st->vref = devm_regulator_get(st->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(st->dev, ad469x_regulator_disable, st->vref);
	if (ret)
		return ret;

	st->ref_clk = devm_clk_get(st->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(st->dev, ad469x_clk_disable,
				       st->ref_clk);
	if (ret)
		return ret;
	st->ref_clk_rate = clk_get_rate(st->ref_clk);

	st->cnv = devm_pwm_get(st->dev, "cnv");
	if (IS_ERR(st->cnv))
		return PTR_ERR(st->cnv);

	ret = devm_add_action_or_reset(st->dev, ad469x_pwm_diasble,
				       st->cnv);
	if (ret)
		return ret;

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->info = &ad469x_info;
	indio_dev->setup_ops = &ad469x_buffer_setup_ops;
	indio_dev->channels = ad469x_channels;
	indio_dev->num_channels = st->chip_info->num_channels;


	ret = ad469x_reset(st);
	if (ret)
		return ret;

	ret = ad469x_set_addr_len(st, AD469x_15BIT_ADDR);
	if (ret)
		return ret;

	ret = ad469x_set_instr_mode(st, AD469x_SINGLE_INSTR_MODE);
	if (ret)
		return ret;

	ret = ad469x_set_reg_access_mode(st, AD469x_BYTE_ACCESS);
	if (ret)
		return ret;

	ret = ad469x_set_busy(st, AD469x_busy_gp0);
	if (ret)
		return ret;

	ref_voltage = regulator_get_voltage(st->vref);
	ad469x_set_ref_voltage(st, ref_voltage);
	if (ret)
		return ret;

	ret = device_property_read_u32(&spi->dev, "adi,osr", &st->adv_seq_osr_resol[0]);
	if (ret)
		return ret;
	ret = ad469x_set_osr(st, st->adv_seq_osr_resol[0], 0);
	if (ret)
		return ret;

	for (i=0; i<st->chip_info->num_channels; i++) {
		ad469x_channels[i].scan_type.realbits = 16 + st->adv_seq_osr_resol[0];
		ad469x_channels[i].scan_type.shift = 16 - st->adv_seq_osr_resol[0];
		st->offset[i] = 0;
		st->gain[i] = 0x8000;
	}

	st->sampling_freq = 2000;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
			&dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct ad469x_chip_info ad469x_chip_info_data[] = {

	[ID_AD4695] = {
		.name = "ad4695",
		.dev_id = ID_AD4695,
		.num_channels = 16,
	},
	[ID_AD4696] = {
		.name = "ad4696",
		.dev_id = ID_AD4696,
		.num_channels = 16,
	},
	[ID_AD4697] = {
		.name = "ad4697",
		.dev_id = ID_AD4697,
		.num_channels = 8,
	},
	[ID_AD4698] = {
		.name = "ad4698",
		.dev_id = ID_AD4698,
		.num_channels = 8,
	},

};

static const struct spi_device_id ad469x_spi_ids[] = {
	{"ad4695", (kernel_ulong_t)&ad469x_chip_info_data[ID_AD4695]},
	{"ad4696", (kernel_ulong_t)&ad469x_chip_info_data[ID_AD4696]},
	{"ad4697", (kernel_ulong_t)&ad469x_chip_info_data[ID_AD4697]},
	{"ad4698", (kernel_ulong_t)&ad469x_chip_info_data[ID_AD4698]},
	{}
};
MODULE_DEVICE_TABLE(spi, ad469x_spi_ids);

static const struct of_device_id ad469x_dt_ids[] = {
	{
		.compatible = "adi,ad4695",
		.data = &ad469x_chip_info_data[ID_AD4695],
	},
	{
		.compatible = "adi,ad4696",
		.data = &ad469x_chip_info_data[ID_AD4696],
	},
	{
		.compatible = "adi,ad4697",
		.data = &ad469x_chip_info_data[ID_AD4697],
	},
	{
		.compatible = "adi,ad4698",
		.data = &ad469x_chip_info_data[ID_AD4698],
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ad469x_dt_ids);

static struct spi_driver ad469x_driver = {
	.driver = {
		.name   = "ad4696",
		.of_match_table = ad469x_dt_ids,
	},
	.probe          = ad469x_probe,
	.id_table	= ad469x_spi_ids,
};
module_spi_driver(ad469x_driver);

MODULE_AUTHOR("Ramona Bolboaca <ramona.bolboaca@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD469x ADC driver");
MODULE_LICENSE("GPL v2");