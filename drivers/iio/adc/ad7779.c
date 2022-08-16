/*
 * AD7779 ADC
 *
 * Copyright 2022 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#define AD7779_SPI_READ_CMD			0x80
#define AD7779_SPI_WRITE_CMD			0x00

#define AD7779_ENABLE_CRC			0x3F
#define AD7779_ENABLE_SAR			0x29
#define AD7779_DISABLE_SAR			0x09
#define AD7779_PDB_SAR				0x2C
#define AD7779_PDB_SAR_OFF			0x24
#define AD7779_ENABLE_SD			0x29
#define AD7779_DISABLE_SD			0x09


#define AD7779_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD7779_REG_CH_DISABLE			0x08			// Disable clocks to ADC channel
#define AD7779_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD7779_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD7779_REG_DOUT_FORMAT			0x14			// Data out format
#define AD7779_REG_ADC_MUX_CONFIG		0x15			// Main ADC meter and reference Mux control
#define AD7779_REG_GLOBAL_MUX_CONFIG		0x16			// Global diagnostics mux
#define AD7779_REG_GPIO_CONFIG			0x17			// GPIO config
#define AD7779_REG_GPIO_DATA			0x18			// GPIO Data
#define AD7779_REG_BUFFER_CONFIG_1		0x19			// Buffer Config 1
#define AD7779_REG_BUFFER_CONFIG_2		0x1A			// Buffer Config 2
#define AD7779_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD7779_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD7779_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD7779_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD7779_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD7779_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD7779_REG_CH_ERR_REG(ch)		(0x4C + (ch))		// Channel Status Register
#define AD7779_REG_CH0_1_SAT_ERR		0x54			// Channel 0/1 DSP errors
#define AD7779_REG_CH2_3_SAT_ERR		0x55			// Channel 2/3 DSP errors
#define AD7779_REG_CH4_5_SAT_ERR		0x56			// Channel 4/5 DSP errors
#define AD7779_REG_CH6_7_SAT_ERR		0x57			// Channel 6/7 DSP errors
#define AD7779_REG_CHX_ERR_REG_EN		0x58			// Channel 0-7 Error Reg Enable
#define AD7779_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable
#define AD7779_REG_STATUS_REG_1			0x5D			// Error Status Register 1
#define AD7779_REG_STATUS_REG_2			0x5E			// Error Status Register 2
#define AD7779_REG_STATUS_REG_3			0x5F			// Error Status Register 3
#define AD7779_REG_SRC_N_MSB			0x60			// Decimation Rate (N) MSB
#define AD7779_REG_SRC_N_LSB			0x61			// Decimation Rate (N) LSB
#define AD7779_REG_SRC_IF_MSB			0x62			// Decimation Rate (IF) MSB
#define AD7779_REG_SRC_IF_LSB			0x63			// Decimation Rate (IF) LSB
#define AD7779_REG_SRC_UPDATE			0x64			// SRC load source and load update

#define AD7779_CH_GAIN(x)			(((x) & 0x3) << 6)
#define AD7779_CH_RX				(1 << 4)

#define AD777X_POWER_MODE_HIGH			(1 << 6)
#define AD777X_POWER_MODE_LOW			(0 << 6)

#define AD7779_LOW_POWER_MODE			0
#define AD7779_HIGH_POWER_MODE			1

/* AD7779_REG_DOUT_FORMAT */
#define AD7779_DOUT_FORMAT(x)			(((x) & 0x3) << 6)
#define AD7779_DOUT_HEADER_FORMAT		(1 << 5)
#define AD7779_DCLK_CLK_DIV(x)			(((x) & 0x7) << 1)

#define AD777X_DOUT_FORMAT_00			2
#define AD777X_DOUT_FORMAT_01			4
#define AD777X_DOUT_FORMAT_10			8

#define AD777X_DCLK_DIV_HIGH			8
#define AD777X_DCLK_DIV_LOW			4
#define AD777X_MAX_DCLK_DIV			128
#define AD777X_MIN_DCLK_DIV_4_LINES		4
#define AD777X_MIN_DCLK_DIV_2_LINES		2
#define AD777X_MIN_DCLK_DIV_1_LINE		1

#define CLK_MAX_RATE				8192000

/* AXI CONTROL REGS VALUES FOR DATA LINES */
#define AXI_CTRL_4_LINES			0x400
#define AXI_CTRL_2_LINES			0x200
#define AXI_CTRL_1_LINE				0x100

#define AD7779_CRC8_POLY	0x07
DECLARE_CRC8_TABLE(ad7779_crc8_table);

enum ad777x_data_lines {
	AD777x_4LINES,
	AD777x_2LINES,
	AD777x_1LINE,

};

enum ad777x_filter {
	AD777X_SINC3,
	AD777X_SINC5,
};

struct ad7779_state {
	struct spi_device 	*spi;
	struct clk 		*mclk;
	struct regulator	*vref;
	struct gpio_chip	gpiochip;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*start_gpio;
	unsigned int		sampling_freq;
	unsigned int 		power_mode;
	unsigned int 		decimation;
	enum ad777x_data_lines 	data_lines;
	enum ad777x_filter	filter_enabled;
	
	unsigned int 		crc_enabled;
	const char 		**labels;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reg_rx_buf[3] ____cacheline_aligned;
	u8			reg_tx_buf[3];
};

static const char * const ad777x_filter_type[] = {
	[AD777X_SINC3] = "sinc3_filter",
	[AD777X_SINC5] = "sinc5_filter",
};

static const char * const ad777x_data_lines_modes[] = {
	[AD777x_4LINES] = "4_data_lines",
	[AD777x_2LINES] = "2_data_lines",
	[AD777x_1LINE]  = "1_data_line",
};

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

static bool ad7779_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct ad7779_state *ad7779_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if(ad7779_has_axi_adc(&indio_dev->dev)) {
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int ad777x_read_label(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan, char *label)
{
	struct ad7779_state *st = ad7779_get_data(indio_dev);

	return sprintf(label, "%s\n", st->labels[chan->channel]);
}

static int ad7779_spi_read(struct iio_dev *indio_dev, u8 reg, u8 *rbuf) {
	struct ad7779_state *spi_st = ad7779_get_data(indio_dev);
	int ret;
	int length = 2;
	u8 crc_buf[2];
	u8 exp_crc = 0;
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
			.rx_buf = spi_st->reg_rx_buf,
		},
	};

	if(spi_st->crc_enabled)
		length = 3;
	reg_read_tr[0].len = length;	

	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = crc8(ad7779_crc8_table, 
					spi_st->reg_tx_buf, 2, 0);

	ret = spi_sync_transfer(spi_st->spi, reg_read_tr, 
				ARRAY_SIZE(reg_read_tr));
	
	crc_buf[0] = AD7779_SPI_READ_CMD | (reg & 0x7F);
	crc_buf[1] = spi_st->reg_rx_buf[1];
	exp_crc = crc8(ad7779_crc8_table, crc_buf, 2, 0);
	if(spi_st->crc_enabled && (exp_crc != spi_st->reg_rx_buf[2])) {
		dev_err(&spi_st->spi->dev, "Bad CRC %x, expected %x",
			spi_st->reg_rx_buf[2], exp_crc);
		return -EINVAL;
	}
	*rbuf = spi_st->reg_rx_buf[1];

	return ret;
} 

int ad7779_spi_write(struct iio_dev *indio_dev, u8 reg, u8 val) {
	struct ad7779_state *spi_st = ad7779_get_data(indio_dev);
	int length = 2;
	struct spi_transfer reg_write_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
		},
	};

	if(spi_st->crc_enabled)
		length = 3;
	reg_write_tr[0].len = length;

	spi_st->reg_tx_buf[0] = AD7779_SPI_WRITE_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = val;
	spi_st->reg_tx_buf[2] = crc8(ad7779_crc8_table,
					spi_st->reg_tx_buf, 2, 0);
	
	return spi_sync_transfer(spi_st->spi, reg_write_tr,
				ARRAY_SIZE(reg_write_tr));
}

static int ad7779_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int writeval,
				unsigned int *readval) {
	int ret;

	if(readval)
		ret = ad7779_spi_read(indio_dev, reg, (u8 *) readval);
	else
		ret = ad7779_spi_write(indio_dev, reg, writeval);

	return ret;
}

static int ad777x_set_sampling_frequency(struct iio_dev *indio_dev,
					unsigned int sampling_freq)
{
	struct ad7779_state *st = ad7779_get_data(indio_dev);
	int ret;
	unsigned long dec;
	unsigned int dclk;
	unsigned int dec_div;
	unsigned int div;
	int decimal;
	u8 msb, lsb;

	if (st->power_mode) {
		dec = (unsigned long) clk_get_rate(st->mclk) / (4 * sampling_freq);
		dec_div = AD777X_DCLK_DIV_HIGH;
	} else {
		dec = (unsigned long) clk_get_rate(st->mclk) / (8 * sampling_freq);
		dec_div = AD777X_DCLK_DIV_LOW;
	}

	lsb = (int) dec & 0xFF;
	msb = ((int) dec >> 8) & 0xFF;
	switch (st->data_lines) {
	case AD777x_4LINES:
		div = dec / (dec_div * AD777X_DOUT_FORMAT_00);
		if ( div < AD777X_MIN_DCLK_DIV_4_LINES)
			return -EINVAL;
		break;
	case AD777x_2LINES:
		div = dec / (dec_div * AD777X_DOUT_FORMAT_01);
		if ( div < AD777X_MIN_DCLK_DIV_2_LINES)
			return -EINVAL;
		break;
	case AD777x_1LINE:
		div = dec / (dec_div * AD777X_DOUT_FORMAT_10);
		if ( div < AD777X_MIN_DCLK_DIV_1_LINE)
			return -EINVAL;
		break;
	}

	if ( div > AD777X_MAX_DCLK_DIV )
		return -EINVAL;

	if ( div % 4 != 0 )
		div -= div % 4;
	dev_info(&st->spi->dev, "The calculated div is %x", div);

	switch (div)
	{
	case 1:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x20);
		break;
	case 2:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x22);
		break;
	case 4:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x24);
		break;
	case 8:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x26);
		break;
	case 16:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x28);
		break;
	case 32:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x2A);
		break;
	case 64:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x2C);
		break;
	case 128:
		ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 
			0x2E);
		break;
	
	default:
		return -EINVAL;
	}
	dev_info(&st->spi->dev, "The mclk value from device is %x", clk_get_rate(st->mclk));
	dev_info(&st->spi->dev, "The calculated decimation is %x", dec);
	dev_info(&st->spi->dev, "The calculated div is %x", div);
	dev_info(&st->spi->dev, "The msb calculated value was msb %x and lsb %x", msb, lsb);
	
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_N_LSB, lsb);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_N_MSB, msb);

	dev_info(&st->spi->dev, "The calculated decimal is %lf", dec);
	if (dec != ((int) dec)) {
		decimal = (int) (dec - ((int) dec)) * 65536;
		dev_info(&st->spi->dev, "The calculated decimal is %x", decimal);
		lsb = (int) decimal & 0xFF;
		msb = ((int) decimal >> 8) & 0xFF;
		dev_info(&st->spi->dev, "The msb calculated IF msb %x and lsb %x", msb, lsb);
		ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_IF_LSB, lsb);
		ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_IF_MSB, msb);
	}
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x1);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x0);

	if(ret)
		return ret;
	else
		st->sampling_freq = sampling_freq;
		// TODO: also set decimation?
	return 0;
}

static int ad7779_set_power_mode(struct iio_dev *indio_dev, 
				unsigned int power_mode)
{
	int ret;
	u8 temp;

	if(power_mode) {
		ret = ad7779_spi_read(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, &temp);
		temp |= AD777X_POWER_MODE_HIGH;
		ret = ad7779_spi_write(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, temp);
	} else {
		ret = ad7779_spi_read(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, &temp);
		temp |= AD777X_POWER_MODE_LOW;
		ret = ad7779_spi_write(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, temp);
	}

	return ret;
}

static int ad777x_set_data_lines(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				unsigned int mode)
{
	struct ad7779_state *st = ad7779_get_data(indio_dev);
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);

	switch (mode) {
	case AD777x_4LINES:
		ad777x_set_sampling_frequency(indio_dev, 8000);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL, AXI_CTRL_4_LINES);
		break;
	case AD777x_2LINES:
		ad777x_set_sampling_frequency(indio_dev, 4000);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL, AXI_CTRL_2_LINES);	
		break;
	case AD777x_1LINE:
		ad777x_set_sampling_frequency(indio_dev, 2000);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL, AXI_CTRL_1_LINE);
		break;
	default:
		return -EINVAL;
	}

	ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x01);
	ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x00);
  


	st->data_lines = mode;

	return 0;
}

static int ad777x_get_data_lines(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan)
{
	struct ad7779_state *st = ad7779_get_data(indio_dev);

	return st->data_lines;
}

static int ad7779_spi_sar_mode_en(struct iio_dev *indio_dev)
{
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_2,
				AD7779_ENABLE_SAR);
	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_1,
				AD7779_PDB_SAR);

	return ret;
}

static int ad7779_spi_sar_mode_disable(struct iio_dev *indio_dev)
{
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_2,
				AD7779_DISABLE_SAR);
	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_1,
				AD7779_PDB_SAR_OFF);

	return ret;
}

static int ad7779_spi_sar_readback(struct iio_dev *indio_dev, int *readback) {
	int ret;
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int length = 2;
	struct spi_transfer sar_readback_tr[] = {
		{
			.rx_buf = spi_st->reg_rx_buf,
			.tx_buf = spi_st->reg_tx_buf,
		}
	};

	if (spi_st->crc_enabled)
		length = 3;
	sar_readback_tr[0].len = length;
	
	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD;
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = crc8(ad7779_crc8_table,
					spi_st->reg_tx_buf, 2, 0);

	ret = spi_sync_transfer(spi_st->spi, sar_readback_tr,
				ARRAY_SIZE(sar_readback_tr));

	*readback = ((0x0F & spi_st->reg_rx_buf[0]) << 8)
			| spi_st->reg_rx_buf[1];

	return ret;
}

static int ad7779_data_read_en(struct iio_dev *indio_dev) {
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_3,
				AD7779_ENABLE_SD);

	return ret;
}

static int ad7779_data_read_disable(struct iio_dev *indio_dev) {
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_3,
				AD7779_DISABLE_SD);

	return ret;
}

static int ad7779_sigma_delta_data(struct iio_dev *indio_dev) {
	int ret;
	struct ad7779_state *spi_st = ad7779_get_data(indio_dev);
	struct spi_transfer sd_readback_tr[] = {
		{
			.rx_buf = spi_st->reg_rx_buf,
			.tx_buf = spi_st->reg_tx_buf,
			.len = 3,
		}
	};

	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD;
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = 0;

	ret = spi_sync_transfer(spi_st->spi, sd_readback_tr,
				ARRAY_SIZE(sd_readback_tr));

	dev_info(&spi_st->spi->dev, "The data gathered is %x %x %x ", spi_st->reg_rx_buf[0], spi_st->reg_rx_buf[1], spi_st->reg_rx_buf[2]);

	return ret;
}

static int ad7779_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask) 
{
	int ret;
	struct ad7779_state *st = ad7779_get_data(indio_dev);
	u8 temp;

	switch(mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ad7779_spi_read(indio_dev,
					AD7779_CH_GAIN(chan->channel),
					&temp);
		if(ret)
			return ret;
		*val = temp;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		ret = ad7779_spi_read(indio_dev,
					AD7779_REG_CH_SYNC_OFFSET(chan->channel),
					&temp);
		if(ret)
			return ret;
		*val = temp;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad7779_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val,
			int val2,
			long mask) 
{
	int ret;

	switch(mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ad777x_set_sampling_frequency(indio_dev, val);
		return ret;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ad7779_spi_write(indio_dev,
					AD7779_CH_GAIN(chan->channel),
					val);
			return ret;
			return ret;
		return 0;
		return ret;
		return 0;
	case IIO_CHAN_INFO_OFFSET:
		ret = ad7779_spi_write(indio_dev,
					AD7779_REG_CH_SYNC_OFFSET(chan->channel),
					val);
			return ret;
			return ret;
		return 0;
		return ret;
		return 0;
	}

	return -EINVAL;
}

static int ad7779_soft_reset(struct iio_dev *indio_dev)
{
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	u8 reset_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = &reset_buf[0],
			.len = 8,
		},
	};

	return spi_sync_transfer(spi_st->spi, reg_read_tr,
				ARRAY_SIZE(reg_read_tr));
}

static const struct iio_info ad7779_info = {
	.read_raw = ad7779_read_raw,
	.write_raw = ad7779_write_raw,
	.debugfs_reg_access = &ad7779_reg_access,
};

static const struct iio_enum ad777x_data_lines_enum = {
	.items = ad777x_data_lines_modes,
	.num_items = ARRAY_SIZE(ad777x_data_lines_modes),
	.get = ad777x_get_data_lines,
	.set = ad777x_set_data_lines,
};

static const struct iio_chan_spec_ext_info ad777x_ext_info[] = {
	IIO_ENUM("data_lines", IIO_SHARED_BY_ALL, &ad777x_data_lines_enum),
	IIO_ENUM_AVAILABLE_SHARED("data_lines", IIO_SHARED_BY_ALL, &ad777x_data_lines_enum),
	{ },
};

#define AD777x_CHAN(index)							\
	{									\
		.type = IIO_VOLTAGE,						\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET)  |	\
					BIT(IIO_CHAN_INFO_HARDWAREGAIN),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.address = index,						\
		.indexed = 1,							\
		.channel = index,						\
		.scan_index = index,						\
		.ext_info = ad777x_ext_info,					\
		.scan_type = {							\
			.sign = 's',						\
			.realbits = 24,						\
			.storagebits = 32,					\
		},								\
	}


static const struct iio_chan_spec ad7779_channels[] = {
	AD777x_CHAN(0),
	AD777x_CHAN(1),
	AD777x_CHAN(2),
	AD777x_CHAN(2),
	AD777x_CHAN(4),
	AD777x_CHAN(5),
	AD777x_CHAN(6),
	AD777x_CHAN(7),
};

static const struct axiadc_chip_info conv_chip_info = {
	.name = "ad7779_axi_adc",
	.max_rate = 2048000UL,
	.num_channels = 8,
	.channel[0] = AD777x_CHAN(0),
	.channel[1] = AD777x_CHAN(1),
	.channel[2] = AD777x_CHAN(2),
	.channel[3] = AD777x_CHAN(3),
	.channel[4] = AD777x_CHAN(4),
	.channel[5] = AD777x_CHAN(5),
	.channel[6] = AD777x_CHAN(6),
	.channel[7] = AD777x_CHAN(7),
};


static void ad7779_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void ad7779_reg_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

int ad7779_gpio_init(struct ad7779_state *st) 
{
	st->gpiochip.label = "ad777x_gpios";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 2;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.owner = THIS_MODULE;

	return gpiochip_add_data(&st->gpiochip, st);
}

static int ad7779_register_axi(struct ad7779_state *st)
{
	struct axiadc_converter *conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->clk = st->mclk;
	conv->chip_info = &conv_chip_info;
	conv->reg_access = &ad7779_reg_access;
	conv->write_raw = &ad7779_write_raw;
	conv->read_raw = &ad7779_read_raw;
	//conv->read_label = &ad777x_read_label;
	conv->phy = st;
	spi_set_drvdata(st->spi, conv);
	dev_info(&st->spi->dev, "in the register axi function");

	return 0;
}

static int ad7779_register(struct ad7779_state *st, struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	dev_info(&st->spi->dev, "in the register function");

	//indio_dev->dev.parent = &st->spi->dev;
	indio_dev->name = "ad7779";
	indio_dev->info = &ad7779_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = ad7779_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7779_channels);
	dev_info(&st->spi->dev, "before buffer alloc");

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						&dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ad7779_spi_write(indio_dev,
			AD7779_REG_GENERAL_USER_CONFIG_3,
			0x80);

	ad7779_spi_write(indio_dev,
			AD7779_REG_GEN_ERR_REG_1_EN,
			AD7779_ENABLE_CRC);
	st->crc_enabled = true;

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static int ad777x_powerup(struct iio_dev *indio_dev)
{
	int ret;
	struct ad7779_state *st = iio_priv(indio_dev);

	ret = ad7779_spi_write(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, 0x74);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 0x24);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_ADC_MUX_CONFIG, 0x40);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_N_LSB, 0x40);
	st->sampling_freq = 8000;
	st->power_mode = 1;
	st->data_lines = AD777x_4LINES;

	if(ret)
		return ret;

	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x1);
	usleep_range(10, 15);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x0);
	usleep_range(10, 15);

	gpiod_set_value(st->start_gpio, 0);
	usleep_range(10, 15);
	gpiod_set_value(st->start_gpio, 1);
	usleep_range(10, 15);
	gpiod_set_value(st->start_gpio, 0);
	usleep_range(10, 15);

	return ret;
}

static int ad7779_probe(struct spi_device *spi) {
	struct iio_dev *indio_dev;
	struct ad7779_state *ad7779_st;
	int ret;
	// u8 temp, temp2;
	// int rbuf;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ad7779_st));
	if (!indio_dev)
		return -ENOMEM;
	
	ad7779_st = iio_priv(indio_dev);

	ad7779_st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(ad7779_st->vref))
		return PTR_ERR(ad7779_st->vref);

	ret = regulator_enable(ad7779_st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad7779_reg_disable, ad7779_st->vref);
	if (ret)
		return ret;

	ad7779_st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(ad7779_st->mclk))
		return PTR_ERR(ad7779_st->mclk);

	ret = clk_prepare_enable(ad7779_st->mclk);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad7779_clk_disable, ad7779_st->mclk);
	if (ret)
		return ret;

	ad7779_st->reset_gpio = devm_gpiod_get(&spi->dev, "resetn",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ad7779_st->reset_gpio))
		return PTR_ERR(ad7779_st->reset_gpio);

	ad7779_st->start_gpio = devm_gpiod_get(&spi->dev, "startn",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ad7779_st->start_gpio))
		return PTR_ERR(ad7779_st->start_gpio);
	
	if (ad7779_st->reset_gpio) {
		gpiod_set_value(ad7779_st->reset_gpio, 1);
		usleep_range(225, 230);
		gpiod_set_value(ad7779_st->reset_gpio, 0);
		usleep_range(1, 2);
	}

	// TODO: check if it relevant to add labels
	// ad7779_st->labels = devm_kzalloc(&ad7779_st->spi->dev,
	// 			   sizeof(*ad7779_st->labels), GFP_KERNEL);

	crc8_populate_msb(ad7779_crc8_table, AD7779_CRC8_POLY);
	ad7779_st->spi = spi;

	ad7779_st->crc_enabled = false;
	ad777x_powerup(indio_dev);

	// ret = ad7779_register(ad7779_st, indio_dev);
	if (device_property_present(&spi->dev, "dmas"))
		ret = ad7779_register(ad7779_st, indio_dev);
	else
		ret = ad7779_register_axi(ad7779_st);

	return ret;
}

static struct spi_driver ad7779_driver = {
	.driver = {
		.name = "ad7779",
	},
	.probe = ad7779_probe,
};
module_spi_driver(ad7779_driver);

MODULE_AUTHOR("Ramona Alexandra Nechita <ramona.nechita@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7779 ADC");
MODULE_LICENSE("GPL v2");
