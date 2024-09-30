// SPDX-License-Identifier: GPL-2.0
/*
 * IIO driver for Texas Instruments ADS1662  32-bit ADC
 *
 * Datasheet: https://www.ti.com/product/ADS1262
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/init.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <asm/unaligned.h>

/* Commands */
#define ADS1262_CMD_RESET       0x06
#define ADS1262_CMD_START1      0x08
#define ADS1262_CMD_RDATA1      0x12
#define ADS1262_CMD_RREG        0x20
#define ADS1262_CMD_WREG        0x40
#define ADS1262_CMD_STOP1       0x0A

/* Registers */
#define ADS1262_REG_ID          0x00
#define ADS1262_REG_INPMUX      0x06

/* ADS1262_SPECS */
#define ADS1262_MAX_CHANNELS    11
#define ADS1262_BITS_PER_SAMPLE 32
#define ADS1262_CLK_RATE_HZ     7372800
#define ADS1262_CLOCKS_TO_USECS(x)  \
	(DIV_ROUND_UP((x) * MICROHZ_PER_HZ, ADS1262_CLK_RATE_HZ))

/* The Read/Write commands require 4 tCLK to encode and decode, for speeds
 * 2x the clock rate, these commands would require extra time between the
 * command byte and the data. A simple way to tacke this issue is by
 * limiting the SPI bus transfer speed while accessing registers.
 */
#define ADS1262_SPI_BUS_SPEED_SLOW  ADS1262_CLK_RATE_HZ

/* For reading and writing we need a buffer of size 3bytes*/
#define ADS1262_SPI_CMD_BUFFER_SIZE 3

/* Read data buffer size for
 * 1 status byte - 4 byte data (32 bit) - 1 byte checksum / CRC
 */
#define ADS1262_SPI_RDATA_BUFFER_SIZE 6

/*Single ended Internal tempsensor ADC read*/
#define ADS1262_DATA_TEMP_SENS  0xBA
/* Single ended AIN0 ADC read*/
#define ADS1262_DATA_AIN0_SENS  0x0A

struct ads1262 {
	struct spi_device *spi;
	/* Buffer for synchronous SPI exchanges (read/write registers)*/
	u8 cmd_buffer[ADS1262_SPI_CMD_BUFFER_SIZE];
	/* Buffer for incoming SPI data*/
	u8 rx_buffer[ADS1262_SPI_RDATA_BUFFER_SIZE] __aligned(IIO_DMA_MINALIGN);
};

#define ADS1262_CHAN(index)
{
	.type = IIO_VOLTAGE
	.indexed = 1,
	.channel = index,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.scan_index = index,
	.scan_type = {
		.sign = 'u',
		.realbits = 32,
		.storagebtis = 32,
	},
}

static const struct iio_chan_spec ads1262_channels[] = {
	ADS1262_CHAN(0),
	ADS1262_CHAN(1),
	ADS1262_CHAN(2),
	ADS1262_CHAN(3),
	ADS1262_CHAN(4),
	ADS1262_CHAN(5),
	ADS1262_CHAN(6),
	ADS1262_CHAN(7),
	ADS1262_CHAN(8),
	ADS1262_CHAN(9),
};

static int ads1262_write_cmd(struct ads1262 *priv, u8 command)
{
	struct spi_transfer xfer = {
		.tx_buf = priv->cmd_buffer,
		.rx_buf = priv->rx_buffer,
		.len = ADS1262_SPI_RDATA_BUFFER_SIZE,
		.speed_hz = ADS1262_CLK_RATE_HZ,
		.delay = {
			.value = 5,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};

	priv->cmd_buffer[0] = command;

	int ret = spi_sync_transfer(priv->spi, &xfer, 1);
	return ret;
}

static int ads1262_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ads1262 *priv = context;
	struct spi_transfer reg_write_xfer = {
		.tx_buf = priv->cmd_buffer,
		.rx_buf = priv->cmd_buffer,
		.len = 3,
		.speed_hz = ADS1262_CLK_RATE_HZ,
		.delay = {
			.value = 5,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};

	priv->cmd_buffer[0] = ADS1262_CMD_WREG | reg;
	priv->cmd_buffer[1] = 0;
	priv->cmd_buffer[2] = val;
	int ret = spi_sync_transfer(priv->spi, &reg_write_xfer, 1);
	return ret;
}

static int ads1262_reg_read(void *context, unsigned int reg)
{
	unsigned int val;
	struct ads1262 *priv = context;
	struct spi_transfer reg_read_xfer = {
		.tx_buf = priv->cmd_buffer,
		.rx_buf = priv->cmd_buffer,
		.len = 3,
		.speed_hz = ADS1262_CLK_RATE_HZ,
		.delay = {
			.value = 5,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};
	int ret;

	priv->cmd_buffer[0] = ADS1262_CMD_RREG | reg;
	priv->cmd_buffer[1] = 0;
	priv->cmd_buffer[2] = 0;

	ret = spi_sync_transfer(priv->spi, &reg_read_xfer, 1);
	if (ret)
		return ret;

	val = priv->cmd_buffer[2];

	return 0;
}

static int ads1262_init(struct iio_dev *indio_dev)
{
	struct ads1262 *priv = iio_priv(indio_dev);
	int ret;

	ret = ads1262_write_cmd(priv, ADS1262_CMD_RESET);
	if (ret)
		return ret;

	fsleep(10000);

	/* Setting up the MUX to read the internal temperature sensor*/
	ads1262_reg_write(priv, ADS1262_REG_INPMUX, ADS1262_DATA_TEMP_SENS);
	ret = ads1262_reg_read(priv, ADS1262_REG_INPMUX);
	if (ret)
		return ret;

	/* Starting the ADC conversions*/
	return ads1262_write_cmd(priv, ADS1262_CMD_START1);
}

static int ads1262_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ads1262 *spi = iio_priv(indio_dev);
	s32 data;
	int ret;

	ret = ads1262_init(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ads1262_write_cmd(spi, ADS1262_CMD_RDATA1);
		if (ret != 0)
			return -EINVAL;

		data = spi->rx_buffer[1] | spi->rx_buffer[2] |
			spi->rx_buffer[3] | spi->rx_buffer[4];
		*val = sign_extend64(get_unaligned_be32(spi->rx_buffer + 1),
				     ADS1262_BITS_PER_SAMPLE - 1);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ads1262_info = {
	.read_raw = ads1262_read_raw,
};

static void ads1262_stop(void *ptr)
{
	struct ads1262 *adc = (struct ads1262 *)ptr;

	ads1262_write_cmd(adc, ADS1262_CMD_STOP1);
}

static int ads1262_probe(struct spi_device *spi)
{
	struct ads1262 *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	spi->mode = SPI_MODE_1;

	spi->max_speed_hz = ADS1262_SPI_BUS_SPEED_SLOW;

	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads1262_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1262_channels);
	indio_dev->info = &ads1262_info;

	ret = ads1262_reg_read(adc, ADS1262_REG_ID);
	if (ret)
		return ret;

	if (adc->rx_buffer[2] != ADS1262_REG_ID)
		dev_err_probe(&spi->dev, -EINVAL, "Wrong device ID 0x%x\n",
			      adc->rx_buffer[2]);

	ret = devm_add_action_or_reset(&spi->dev, ads1262_stop, adc);
	if (ret)
		return ret;

	ret = ads1262_init(indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_device_id ads1262_id_table[] = {
	{ "ads1262" },
	{}
};
MODULE_DEVICE_TABLE(spi, ads1262_id_table);

static const struct of_device_id ads1262_of_match[] = {
	{ .compatible = "ti,ads1262" },
	{},
};
MODULE_DEVICE_TABLE(of, ads1262_of_match);

static struct spi_driver ads1262_driver = {
	.driver = {
		.name = "ads1262",
		.of_match_table = ads1262_of_match,
	},
	.probe = ads1262_probe,
	.id_table = ads1262_id_table,
};
module_spi_driver(ads1262_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayyad Abid <sayyad.abid16@gmail.com>");
MODULE_DESCRIPTION("TI ADS1262 ADC");
