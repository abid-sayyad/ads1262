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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/err.h>

#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>


#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer.h>

#include <asm/unaligned.h>

/* Commands */
#define ADS1262_CMD_NOP		0x00
#define ADS1262_CMD_RESET       0x06
#define ADS1262_CMD_START1      0x08
#define ADS1262_CMD_RDATA1      0x12
#define ADS1262_CMD_RREG        0x20
#define ADS1262_CMD_WREG        0x40
#define ADS1262_CMD_STOP1       0x0A

/* Registers */
#define ADS1262_REG_ID          0x00
#define ADS1262_REG_POWER       0x01
#define ADS1262_REG_INTERFACE   0x02
#define ADS1262_REG_MODE0       0x03
#define ADS1262_REG_MODE1       0x04
#define ADS1262_REG_MODE2       0x05
#define ADS1262_REG_INPMUX      0x06
#define ADS1262_REG_OFCAL0      0x07
#define ADS1262_REG_OFCAL1      0x08
#define ADS1262_REG_OFCAL2      0x09
#define ADS1262_REG_FSCAL0      0x0A
#define ADS1262_REG_FSCAL1      0x0B
#define ADS1262_REG_FSCAL2      0x0C
#define ADS1262_REG_IDACMUX     0x0D
#define ADS1262_REG_IDACMAG     0x0E
#define ADS1262_REG_REFMUX      0x0F
#define ADS1262_REG_TDACP       0x10
#define ADS1262_REG_TDACN       0x11
#define ADS1262_REG_GPIOCON     0x12
#define ADS1262_REG_GPIODIR     0x13
#define ADS1262_REG_GPIODAT     0x14

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

enum ads126_id {
	ADS1262_ID,
};

struct ads1262_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

struct ads1262_private {
	const struct ads1262_chip_info *chip_info;
	struct gpio_desc *reset_gpio;
	struct spi_device *spi;
	struct mutex lock;
	/* Buffer for incoming SPI data*/
	//u8 rx_buffer[ADS1262_SPI_RDATA_BUFFER_SIZE] __aligned(IIO_DMA_MINALIGN);

	u32 buffer[ADS1262_MAX_CHANNELS + sizeof(s64)/sizeof(u32)] __aligned(8);
	/* Buffer for synchronous SPI exchanges (read/write registers)*/
	u8 data[ADS1262_SPI_CMD_BUFFER_SIZE] __aligned(IIO_DMA_MINALIGN);
};

#define ADS1262_CHAN(index)				\
{							\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = index,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.scan_index = index,				\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = 32,				\
		.storagebits = 32,			\
		.endianness = IIO_CPU,			\
	},						\
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

static const struct ads1262_chip_info ads1262_chip_info_tbl[] = {
	[ADS1262_ID] = {
		.channels = ads1262_channels,
		.num_channels = ARRAY_SIZE(ads1262_channels),
	},
};

static int ads1262_write_cmd(struct iio_dev *indio_dev, u8 command)
{	struct ads1262_private *priv = iio_priv(indio_dev);

	priv->data[0] = command;

	return spi_write(priv->spi, &priv->data[0], 1);
}

static int ads1262_reg_write(struct iio_dev *indio_dev, u8 reg, u8 data)
{
	struct ads1262_private *priv = iio_priv(indio_dev);

	priv->data[0] = ADS1262_CMD_WREG | reg;
	priv->data[1] = 0;
	priv->data[2] = data;
	return spi_write(priv->spi, &priv->data[0], 3);
}

static int ads1262_reset(struct iio_dev *indio_dev)
{
	struct ads1262_private *priv = iio_priv(indio_dev);

	if(priv->reset_gpio){
		gpiod_set_value(priv->reset_gpio, 0);
		udelay(200);
		gpiod_set_value(priv->reset_gpio, 1);
	} else {
		return ads1262_write_cmd(indio_dev, ADS1262_CMD_RESET);
	}
	return 0;
};

static int ads1262_read(struct iio_dev *indio_dev)
{
	struct ads1262_private *priv = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &priv->data[0],
			.len = 5,
			.cs_change = 1,
		}, {
			.tx_buf = &priv->data[1],
			.rx_buf = &priv->data[1],
			.len = 5,
		},
	};

	priv->data[0] = ADS1262_CMD_RDATA1;
	memset(&priv->data[1], ADS1262_CMD_NOP, sizeof(priv->data) - 1);

	printk("Data buffer before sending : %x, %x, %x, %x, %x, %x\n",priv->data[0] ,priv->data[1] ,priv->data[2] , priv->data[3] , priv->data[4] , priv->data[5]);


	ret = spi_sync_transfer(priv->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	printk("Data incomming : %x, %x, %x, %x\n",priv->data[2] , priv->data[3] , priv->data[4] , priv->data[5]);
	u32 data_debug = priv->data[2] | priv->data[3] | priv->data[4] | priv->data[5];
	printk("Data 0x%08x\n",data_debug);
	
	return get_unaligned_be32(&priv->data[2]);
}

static int ads1262_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ads1262_private *priv = iio_priv(indio_dev);
	int ret;

	mutex_lock(&priv->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ads1262_reg_write(indio_dev, ADS1262_REG_INPMUX,
					chan->channel);
		if (ret){
			dev_err(&priv->spi->dev, "Set ADC CH failed \n");
			goto out;
		}

		ret = ads1262_write_cmd(indio_dev, ADS1262_CMD_START1);
		if (ret) {
			dev_err(&priv->spi->dev, "Start conversion dailed\n");
			goto out;
		}
		ret = ads1262_read(indio_dev);
		if (ret) {
			dev_err(&priv->spi->dev, "Read ADC failed\n");
			printk("ADC read fail: %d\n", ret);
			goto out;
		}
		*val = ret;

		ret = ads1262_write_cmd(indio_dev, ADS1262_CMD_STOP1);
		if (ret) {
			dev_err(&priv->spi->dev, "Stop conversions failed\n");
			goto out;
		}

		return IIO_VAL_INT;
		break;
	default:
		return -EINVAL;
		break;
	}
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static const struct iio_info ads1262_info = {
	.read_raw = ads1262_read_raw,
};

static irqreturn_t ads1262_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads1262_private *priv = iio_priv(indio_dev);
	int scan_index, j=0;
	int ret;

	for_each_set_bit(scan_index, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = ads1262_reg_write(indio_dev, ADS1262_REG_INPMUX,
					scan_index);
		if (ret)
			dev_err(&priv->spi->dev,
				"Set ADC CH failed\n");
		ret = ads1262_write_cmd(indio_dev, ADS1262_CMD_START1);
		if (ret)
			dev_err(&priv->spi->dev,
				"stop ADC conversions fialed\n");
	 	priv->buffer[j] = ads1262_read(indio_dev);
		ret = ads1262_write_cmd(indio_dev, ADS1262_CMD_STOP1);
		if (ret)
			dev_err(&priv->spi->dev,
				"stop ADC conversions fialed\n");
		j++;
	 }

	iio_push_to_buffers_with_timestamp(indio_dev, priv->buffer,
	 				    pf->timestamp);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ads1262_probe(struct spi_device *spi)
{
	struct ads1262_private *ads1262_priv;
	struct iio_dev *indio_dev;
	const struct spi_device_id *spi_id = spi_get_device_id(spi);
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ads1262_priv));
	if (indio_dev == NULL)
		return -ENOMEM;

	ads1262_priv = iio_priv(indio_dev);

	ads1262_priv->reset_gpio = devm_gpiod_get_optional(&spi->dev,
							   "reset",
							   GPIOD_OUT_LOW);
	if(IS_ERR(ads1262_priv->reset_gpio))
		dev_info(&spi->dev, "Reset GPIO not defined\n");

	ads1262_priv->chip_info = &ads1262_chip_info_tbl[spi_id->driver_data];
	
	ads1262_priv->spi = spi;

	indio_dev->name = spi_id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads1262_priv->chip_info->channels;
	indio_dev->num_channels = ads1262_priv->chip_info->num_channels;
	indio_dev->info = &ads1262_info;

	mutex_init(&ads1262_priv->lock);

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL,
						ads1262_trigger_handler, NULL);
	if (ret) {
		dev_err(&spi->dev, "iio triggered buffer setup failed\n");
		return ret;
	}

	ads1262_reset(indio_dev);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_device_id ads126_id[] = {
	{ "ads1262", ADS1262_ID },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads126_id);

static const struct of_device_id ads1262_of_match[] = {
	{ .compatible = "ti,ads1262" },
	{ },
};
MODULE_DEVICE_TABLE(of, ads1262_of_match);

static struct spi_driver ads1262_driver = {
	.driver = {
		.name = "ads1262",
		.of_match_table = ads1262_of_match,
	},
	.probe = ads1262_probe,
	.id_table = ads126_id,
};
module_spi_driver(ads1262_driver)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI ADS1262 ADC");
MODULE_AUTHOR("Sayyad Abid <sayyad.abid16@gmail.com>");
