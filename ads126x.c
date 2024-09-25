#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi>
#include <linux/cleanup.h>
#include <linux/bitfield.h>
#include <math.h>

#include <linux/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <asm/unaligned.h>

/* Commands */
#define ADS126x_CMD_NOP         0x00
#define ADS126x_CMD_RESET       0x06
#define ADS126x_CMD_START1      0x08
#define ADS126x_CMD_STOP1       0x0A
#define ADS126x_CMD_START2      0x0C
#define ADS126x_CMD_STOP2       0x0E
#define ADS126x_CMD_RDATA1      0x12
#define ADS126x_CMD_RDATA2      0x14
#define ADS126x_CMD_SYOCAL1     0x16
#define ADS126x_CMD_SYGCAL1     0x17
#define ADS126x_CMD_SFOCAL1     0x19
#define ADS126x_CMD_SYOCAL2     0x1B
#define ADS126x_CMD_SYGCAL2     0x1C
#define ADS126x_CMD_SFOCAL2     0x1E
#define ADS126x_CMD_RREG        0x20
#define ADS126x_CMD_WREG        0x40

/* Registers */
#define ADS126x_REG_ID          0x00
#define ADS126x_REG_POWER       0x01
#define ADS126x_REG_INTERFACE   0x02
#define ADS126x_REG_MODE0       0x03
#define ADS126x_REG_MODE1       0x04
#define ADS126x_REG_MODE2       0x05
#define ADS126x_REG_INPMUX      0x06
#define ADS126x_REG_OFCAL0      0x07
#define ADS126x_REG_OFCAL1      0x08
#define ADS126x_REG_OFCAL2      0x09
#define ADS126x_REG_FSCAL0      0x0A
#define ADS126x_REG_FSCAL1      0x0B
#define ADS126x_REG_FSCAL2      0x0C
#define ADS126x_REG_IDACMUX     0x0D
#define ADS126x_REG_IDACMAG     0x0E
#define ADS126x_REG_REFMUX      0x0F
#define ADS126x_REG_TDACP       0x10
#define ADS126x_REG_TDACN       0x11
#define ADS126x_REG_GPIOCON     0x12
#define ADS126x_REG_GPIODIR     0x13
#define ADS126x_REG_GPIODAT     0x14
#define ADS126x_REG_ADC2CFG     0x16
#define ADS126x_REG_ADC2OFC0    0x17
#define ADS126x_REG_ADC2OFC1    0x18
#define ADS126x_REG_ADC2FSC0    0x19 
#define ADS126x_REG_ADC2FSC1    0x1A

/* ADS126x_HW_SPECS */
#define ADS126x_MAX_CHANNELS    11
#define ADS126x_BITS_PER_SAMPLE 32
#define ADS126x_CLK_RATE_HZ     7372800
#define ADS126x_CLOCKS_TO_USECS(x)  \
                (DIV_ROUND_UP((x) * MICROHZ_PER_HZ, ADS126x_CLK_RATE_HZ))

/* Channel select for single ended analog read
 * The ADC has two mux that help read differnetial values
 * Here '0xA' selects the AINCOM (MUX[0:3])as ground ref input
 * The addition of (n*10) selects the input channel.
 */
#define ADS126x_CHAnSEL(n)      (0xA + (n * 10))

/* The Read/Write commands require 4 tCLK to encode and decode, for speeds
 * 2x the clock rate, these commands would require extra time between the
 * command byte and the data. A simple way to tacke this issue is by
 * limiting the SPI bus transfer speed while accessing registers.
 */
#define ADS126x_SPI_BUS_SPEED_SLOW  ADS126x_CLK_RATE_HZ

/* For reading and writing we need a buffer of size 3bytes*/
#define ADS126x_SPI_CMD_BUFFER_SIZE 3

/* Read data buffer size for 
 * 1 status byte - 4 byte data (32 bit) - 1 byte checksum / CRC
 */
#define ADS126x_SPI_RDATA_BUFFER_SIZE(n)    (1 + (n) + 1)
#define ADS126x_SPI_RDATA_BUFFER_SIZE_MAX   \
                ADS126x_SPI_RDATA_BUFFER_SIZE(ADS126x_MAX_CHANNELS)

/*Single ended Internal tempsensor ADC read*/
#define ADS126x_DATA_TEMP_SENS  0xBA
/* Single ended AIN0 ADC read*/
#define ADS126x_DATA_AIN0_SENS  0x0A

enum {
        ADS1262,
        ADS1263,
};

struct ads162 {
        struct spi_device *spi;

        /* Buffer for synchronous SPI exchanges (read/write registers)*/
        u8 cmd_buffer[ADS126x_SPI_CMD_BUFFER_SIZE];
        /* Buffer for incoming SPI data*/
        __be32 rx_buffer[ADS126x_SPI_RDATA_BUFFER_SIZE_MAX] __aligned(IIO_DMA_MINALIGN);
        // /* Contains the RDATA command and zeroes to clock out*/
        // u8 tx_buffer[ADS126x_SPI_RDATA_BUFFER_SIZE_MAX];

};

/* Four bytes per sample (32 bit precision per channel)*/
#define ADS126x_OFFSET_INT_RX_BUFFER(index)             (4 * (index) + 4)

#define ADS1262_CHAN(index)                                             \
{                                                                       \
        .type = IIO_VOLATAGE,                                           \
        .indexed = 1,                                                   \
        .channel = index,                                               \
        .address = ADS126x_OFFSET_INT_RX_BUFFER(index),                 \
        .info_mask_separate =                                           \
               BIT(IIO_CHAN_INFO_RAW) |                                 \
               BIT(IIO_CHAN_INFO_SCALE),                                \
        .scan_index = index,                                            \
        .scan_type = {                                                  \
                .sign = 's',                                            \
                .realbits = ADS126x_BITS_PER_SAMPLE,                    \
                .storagebits = 32,                                      \
                .endianness = IIO_CPU,                                  \
        },
}

static const struct iio_chan_spec ads1262_channels[] = {
        ADS126x_CHAN(0),
        ADS126x_CHAN(1),
        ADS126x_CHAN(2),
        ADS126x_CHAN(3),
        ADS126x_CHAN(4),
        ADS126x_CHAN(5),
        ADS126x_CHAN(6),
        ADS126x_CHAN(7),
        ADS126x_CHAN(8),
        ADS126x_CHAN(9),
        ADS126x_CHAN(10)
};

static int ads1262_read_raw(struct iio_dev * indio_dev,
                            struct iio_chan_spec const * chan,
                            int *val, int *val2, long mask)
{
        struct ads1262 *spi = iio_priv(indio_dev);
        int ret;

        ret = ads1262_init();

        switch (mask) {
        case IIO_CHAN_INFO_RAW:
                ret = ads1262_write_cmd(spi, ADS126x_CMD_RDATA1);
                if(ret !=0 ) {
                        printk("dt-iio - Error reading ADC value!\n");
                        return ret;
                }
                *val = spi->rx_buffer[1];
                return IIO_VAL_INT;
        default:
                break;
        }
        return -EINVAL;
}

// static int ads1262_write_raw(struct iio_dev * indio_dev,
//                              struct iio_can_spec const *chan,
//                              itn val, int val2, long info)
// {
//         struct ads1262 *spi = iio_priv(indio_dev);

//         switch (mask) {
//         case IIO_CHAN_INFO_SAMP_FREQ:
//                 return ads1262_set_samp_freq(priv, val);
//         default:
//                 return -EINVAL;
//         }
// }
static int ads1262_write_cmd(struct ads1262 *priv, u8 command)
{
        struct spi_transfer xfer = {
                .tx_buf = priv->rx_buffer,
                .rx_buf = priv->rx_buffer,
                .len = ADS126x_SPI_RDATA_BUFFER_SIZE_MAX,
                .speed_hz = ADS126x_CLK_RATE_HZ,
                .delay = {
                        .vlaue = 2,
                        .unit = SPI_DELAY_UNIT_USECS,
                },
        };

        priv->cmd_buffer[0] = command;

        return spi_sync_transfer(priv->spi, &xfer, 1);
}

static int ads1262_reg_write(void *context, unsigned int reg, unsigned int val)
{
        struct ads1262 *priv = context;
        struct spi_transfer reg_write_xfer = {
                .tx_buf = priv->cmd_buffer,
                .rx_buf = priv->cmd_buffer,
                .len = 3,
                .speed_hz = ADS126x_CLK_RATE_HZ,
                .delay = {
                        .value = 2,
                        .unit = SPI_DELAY_UNIT_USECS,
                },
        };

        priv->cmd_buffer[0] = ADS126x_CMD_WREG | reg;
        priv->cmd_buffer[1] = 0;
        priv->cmd_buffer[2] = val;

        return spi_sync_transfer(priv->spi, &reg_write_xfer, 1);
}

static int ads1262_reg_read(void *context, unsigned int reg, unsigned int *val)
{
        struct ads1262 *priv = context;
        struct spi_transfer regg_read_xfer = {
                .tx_buf = priv->cmd_buffer,
                .rx_buf = priv->cmd_buffer,
                .len = 3,
                .speed_hz = ADS126x_CLK_RATE_HZ,
                .delay = {
                        .value = 2,
                        .unit = SPI_DELAY_UNIT_USECS,
                },
        };
        int ret;

        priv->cmd_buffer[0] = ADS126x_CMD_RREG | reg;
        priv->cmd_buffer[1] = 0;
        priv->cmd_buffer[2] = 0;

        ret = spi_sync_transfer(priv->spi, &reg_read_xfer, 1);
        if (ret)
                return ret;
        
        *val = priv->cmd_buffer[2];

        return 0;

}

static int ads1262_init(struct iio_dev *indio_dev)
{
        struct ads1262 *priv = iio_priv(indio_dev);
        struct device *dev = &priv->spi->dev;
        unsigned int val;
        int ret;

        ret = ads1262_write_cmd(priv, ADS126x_CMD_RESET);
        if(ret != 0)
                printf("There is something wrong with the deviec %x\n", ret);
        
        /* Setting up the MUX to read the internal temperature sensor*/
        ads1262_reg_write(priv, ADS126x_REG_INPMUX, ADS126x_DATA_TEMP_SENS, val);
        
        ret = ads1262_reg_read(priv, ADS126x_CMD_RREG, ADS126x_REG_INPMUX, val);
        if (!(priv->cmd_buffer[2] & ADS126x_DATA_TEMP_SENS))
                printf("Err writing to the INPMUX %x\n", priv->cmd_buffer[2]);
        
        /* Starting the ADC conversions*/
        ret = ads1262_write_cmd(priv, ADS126x_CMD_START1);

}

static const struct iio_info ads1262_info = {
        .read_raw = ads1262_read_raw,
        // .write_raw = &ads1262_write_raw,
};

static int ads1262_probe(struct spi_device *spi)
{
        return 0;
}

static const struct spi_device_id ads1262_id_table[] = {
        { "ad1262", 0 },
        {}
};
MODULE_DEVICE_TABLE(spi, ads1262_id_table);

staic const struct of_device_id ads1262_of_match[] = {
        { .compatible = "ti,ads1262"},
        {},
};
MODULE_DEVICE_TABLE(of, ads1262_of_match);

static struct spi_driver ads1262_driver = {
        .driver = {
                .name = "ads1262",
                .of_match_table = ads1262_of_match,
        },
        .proble = ads1262_probe,
        .id_table = ads1262_id_table,
};
module_spi_driver(ads1262_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayyad Abid <sayyad.abid16@gmail.com>");
MODULE_DESCRIPTION("TI ADS1262 ADC");