#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spi/spi>
#include <linux/cleanup.h>
#include <linux/bitfield.h>
#include <math.h>

#include <linux/iio.h>
#include <linux/iio/buffer.h>

#include <asm/unaligned.h>

/* Commands */
#define ADS126x_CMD_NOP 0x00
#define ADS126x_CMD_RESET 0x06
#define ADS126x_CMD_START1 0x08
#define ADS126x_CMD_STOP1 0x0A
#define ADS126x_CMD_START2 0x0C
#define ADS126x_CMD_STOP2 0x0E
#define ADS126x_CMD_RDATA1 0x12
#define ADS126x_CMD_RDATA2 0x14
#define ADS126x_CMD_SYOCAL1 0x16
#define ADS126x_CMD_SYGCAL1 0x17
#define ADS126x_CMD_SFOCAL1 0x19
#define ADS126x_CMD_SYOCAL2 0x1B
#define ADS126x_CMD_SYGCAL2 0x1C
#define ADS126x_CMD_SFOCAL2 0x1E
#define ADS126x_CMD_RREG 0x20
#define ADS126x_CMD_WREG 0x40

/* Registers */
#define ADS126x_REG_ID 0x00
#define ADS126x_REG_POWER 0x01
#define ADS126x_REG_INTERFACE 0x02
#define ADS126x_REG_MODE0 0x03
#define ADS126x_REG_MODE1 0x04
#define ADS126x_REG_MODE2 0x05
#define ADS126x_REG_INPMUX 0x06
#define ADS126x_REG_OFCAL0 0x07
#define ADS126x_REG_OFCAL1 0x08
#define ADS126x_REG_OFCAL2 0x09
#define ADS126x_REG_FSCAL0 0x0A
#define ADS126x_REG_FSCAL1 0x0B
#define ADS126x_REG_FSCAL2 0x0C
#define ADS126x_REG_IDACMUX 0x0D
#define ADS126x_REG_IDACMAG 0x0E
#define ADS126x_REG_REFMUX 0x0F
#define ADS126x_REG_TDACP 0x10
#define ADS126x_REG_TDACN 0x11
#define ADS126x_REG_GPIOCON 0x12
#define ADS126x_REG_GPIODIR 0x13
#define ADS126x_REG_GPIODAT 0x14
#define ADS126x_REG_ADC2CFG 0x16
#define ADS126x_REG_ADC2OFC0 0x17
#define ADS126x_REG_ADC2OFC1 0x18
#define ADS126x_REG_ADC2FSC0 0x19
#define ADS126x_REG_ADC2FSC1 0x1A


