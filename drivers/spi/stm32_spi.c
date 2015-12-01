/*
 * (C) Copyright 2015
 * Kamil Lulko, <kamil.lulko@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <malloc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/stm32.h>
#include <spi.h>

#define RCC_SPI1_ENABLE	(1 << 12)
#define RCC_SPI2_ENABLE	(1 << 14)
#define RCC_SPI3_ENABLE	(1 << 15)
#define RCC_SPI4_ENABLE	(1 << 13)
#define RCC_SPI5_ENABLE	(1 << 20)
#define RCC_SPI6_ENABLE	(1 << 21)

#define STM32_MAX_SPI 6

static const uint32_t spi_base_rcc_pair[STM32_MAX_SPI][2] = {
	{ STM32_SPI1_BASE, RCC_SPI1_ENABLE },
	{ STM32_SPI2_BASE, RCC_SPI2_ENABLE },
	{ STM32_SPI3_BASE, RCC_SPI3_ENABLE },
	{ STM32_SPI4_BASE, RCC_SPI4_ENABLE },
	{ STM32_SPI5_BASE, RCC_SPI5_ENABLE },
	{ STM32_SPI6_BASE, RCC_SPI6_ENABLE }
};

/* To have board specific CS pin list without having to touch this driver
 * the CS arrays are declared as weak symbols. Define your own arrays
 * in your board.c
 */
struct stm32_gpio_dsc spi1_cs_gpio[] __attribute__((weak)) = { {-1, -1} };
struct stm32_gpio_dsc spi2_cs_gpio[] __attribute__((weak)) = { {-1, -1} };
struct stm32_gpio_dsc spi3_cs_gpio[] __attribute__((weak)) = { {-1, -1} };
struct stm32_gpio_dsc spi4_cs_gpio[] __attribute__((weak)) = { {-1, -1} };
struct stm32_gpio_dsc spi5_cs_gpio[] __attribute__((weak)) = { {-1, -1} };
struct stm32_gpio_dsc spi6_cs_gpio[] __attribute__((weak)) = { {-1, -1} };

static const struct stm32_gpio_dsc *spi_cs_array[] = {
		spi1_cs_gpio, spi2_cs_gpio, spi3_cs_gpio,
		spi4_cs_gpio, spi5_cs_gpio, spi6_cs_gpio
};

struct stm32_spi {
	uint32_t cr1;
	uint32_t cr2;
	uint32_t sr;
	uint32_t dr;
	uint32_t crcpr;
	uint32_t txcrcr;
	uint32_t i2scfgr;
	uint32_t i2spr;
};

struct stm32_spi_platdata {
	int frequency;		/* Default clock frequency, -1 for none */
	ulong base;
};

struct stm32_spi_priv {
	struct stm32_spi *regs;
	unsigned int freq;
	unsigned int mode;
	int apb_bus;
};

struct stm32_spi_slave {
	struct spi_slave slave;
	u8 cs_pol;
};

#define SPI_CR1_CPHA			(1 << 0)
#define SPI_CR1_CPOL			(1 << 1)
#define SPI_CR1_MSTR			(1 << 2)
#define SPI_CR1_BR0			(1 << 3)
#define SPI_CR1_BR1			(1 << 4)
#define SPI_CR1_BR2			(1 << 5)
#define SPI_CR1_BR_SHIFT		3
#define SPI_CR1_BR_MASK		(7 << SPI_CR1_BR_SHIFT)
#define SPI_CR1_SPE			(1 << 6)
#define SPI_CR1_LSBFIRST		(1 << 7)
#define SPI_CR1_SSI			(1 << 8)
#define SPI_CR1_SSM			(1 << 9)
#define SPI_CR1_RXONLY		(1 << 10)
#define SPI_CR1_DFF			(1 << 11)
#define SPI_CR1_CRCNEXT		(1 << 12)
#define SPI_CR1_CRCEN			(1 << 13)
#define SPI_CR1_BIDIOE		(1 << 14)
#define SPI_CR1_BIDIMODE		(1 << 15)

#define SPI_SR_RXNE			(1 << 0)
#define SPI_SR_TXE			(1 << 1)

#define SPI_I2SCFGR_I2SMOD	(1 << 11)

DECLARE_GLOBAL_DATA_PTR;

int stm32_spi_claim_bus(struct udevice *dev)
{
	struct stm32_spi_priv *priv = dev_get_priv(dev);

	setbits_le32(priv->regs->cr1, SPI_CR1_SPE);

	//spi_cs_deactivate(slave);

	return 0;
}

int stm32_spi_release_bus(struct udevice *dev)
{
	/* TODO: Shut the controller down */
	return 0;
}

int  stm32_spi_xfer(struct udevice *dev, unsigned int bitlen, const void *dout,
		    void *din, unsigned long flags)
{
	struct stm32_spi_priv *priv = dev_get_priv(dev);
	unsigned int	len;
	const u8	*txp = dout;
	u8 *rxp = din;
	u8 value;

	if (bitlen == 0)
		return 0;

	for (len = bitlen / 8; len > 0; len--) {
		if (txp)
			value = *txp++;
		else
			value = 0xFF;
		while ((readl(&priv->regs->sr) & SPI_SR_TXE) == 0)
			;
		writel(value, &priv->regs->dr);
		while ((readl(&priv->regs->sr) & SPI_SR_RXNE) == 0)
			;
		value = readl(&priv->regs->dr);
		if (rxp)
			*rxp++ = value;
	}

	return 0;
}

void spi_cs_activate(struct spi_slave *slave)
{
	//struct stm32_spi_slave *stm32_slave = to_stm32_spi_slave(slave);

//	if (stm32_slave->cs_pol == 0)
//		stm32_gpout_set(&spi_cs_array[slave->bus][slave->cs], 0);
//	else
//		stm32_gpout_set(&spi_cs_array[slave->bus][slave->cs], 1);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	//struct stm32_spi_slave *stm32_slave = to_stm32_spi_slave(slave);

//	if (stm32_slave->cs_pol == 0)
//		stm32_gpout_set(&spi_cs_array[slave->bus][slave->cs], 1);
//	else
//		stm32_gpout_set(&spi_cs_array[slave->bus][slave->cs], 0);
}

int stm32_spi_set_speed(struct udevice *bus, uint hz)
{
	struct stm32_spi_priv *priv = dev_get_priv(bus);
	int apb_clk, i;

	if (priv->apb_bus == 1)
		apb_clk = clock_get(CLOCK_APB1);
	else
		apb_clk = clock_get(CLOCK_APB2);


	clrbits_le32(&priv->regs->cr1, SPI_CR1_BR_MASK);

	for (i = 0; i < 8; i++) {
		int spi_clk = apb_clk / (1 << (i + 1));
		if (spi_clk <= hz) {
			setbits_le32(&priv->regs->cr1, (i << SPI_CR1_BR_SHIFT));
			break;
		}
	}

	return 0;
}

int stm32_spi_set_mode(struct udevice *bus, uint mode)
{
	struct stm32_spi_priv *priv = dev_get_priv(bus);

	if (mode & SPI_CPHA)
		setbits_le32(&priv->regs->cr1, SPI_CR1_CPHA);
	else
		clrbits_le32(&priv->regs->cr1, SPI_CR1_CPHA);

	if (mode & SPI_CPOL)
		setbits_le32(&priv->regs->cr1, SPI_CR1_CPOL);
	else
		clrbits_le32(&priv->regs->cr1, SPI_CR1_CPOL);

	if (mode & SPI_LSB_FIRST)
		setbits_le32(&priv->regs->cr1, SPI_CR1_LSBFIRST);
	else
		clrbits_le32(&priv->regs->cr1, SPI_CR1_LSBFIRST);
	return 0;
}

int stm32_spi_cs_info(struct udevice *bus, uint cs, struct spi_cs_info *info)
{
	return 0;
}

static int stm32_spi_probe(struct udevice *bus)
{
	struct stm32_spi_platdata *plat = dev_get_platdata(bus);
	struct stm32_spi_priv *priv = dev_get_priv(bus);
	struct stm32_spi *regs;
	s32 spi_number = -1;
	s32 i;

	priv->regs = (struct stm32_spi *)plat->base;
	regs = priv->regs;

	priv->freq = plat->frequency;

	if ((plat->base & STM32_BUS_MASK) == STM32_APB1PERIPH_BASE)
		priv->apb_bus = 1;
	else if ((plat->base & STM32_BUS_MASK) == STM32_APB2PERIPH_BASE)
		priv->apb_bus = 2;
	else
		return -EINVAL;

	for (i = 0; i < STM32_MAX_SPI; i++) {
		if (plat->base == spi_base_rcc_pair[i][0])
			spi_number = i;
	}

	if (spi_number == -1)
		return -EINVAL;

	if (priv->apb_bus == 1)
		setbits_le32(&STM32_RCC->apb1enr, spi_base_rcc_pair[spi_number][1]);
	else
		setbits_le32(&STM32_RCC->apb2enr, spi_base_rcc_pair[spi_number][1]);

	writel(SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM ,&regs->cr1);
	clrbits_le32(&regs->i2scfgr, SPI_I2SCFGR_I2SMOD);

	return 0;
}

static const struct dm_spi_ops stm32_spi_ops = {
	.claim_bus	= stm32_spi_claim_bus,
	.release_bus	= stm32_spi_release_bus,
	.xfer		= stm32_spi_xfer,
	.set_speed	= stm32_spi_set_speed,
	.set_mode	= stm32_spi_set_mode,
	.cs_info	= stm32_spi_cs_info,
};

U_BOOT_DRIVER(stm32_spi) = {
	.name	= "stm32_spi",
	.id	= UCLASS_SPI,
	.ops	= &stm32_spi_ops,
	.probe	= stm32_spi_probe,
};
