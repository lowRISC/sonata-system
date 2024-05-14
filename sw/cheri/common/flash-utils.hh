#pragma once
#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-spi.hh>

typedef CHERI::Capability<volatile SonataGPIO> &GpioRef;
typedef CHERI::Capability<volatile SonataSpi>  &SpiRef;

static const uint8_t CmdReadJEDECId         = 0x9f;
static const uint8_t CmdWriteEnable         = 0x06;
static const uint8_t CmdSectorErase         = 0x20;
static const uint8_t CmdReadStatusRegister1 = 0x05;
static const uint8_t CmdPageProgram         = 0x02;
static const uint8_t CmdReadData            = 0x03;

class SpiFlash
{
	private:
	SpiRef   spi;
	GpioRef  gpio;
	uint32_t csn_bit;

	void set_cs(bool enable)
	{
		gpio->output =
		  enable ? (gpio->output & ~csn_bit) : (gpio->output | csn_bit);
	}

	public:
	SpiFlash(SpiRef spi_, GpioRef gpio_, size_t csn_index)
	  : spi(spi_), gpio(gpio_), csn_bit(1 << csn_index)
	{
	}

	void read_jedec_id(uint8_t *jedec_id_out)
	{
		set_cs(true);
		spi->tx(&CmdReadJEDECId, 1);
		spi->rx(jedec_id_out, 3);
		set_cs(false);
	}

	void erase_sector(uint32_t address)
	{
		const uint8_t erase_cmd[4] = {CmdSectorErase,
		                              uint8_t((address >> 16) & 0xff),
		                              uint8_t((address >> 8) & 0xff),
		                              uint8_t(address & 0xff)};

		set_cs(true);
		spi->tx(&CmdWriteEnable, 1);
		set_cs(false);

		set_cs(true);
		spi->tx(erase_cmd, 4);
		set_cs(false);

		set_cs(true);
		spi->tx(&CmdReadStatusRegister1, 1);

		uint8_t status;
		do
		{
			spi->rx(&status, 1);
		} while ((status & 0x1) == 1);

		set_cs(false);
	}

	void write_page(uint32_t address, uint8_t *data)
	{
		const uint8_t write_cmd[4] = {CmdPageProgram,
		                              uint8_t((address >> 16) & 0xff),
		                              uint8_t((address >> 8) & 0xff),
		                              uint8_t(address & 0xff)};

		set_cs(true);
		spi->tx(&CmdWriteEnable, 1);
		set_cs(false);

		set_cs(true);
		spi->tx(write_cmd, 4);
		spi->tx(data, 256);
		set_cs(false);

		set_cs(true);
		spi->tx(&CmdReadStatusRegister1, 1);

		uint8_t status;
		do
		{
			spi->rx(&status, 1);
		} while ((status & 0x1) == 1);

		set_cs(false);
	}

	void read(uint32_t address, uint8_t *data_out, uint32_t len)
	{
		const uint8_t read_cmd[4] = {CmdReadData,
		                             uint8_t((address >> 16) & 0xff),
		                             uint8_t((address >> 8) & 0xff),
		                             uint8_t(address & 0xff)};
		set_cs(true);
		spi->tx(read_cmd, 4);
		spi->rx(data_out, len);
		set_cs(false);
	}
};
