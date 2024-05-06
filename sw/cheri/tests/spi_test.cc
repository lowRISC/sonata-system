#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <cheri.hh>
#include <stdint.h>

using namespace CHERI;

#define CPU_FREQ_HZ (25'000'000)
#define BAUD_RATE   (   115'200)

/**
 * OpenTitan UART
 */
class OpenTitanUart
{
  typedef uint32_t RegisterType;

 [[maybe_unused]] RegisterType intrState;
 [[maybe_unused]] RegisterType intrEnable;
 [[maybe_unused]] RegisterType intrTest;
 [[maybe_unused]] RegisterType alertTest;
 [[maybe_unused]] RegisterType ctrl;
 [[maybe_unused]] RegisterType status;
 [[maybe_unused]] RegisterType rData;
 RegisterType wData;
 [[maybe_unused]] RegisterType fifoCtrl;
 RegisterType fifoStatus;
 [[maybe_unused]] RegisterType ovrd;
 [[maybe_unused]] RegisterType val;
 [[maybe_unused]] RegisterType timeoutCtrl;

  public:
  void init() volatile
  {
    // NCO = 2^20 * baud rate / cpu frequency
    const uint32_t nco = (((uint64_t) BAUD_RATE << 20) / CPU_FREQ_HZ);
    // Set the baud rate and enable transmit & receive
    ctrl = (nco << 16) | 0b11;
  };

  bool can_write() volatile
  {
    return (fifoStatus & 0xff) < 32;
  };

  /**
   * Write one byte, blocking until the byte is written.
   */
  void blocking_write(uint8_t byte) volatile
  {
    while (!can_write()) {}
    wData = byte;
  }

  void write_str(const char* str) volatile
  {
    while(*str) {
      blocking_write(*str);
      ++str;
    }
  }

  void write_hex(uint32_t n) volatile
  {
    char str_buf[9];

    for (int i = 0;i < 8;++i) {
      if ((n & 0xf) < 10) {
        str_buf[7 - i] = (n & 0xf) + '0';
      } else {
        str_buf[7 - i] = (n & 0xf) + 'a' - 10;
      }

      n >>= 4;
    }
    str_buf[8] = 0;

    write_str(str_buf);
  }

  void write_hex8b(uint8_t n) volatile
  {
    char str_buf[3];

    for (int i = 0;i < 2;++i) {
      if ((n & 0xf) < 10) {
        str_buf[1 - i] = (n & 0xf) + '0';
      } else {
        str_buf[1 - i] = (n & 0xf) + 'a' - 10;
      }

      n >>= 4;
    }
    str_buf[2] = 0;

    write_str(str_buf);
  }
};

class Spi
{
  private:
    typedef uint32_t RegisterType;

    [[maybe_unused]] RegisterType intrState;
    [[maybe_unused]] RegisterType intrEnable;
    [[maybe_unused]] RegisterType intrTest;
    [[maybe_unused]] RegisterType cfg;
    [[maybe_unused]] RegisterType control;
    [[maybe_unused]] RegisterType status;
    [[maybe_unused]] RegisterType start;
    [[maybe_unused]] RegisterType rxFifo;
    [[maybe_unused]] RegisterType txFifo;
  public:
    void init(bool cpol, bool cpha, bool msb_first, uint16_t half_clk_period) volatile
    {
      cfg = (cpol      ? 1 << 31 : 0) |
            (cpha      ? 1 << 30 : 0) |
            (msb_first ? 1 << 29 : 0) |
            half_clk_period;
    }

    void wait_idle() volatile
    {
      // Wait whilst IDLE field in STATUs is low
      while((status & 0x40000) == 0);
    }

    void tx(const uint8_t* data, uint32_t len) volatile {
      wait_idle();
      // Set TX_ENABLE
      control = 0x4;
      start = len;

      uint32_t tx_avail = 0;
      for (int i = 0;i < len; ++i) {
        if (tx_avail == 0) {
          while (tx_avail < 64) {
            // Read number of bytes in TX FIFO to calculate space available for more bytes
            tx_avail = 64 - (status & 0xff);
          }
        }

        txFifo = data[i];
        tx_avail--;
      }
    }

    void rx(uint8_t* data, uint32_t len) volatile {
      wait_idle();
      // Set RX_ENABLE
      control = 0x8;
      start = len;

      for (int i = 0;i < len; ++i) {
        // Wait for at least one byte to be available in the RX FIFO
        while (((status >> 8) & 0xff) == 0);
        data[i] = static_cast<uint8_t>(rxFifo);
      }
    }
};

class SpiFlash
{
  private:
    Capability<volatile Spi>& spi;
    Capability<volatile uint32_t> gpo;
    uint32_t csn_bit;

    const uint8_t CmdReadJEDECId = 0x9f;
    const uint8_t CmdWriteEnable = 0x06;
    const uint8_t CmdSectorErase = 0x20;
    const uint8_t CmdReadStatusRegister1 = 0x05;
    const uint8_t CmdPageProgram = 0x02;
    const uint8_t CmdReadData = 0x03;

    void set_cs(bool enable) {
      if (enable) {
        *gpo = *gpo & ~csn_bit;
      } else {
        *gpo = *gpo | csn_bit;
      }
    }
  public:
    SpiFlash(Capability<volatile Spi>& spi_,
        Capability<volatile uint32_t>& gpo_, uint32_t csn_index) : spi(spi_),
          gpo(gpo_), csn_bit(1 << csn_index) {}

    void read_jedec_id(uint8_t* jedec_id_out) {
      set_cs(true);
      spi->tx(&CmdReadJEDECId, 1);
      spi->rx(jedec_id_out, 3);
      set_cs(false);
    }

    void erase_sector(uint32_t address)
    {
      uint8_t erase_cmd[4] = {CmdSectorErase,
        uint8_t((address >> 16) & 0xff), uint8_t((address >> 8) & 0xff),
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
      do {
        spi->rx(&status, 1);
      } while ((status & 0x1) == 1);

      set_cs(false);
    }

    void write_page(uint32_t address, uint8_t* data)
    {
      uint8_t write_cmd[4] = {CmdPageProgram,
        uint8_t((address >> 16) & 0xff), uint8_t((address >> 8) & 0xff),
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
      do {
        spi->rx(&status, 1);
      } while ((status & 0x1) == 1);

      set_cs(false);
    }


    void read(uint32_t address, uint8_t* data_out, uint32_t len)
    {
      uint8_t read_cmd[4] = {CmdReadData,
        uint8_t((address >> 16) & 0xff), uint8_t((address >> 8) & 0xff),
        uint8_t(address & 0xff)};

      set_cs(true);
      spi->tx(read_cmd, 4);
      spi->rx(data_out, len);
      set_cs(false);
    }
};


/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]]
extern "C" void rom_loader_entry(void *rwRoot)
{
  Capability<void> root{rwRoot};

  uint8_t write_data[256];
  uint8_t read_data[256];

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address() = 0x80100000;
  uart.bounds()  = 0x1000;

  Capability<volatile Spi> spi = root.cast<volatile Spi>();
  spi.address() = 0x80300000;
  spi.bounds() = 0x1000;

  Capability<volatile uint32_t> gpo = root.cast<volatile uint32_t>();
  gpo.address() = 0x80000000;
  gpo.bounds() = sizeof(uint32_t);

  SpiFlash spi_flash(spi, gpo, 12);

  spi->init(false, false, true, 0);
  uart->init();
  uart->write_str("Hello World!\r\n");

  uint8_t jedec_id[3];

  spi_flash.read_jedec_id(jedec_id);

  uart->write_str("JEDEC ID: ");
  uart->write_hex8b(jedec_id[0]);
  uart->write_str(" ");
  uart->write_hex8b(jedec_id[1]);
  uart->write_str(" ");
  uart->write_hex8b(jedec_id[2]);
  uart->write_str("\r\n");

  for (int i = 0; i < 256; ++i) {
    write_data[i] = i;
  }

  spi_flash.erase_sector(0);
  spi_flash.write_page(0, write_data);
  spi_flash.read(0, read_data, 256);

  uart->write_str("Got first flash read:\r\n");
  for (int i = 0; i < 256; ++i) {
    uart->write_hex8b(read_data[i]);
    uart->write_str("\r\n");
  }

  spi_flash.read(128, read_data, 256);

  uart->write_str("Got second flash read:\r\n");
  for (int i = 0; i < 256; ++i) {
    uart->write_hex8b(read_data[i]);
    uart->write_str("\r\n");
  }

  while (true) {
    asm("");
  }
}
