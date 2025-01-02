# Pin multiplexer

This allows software to dynamically switch FPGA pins between input and output as well as reassign them for SPI, I2C, UART, etc.
The block also allows pad control.

To see the possible mappings, please refer to [the top configuration](https://github.com/lowRISC/sonata-system/blob/main/data/top_config.toml).
All selectors are byte addressable, this means that you can write four selectors at a time with a 32-bit write.

There are output pin selectors, which select which block output is connected to a particular FPGA pin.
The selector is one-hot, so you need to write `8'b100` if you want to select input 3 for example.
The default value for all of these selectors is `'b10`. As a consequence, you will need to use the pinmux before attempting use the additional headers as GPIO (e.g. the Raspberry Pi header's GPIO).

| Address | Pin output | Possible block outputs |
|---------|------------|------------------------|
| 0x000 | `ser0_tx` | 0, `uart_0_tx` |
| 0x001 | `ser1_tx` | 0, `uart_1_tx`, `uart_2_tx` |
| 0x002 | `rs232_tx` | 0, `uart_2_tx` |
| 0x003 | `rs485_tx` | 0, `uart_2_tx` |
| 0x004 | `scl0` | 0, `i2c_0_scl` |
| 0x005 | `sda0` | 0, `i2c_0_sda` |
| 0x006 | `scl1` | 0, `i2c_1_scl` |
| 0x007 | `sda1` | 0, `i2c_1_sda` |
| 0x008 | `rph_g0` | 0, `i2c_0_sda`, `gpio_0_ios_0` |
| 0x009 | `rph_g1` | 0, `i2c_0_scl`, `gpio_0_ios_1` |
| 0x00a | `rph_g2_sda` | 0, `i2c_1_sda`, `gpio_0_ios_2` |
| 0x00b | `rph_g3_scl` | 0, `i2c_1_scl`, `gpio_0_ios_3` |
| 0x00c | `rph_g4` | 0, `gpio_0_ios_4` |
| 0x00d | `rph_g5` | 0, `gpio_0_ios_5` |
| 0x00e | `rph_g6` | 0, `gpio_0_ios_6` |
| 0x00f | `rph_g7` | 0, `spi_1_cs_1`, `gpio_0_ios_7` |
| 0x010 | `rph_g8` | 0, `spi_1_cs_0`, `gpio_0_ios_8` |
| 0x011 | `rph_g9` | 0, `gpio_0_ios_9` |
| 0x012 | `rph_g10` | 0, `spi_1_copi`, `gpio_0_ios_10` |
| 0x013 | `rph_g11` | 0, `spi_1_sclk`, `gpio_0_ios_11` |
| 0x014 | `rph_g12` | 0, `gpio_0_ios_12`, `pwm_out_0` |
| 0x015 | `rph_g13` | 0, `gpio_0_ios_13`, `pwm_out_1` |
| 0x016 | `rph_txd0` | 0, `uart_1_tx`, `gpio_0_ios_14` |
| 0x017 | `rph_rxd0` | 0, `gpio_0_ios_15` |
| 0x018 | `rph_g16` | 0, `spi_2_cs_2`, `gpio_0_ios_16` |
| 0x019 | `rph_g17` | 0, `spi_2_cs_1`, `gpio_0_ios_17` |
| 0x01a | `rph_g18` | 0, `spi_2_cs_0`, `gpio_0_ios_18`, `pwm_out_2` |
| 0x01b | `rph_g19` | 0, `gpio_0_ios_19`, `pwm_out_3` |
| 0x01c | `rph_g20` | 0, `spi_2_copi`, `gpio_0_ios_20`, `pwm_out_4` |
| 0x01d | `rph_g21` | 0, `spi_2_sclk`, `gpio_0_ios_21`, `pwm_out_5` |
| 0x01e | `rph_g22` | 0, `gpio_0_ios_22` |
| 0x01f | `rph_g23` | 0, `gpio_0_ios_23` |
| 0x020 | `rph_g24` | 0, `gpio_0_ios_24` |
| 0x021 | `rph_g25` | 0, `gpio_0_ios_25` |
| 0x022 | `rph_g26` | 0, `gpio_0_ios_26` |
| 0x023 | `rph_g27` | 0, `gpio_0_ios_27` |
| 0x024 | `ah_tmpio0` | 0, `gpio_1_ios_0` |
| 0x025 | `ah_tmpio1` | 0, `uart_1_tx`, `gpio_1_ios_1` |
| 0x026 | `ah_tmpio2` | 0, `gpio_1_ios_2` |
| 0x027 | `ah_tmpio3` | 0, `gpio_1_ios_3`, `pwm_out_0` |
| 0x028 | `ah_tmpio4` | 0, `gpio_1_ios_4` |
| 0x029 | `ah_tmpio5` | 0, `gpio_1_ios_5`, `pwm_out_1` |
| 0x02a | `ah_tmpio6` | 0, `gpio_1_ios_6`, `pwm_out_2` |
| 0x02b | `ah_tmpio7` | 0, `gpio_1_ios_7` |
| 0x02c | `ah_tmpio8` | 0, `gpio_1_ios_8` |
| 0x02d | `ah_tmpio9` | 0, `gpio_1_ios_9`, `pwm_out_3` |
| 0x02e | `ah_tmpio10` | 0, `spi_1_cs_3`, `gpio_1_ios_10`, `pwm_out_4` |
| 0x02f | `ah_tmpio11` | 0, `spi_1_copi`, `gpio_1_ios_11`, `pwm_out_5` |
| 0x030 | `ah_tmpio12` | 0, `gpio_1_ios_12` |
| 0x031 | `ah_tmpio13` | 0, `spi_1_sclk`, `gpio_1_ios_13` |
| 0x032 | `mb1` | 0, `spi_2_cs_3` |
| 0x033 | `mb2` | 0, `spi_2_sclk` |
| 0x034 | `mb4` | 0, `spi_2_copi` |
| 0x035 | `mb5` | 0, `i2c_1_sda` |
| 0x036 | `mb6` | 0, `i2c_1_scl` |
| 0x037 | `mb7` | 0, `uart_1_tx` |
| 0x038 | `mb10` | 0, `pwm_out_0` |
| 0x039 | `pmod0_1` | 0, `gpio_2_ios_0`, `spi_1_cs_0` |
| 0x03a | `pmod0_2` | 0, `gpio_2_ios_1`, `spi_1_copi`, `pwm_out_1`, `uart_1_tx` |
| 0x03b | `pmod0_3` | 0, `gpio_2_ios_2`, `i2c_0_scl` |
| 0x03c | `pmod0_4` | 0, `gpio_2_ios_3`, `spi_1_sclk`, `i2c_0_sda` |
| 0x03d | `pmod0_7` | 0, `gpio_2_ios_4` |
| 0x03e | `pmod0_8` | 0, `gpio_2_ios_5`, `pwm_out_2` |
| 0x03f | `pmod0_9` | 0, `gpio_2_ios_6`, `spi_1_cs_1` |
| 0x040 | `pmod0_10` | 0, `gpio_2_ios_7`, `spi_1_cs_2` |
| 0x041 | `pmod1_1` | 0, `gpio_3_ios_0`, `spi_2_cs_0` |
| 0x042 | `pmod1_2` | 0, `gpio_3_ios_1`, `spi_2_copi`, `pwm_out_3`, `uart_2_tx` |
| 0x043 | `pmod1_3` | 0, `gpio_3_ios_2`, `i2c_1_scl` |
| 0x044 | `pmod1_4` | 0, `gpio_3_ios_3`, `spi_2_sclk`, `i2c_1_sda` |
| 0x045 | `pmod1_7` | 0, `gpio_3_ios_4` |
| 0x046 | `pmod1_8` | 0, `gpio_3_ios_5`, `pwm_out_4` |
| 0x047 | `pmod1_9` | 0, `gpio_3_ios_6`, `spi_2_cs_1` |
| 0x048 | `pmod1_10` | 0, `gpio_3_ios_7`, `spi_2_cs_2` |
| 0x049 | `pmodc_1` | 0, `gpio_4_ios_0` |
| 0x04a | `pmodc_2` | 0, `gpio_4_ios_1` |
| 0x04b | `pmodc_3` | 0, `gpio_4_ios_2` |
| 0x04c | `pmodc_4` | 0, `gpio_4_ios_3` |
| 0x04d | `pmodc_5` | 0, `gpio_4_ios_4` |
| 0x04e | `pmodc_6` | 0, `gpio_4_ios_5` |
| 0x04f | `appspi_d0` | 0, `spi_0_copi` |
| 0x050 | `appspi_clk` | 0, `spi_0_sclk` |
| 0x051 | `appspi_cs` | 0, `spi_0_cs_0` |
| 0x052 | `microsd_cmd` | 0, `spi_0_copi` |
| 0x053 | `microsd_clk` | 0, `spi_0_sclk` |
| 0x054 | `microsd_dat3` | 0, `spi_0_cs_1` |

Besides the output pin selectors, there are also selectors for which pin should drive block inputs:

| Address | Block input | Possible pin inputs |
|---------|-------------|---------------------|
| 0x800 | `gpio_0_ios_0` | 0, `rph_g0` |
| 0x801 | `gpio_0_ios_1` | 0, `rph_g1` |
| 0x802 | `gpio_0_ios_2` | 0, `rph_g2_sda` |
| 0x803 | `gpio_0_ios_3` | 0, `rph_g3_scl` |
| 0x804 | `gpio_0_ios_4` | 0, `rph_g4` |
| 0x805 | `gpio_0_ios_5` | 0, `rph_g5` |
| 0x806 | `gpio_0_ios_6` | 0, `rph_g6` |
| 0x807 | `gpio_0_ios_7` | 0, `rph_g7` |
| 0x808 | `gpio_0_ios_8` | 0, `rph_g8` |
| 0x809 | `gpio_0_ios_9` | 0, `rph_g9` |
| 0x80a | `gpio_0_ios_10` | 0, `rph_g10` |
| 0x80b | `gpio_0_ios_11` | 0, `rph_g11` |
| 0x80c | `gpio_0_ios_12` | 0, `rph_g12` |
| 0x80d | `gpio_0_ios_13` | 0, `rph_g13` |
| 0x80e | `gpio_0_ios_14` | 0, `rph_txd0` |
| 0x80f | `gpio_0_ios_15` | 0, `rph_rxd0` |
| 0x810 | `gpio_0_ios_16` | 0, `rph_g16` |
| 0x811 | `gpio_0_ios_17` | 0, `rph_g17` |
| 0x812 | `gpio_0_ios_18` | 0, `rph_g18` |
| 0x813 | `gpio_0_ios_19` | 0, `rph_g19` |
| 0x814 | `gpio_0_ios_20` | 0, `rph_g20` |
| 0x815 | `gpio_0_ios_21` | 0, `rph_g21` |
| 0x816 | `gpio_0_ios_22` | 0, `rph_g22` |
| 0x817 | `gpio_0_ios_23` | 0, `rph_g23` |
| 0x818 | `gpio_0_ios_24` | 0, `rph_g24` |
| 0x819 | `gpio_0_ios_25` | 0, `rph_g25` |
| 0x81a | `gpio_0_ios_26` | 0, `rph_g26` |
| 0x81b | `gpio_0_ios_27` | 0, `rph_g27` |
| 0x81c | `gpio_1_ios_0` | 0, `ah_tmpio0` |
| 0x81d | `gpio_1_ios_1` | 0, `ah_tmpio1` |
| 0x81e | `gpio_1_ios_2` | 0, `ah_tmpio2` |
| 0x81f | `gpio_1_ios_3` | 0, `ah_tmpio3` |
| 0x820 | `gpio_1_ios_4` | 0, `ah_tmpio4` |
| 0x821 | `gpio_1_ios_5` | 0, `ah_tmpio5` |
| 0x822 | `gpio_1_ios_6` | 0, `ah_tmpio6` |
| 0x823 | `gpio_1_ios_7` | 0, `ah_tmpio7` |
| 0x824 | `gpio_1_ios_8` | 0, `ah_tmpio8` |
| 0x825 | `gpio_1_ios_9` | 0, `ah_tmpio9` |
| 0x826 | `gpio_1_ios_10` | 0, `ah_tmpio10` |
| 0x827 | `gpio_1_ios_11` | 0, `ah_tmpio11` |
| 0x828 | `gpio_1_ios_12` | 0, `ah_tmpio12` |
| 0x829 | `gpio_1_ios_13` | 0, `ah_tmpio13` |
| 0x82a | `gpio_2_ios_0` | 0, `pmod0_1` |
| 0x82b | `gpio_2_ios_1` | 0, `pmod0_2` |
| 0x82c | `gpio_2_ios_2` | 0, `pmod0_3` |
| 0x82d | `gpio_2_ios_3` | 0, `pmod0_4` |
| 0x82e | `gpio_2_ios_4` | 0, `pmod0_7` |
| 0x82f | `gpio_2_ios_5` | 0, `pmod0_8` |
| 0x830 | `gpio_2_ios_6` | 0, `pmod0_9` |
| 0x831 | `gpio_2_ios_7` | 0, `pmod0_10` |
| 0x832 | `gpio_3_ios_0` | 0, `pmod1_1` |
| 0x833 | `gpio_3_ios_1` | 0, `pmod1_2` |
| 0x834 | `gpio_3_ios_2` | 0, `pmod1_3` |
| 0x835 | `gpio_3_ios_3` | 0, `pmod1_4` |
| 0x836 | `gpio_3_ios_4` | 0, `pmod1_7` |
| 0x837 | `gpio_3_ios_5` | 0, `pmod1_8` |
| 0x838 | `gpio_3_ios_6` | 0, `pmod1_9` |
| 0x839 | `gpio_3_ios_7` | 0, `pmod1_10` |
| 0x83a | `gpio_4_ios_0` | 0, `pmodc_1` |
| 0x83b | `gpio_4_ios_1` | 0, `pmodc_2` |
| 0x83c | `gpio_4_ios_2` | 0, `pmodc_3` |
| 0x83d | `gpio_4_ios_3` | 0, `pmodc_4` |
| 0x83e | `gpio_4_ios_4` | 0, `pmodc_5` |
| 0x83f | `gpio_4_ios_5` | 0, `pmodc_6` |
| 0x840 | `uart_0_rx` | 1, `ser0_rx` |
| 0x841 | `uart_1_rx` | 1, `ser1_rx`, `rph_rxd0`, `ah_tmpio0`, `mb8`, `pmod0_3` |
| 0x842 | `uart_2_rx` | 1, `ser1_rx`, `rs232_rx`, `rs485_rx`, `pmod1_3` |
| 0x843 | `spi_0_cipo` | 0, `appspi_d1`, `microsd_dat0` |
| 0x844 | `spi_1_cipo` | 0, `rph_g9`, `ah_tmpio12`, `pmod0_3` |
| 0x845 | `spi_2_cipo` | 0, `rph_g19`, `mb3`, `pmod1_3` |

## Regeneration

If any changes are made to the top configuration, the templates or the bus, you must regenerate the top.
You can do so using the top generation utility, which regenerates the pinmux, the bus and the sonata package which is used by the SystemVerilog generate statements throughout the project.

```sh
./util/top_gen.py
```
