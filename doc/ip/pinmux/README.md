# Pin multiplexer

This allows software to dynamically switch FPGA pins between input and output as well as reassign them for SPI, I2C, UART, etc.
The block also allows pad control.

To see the possible mappings, please refer to [the top configuration](https://github.com/lowRISC/sonata-system/blob/main/data/top_config.toml).
All selectors are byte addressable, this means that you can write four selectors at a time with a 32-bit write.

There are output pin selectors, which select which block output is connected to a particular FPGA pin.
The selector is one-hot, so you need to write `8'b100` if you want to select input 3 for example.
The default value for all of these selectors is `'b10`.

| Address | Pin output | Possible block outputs |
|---------|------------|------------------------|
| 0x000 | `ser0_tx` | 0, `uart[0].tx` |
| 0x001 | `ser1_tx` | 0, `uart[1].tx`, `uart[2].tx` |
| 0x002 | `rs232_tx` | 0, `uart[2].tx` |
| 0x003 | `rs485_tx` | 0, `uart[2].tx` |
| 0x004 | `scl0` | 0, `i2c[0].scl` |
| 0x005 | `sda0` | 0, `i2c[0].sda` |
| 0x006 | `scl1` | 0, `i2c[1].scl` |
| 0x007 | `sda1` | 0, `i2c[1].sda` |
| 0x008 | `rph_g0` | 0, `i2c[0].sda`, `gpio[0].ios[0]` |
| 0x009 | `rph_g1` | 0, `i2c[0].scl`, `gpio[0].ios[1]` |
| 0x00a | `rph_g2_sda` | 0, `i2c[1].sda`, `gpio[0].ios[2]` |
| 0x00b | `rph_g3_scl` | 0, `i2c[1].scl`, `gpio[0].ios[3]` |
| 0x00c | `rph_g4` | 0, `gpio[0].ios[4]` |
| 0x00d | `rph_g5` | 0, `gpio[0].ios[5]` |
| 0x00e | `rph_g6` | 0, `gpio[0].ios[6]` |
| 0x00f | `rph_g7` | 0, `spi[1].cs[1]`, `gpio[0].ios[7]` |
| 0x010 | `rph_g8` | 0, `spi[1].cs[0]`, `gpio[0].ios[8]` |
| 0x011 | `rph_g9` | 0, `gpio[0].ios[9]` |
| 0x012 | `rph_g10` | 0, `spi[1].copi`, `gpio[0].ios[10]` |
| 0x013 | `rph_g11` | 0, `spi[1].sclk`, `gpio[0].ios[11]` |
| 0x014 | `rph_g12` | 0, `gpio[0].ios[12]`, `pwm_out[0]` |
| 0x015 | `rph_g13` | 0, `gpio[0].ios[13]`, `pwm_out[1]` |
| 0x016 | `rph_txd0` | 0, `uart[1].tx`, `gpio[0].ios[14]` |
| 0x017 | `rph_rxd0` | 0, `gpio[0].ios[15]` |
| 0x018 | `rph_g16` | 0, `spi[2].cs[2]`, `gpio[0].ios[16]` |
| 0x019 | `rph_g17` | 0, `spi[2].cs[1]`, `gpio[0].ios[17]` |
| 0x01a | `rph_g18` | 0, `spi[2].cs[0]`, `gpio[0].ios[18]`, `pwm_out[2]` |
| 0x01b | `rph_g19` | 0, `gpio[0].ios[19]`, `pwm_out[3]` |
| 0x01c | `rph_g20` | 0, `spi[2].copi`, `gpio[0].ios[20]`, `pwm_out[4]` |
| 0x01d | `rph_g21` | 0, `spi[2].sclk`, `gpio[0].ios[21]`, `pwm_out[5]` |
| 0x01e | `rph_g22` | 0, `gpio[0].ios[22]` |
| 0x01f | `rph_g23` | 0, `gpio[0].ios[23]` |
| 0x020 | `rph_g24` | 0, `gpio[0].ios[24]` |
| 0x021 | `rph_g25` | 0, `gpio[0].ios[25]` |
| 0x022 | `rph_g26` | 0, `gpio[0].ios[26]` |
| 0x023 | `rph_g27` | 0, `gpio[0].ios[27]` |
| 0x024 | `ah_tmpio0` | 0, `gpio[1].ios[0]` |
| 0x025 | `ah_tmpio1` | 0, `uart[1].tx`, `gpio[1].ios[1]` |
| 0x026 | `ah_tmpio2` | 0, `gpio[1].ios[2]` |
| 0x027 | `ah_tmpio3` | 0, `gpio[1].ios[3]`, `pwm_out[0]` |
| 0x028 | `ah_tmpio4` | 0, `gpio[1].ios[4]` |
| 0x029 | `ah_tmpio5` | 0, `gpio[1].ios[5]`, `pwm_out[1]` |
| 0x02a | `ah_tmpio6` | 0, `gpio[1].ios[6]`, `pwm_out[2]` |
| 0x02b | `ah_tmpio7` | 0, `gpio[1].ios[7]` |
| 0x02c | `ah_tmpio8` | 0, `gpio[1].ios[8]` |
| 0x02d | `ah_tmpio9` | 0, `gpio[1].ios[9]`, `pwm_out[2]` |
| 0x02e | `ah_tmpio10` | 0, `spi[1].cs[3]`, `gpio[1].ios[10]`, `pwm_out[4]` |
| 0x02f | `ah_tmpio11` | 0, `spi[1].copi`, `gpio[1].ios[11]`, `pwm_out[5]` |
| 0x030 | `ah_tmpio12` | 0, `gpio[1].ios[12]` |
| 0x031 | `ah_tmpio13` | 0, `spi[1].sclk`, `gpio[1].ios[13]` |
| 0x032 | `mb1` | 0, `spi[2].cs[3]` |
| 0x033 | `mb2` | 0, `spi[2].sclk` |
| 0x034 | `mb4` | 0, `spi[2].copi` |
| 0x035 | `mb5` | 0, `i2c[1].sda` |
| 0x036 | `mb6` | 0, `i2c[1].scl` |
| 0x037 | `mb7` | 0, `uart[1].tx` |
| 0x038 | `mb10` | 0, `pwm_out[0]` |
| 0x039 | `pmod0_1` | 0, `gpio[2].ios[0]`, `spi[1].cs[0]` |
| 0x03a | `pmod0_2` | 0, `gpio[2].ios[1]`, `spi[1].copi`, `pwm_out[1]`, `uart[1].tx` |
| 0x03b | `pmod0_3` | 0, `gpio[2].ios[2]`, `i2c[0].scl` |
| 0x03c | `pmod0_4` | 0, `gpio[2].ios[3]`, `spi[1].sclk`, `i2c[0].sda` |
| 0x03d | `pmod0_7` | 0, `gpio[2].ios[4]` |
| 0x03e | `pmod0_8` | 0, `gpio[2].ios[5]`, `pwm_out[2]` |
| 0x03f | `pmod0_9` | 0, `gpio[2].ios[6]`, `spi[1].cs[1]` |
| 0x040 | `pmod0_10` | 0, `gpio[2].ios[7]`, `spi[1].cs[2]` |
| 0x041 | `pmod1_1` | 0, `gpio[3].ios[0]`, `spi[2].cs[0]` |
| 0x042 | `pmod1_2` | 0, `gpio[3].ios[1]`, `spi[2].copi`, `pwm_out[3]`, `uart[2].tx` |
| 0x043 | `pmod1_3` | 0, `gpio[3].ios[2]`, `i2c[1].scl` |
| 0x044 | `pmod1_4` | 0, `gpio[3].ios[3]`, `spi[2].sclk`, `i2c[1].sda` |
| 0x045 | `pmod1_7` | 0, `gpio[3].ios[4]` |
| 0x046 | `pmod1_8` | 0, `gpio[3].ios[5]`, `pwm_out[4]` |
| 0x047 | `pmod1_9` | 0, `gpio[3].ios[6]`, `spi[2].cs[1]` |
| 0x048 | `pmod1_10` | 0, `gpio[3].ios[7]`, `spi[2].cs[2]` |
| 0x049 | `pmodc_1` | 0, `gpio[4].ios[0]` |
| 0x04a | `pmodc_2` | 0, `gpio[4].ios[1]` |
| 0x04b | `pmodc_3` | 0, `gpio[4].ios[2]` |
| 0x04c | `pmodc_4` | 0, `gpio[4].ios[3]` |
| 0x04d | `pmodc_5` | 0, `gpio[4].ios[4]` |
| 0x04e | `pmodc_6` | 0, `gpio[4].ios[5]` |
| 0x04f | `appspi_d0` | 0, `spi[0].copi` |
| 0x050 | `appspi_clk` | 0, `spi[0].sclk` |
| 0x051 | `appspi_cs` | 0, `spi[0].cs[0]` |
| 0x052 | `microsd_cmd` | 0, `spi[0].copi` |
| 0x053 | `microsd_clk` | 0, `spi[0].sclk` |
| 0x054 | `microsd_dat3` | 0, `spi[0].cs[1]` |

Besides the output pin selectors, there are also selectors for which pin should drive block inputs:

| Address | Block input | Possible pin inputs |
|---------|-------------|---------------------|
| 0x800 | `gpio[0].ios[0]` | 0, `rph_g0` |
| 0x801 | `gpio[0].ios[1]` | 0, `rph_g1` |
| 0x802 | `gpio[0].ios[2]` | 0, `rph_g2_sda` |
| 0x803 | `gpio[0].ios[3]` | 0, `rph_g3_scl` |
| 0x804 | `gpio[0].ios[4]` | 0, `rph_g4` |
| 0x805 | `gpio[0].ios[5]` | 0, `rph_g5` |
| 0x806 | `gpio[0].ios[6]` | 0, `rph_g6` |
| 0x807 | `gpio[0].ios[7]` | 0, `rph_g7` |
| 0x808 | `gpio[0].ios[8]` | 0, `rph_g8` |
| 0x809 | `gpio[0].ios[9]` | 0, `rph_g9` |
| 0x80a | `gpio[0].ios[10]` | 0, `rph_g10` |
| 0x80b | `gpio[0].ios[11]` | 0, `rph_g11` |
| 0x80c | `gpio[0].ios[12]` | 0, `rph_g12` |
| 0x80d | `gpio[0].ios[13]` | 0, `rph_g13` |
| 0x80e | `gpio[0].ios[14]` | 0, `rph_txd0` |
| 0x80f | `gpio[0].ios[15]` | 0, `rph_rxd0` |
| 0x810 | `gpio[0].ios[16]` | 0, `rph_g16` |
| 0x811 | `gpio[0].ios[17]` | 0, `rph_g17` |
| 0x812 | `gpio[0].ios[18]` | 0, `rph_g18` |
| 0x813 | `gpio[0].ios[19]` | 0, `rph_g19` |
| 0x814 | `gpio[0].ios[20]` | 0, `rph_g20` |
| 0x815 | `gpio[0].ios[21]` | 0, `rph_g21` |
| 0x816 | `gpio[0].ios[22]` | 0, `rph_g22` |
| 0x817 | `gpio[0].ios[23]` | 0, `rph_g23` |
| 0x818 | `gpio[0].ios[24]` | 0, `rph_g24` |
| 0x819 | `gpio[0].ios[25]` | 0, `rph_g25` |
| 0x81a | `gpio[0].ios[26]` | 0, `rph_g26` |
| 0x81b | `gpio[0].ios[27]` | 0, `rph_g27` |
| 0x81c | `gpio[1].ios[0]` | 0, `ah_tmpio0` |
| 0x81d | `gpio[1].ios[1]` | 0, `ah_tmpio1` |
| 0x81e | `gpio[1].ios[2]` | 0, `ah_tmpio2` |
| 0x81f | `gpio[1].ios[3]` | 0, `ah_tmpio3` |
| 0x820 | `gpio[1].ios[4]` | 0, `ah_tmpio4` |
| 0x821 | `gpio[1].ios[5]` | 0, `ah_tmpio5` |
| 0x822 | `gpio[1].ios[6]` | 0, `ah_tmpio6` |
| 0x823 | `gpio[1].ios[7]` | 0, `ah_tmpio7` |
| 0x824 | `gpio[1].ios[8]` | 0, `ah_tmpio8` |
| 0x825 | `gpio[1].ios[9]` | 0, `ah_tmpio9` |
| 0x826 | `gpio[1].ios[10]` | 0, `ah_tmpio10` |
| 0x827 | `gpio[1].ios[11]` | 0, `ah_tmpio11` |
| 0x828 | `gpio[1].ios[12]` | 0, `ah_tmpio12` |
| 0x829 | `gpio[1].ios[13]` | 0, `ah_tmpio13` |
| 0x82a | `gpio[2].ios[0]` | 0, `pmod0_1` |
| 0x82b | `gpio[2].ios[1]` | 0, `pmod0_2` |
| 0x82c | `gpio[2].ios[2]` | 0, `pmod0_3` |
| 0x82d | `gpio[2].ios[3]` | 0, `pmod0_4` |
| 0x82e | `gpio[2].ios[4]` | 0, `pmod0_7` |
| 0x82f | `gpio[2].ios[5]` | 0, `pmod0_8` |
| 0x830 | `gpio[2].ios[6]` | 0, `pmod0_9` |
| 0x831 | `gpio[2].ios[7]` | 0, `pmod0_10` |
| 0x832 | `gpio[3].ios[0]` | 0, `pmod1_1` |
| 0x833 | `gpio[3].ios[1]` | 0, `pmod1_2` |
| 0x834 | `gpio[3].ios[2]` | 0, `pmod1_3` |
| 0x835 | `gpio[3].ios[3]` | 0, `pmod1_4` |
| 0x836 | `gpio[3].ios[4]` | 0, `pmod1_7` |
| 0x837 | `gpio[3].ios[5]` | 0, `pmod1_8` |
| 0x838 | `gpio[3].ios[6]` | 0, `pmod1_9` |
| 0x839 | `gpio[3].ios[7]` | 0, `pmod1_10` |
| 0x83a | `gpio[4].ios[0]` | 0, `pmodc_1` |
| 0x83b | `gpio[4].ios[1]` | 0, `pmodc_2` |
| 0x83c | `gpio[4].ios[2]` | 0, `pmodc_3` |
| 0x83d | `gpio[4].ios[3]` | 0, `pmodc_4` |
| 0x83e | `gpio[4].ios[4]` | 0, `pmodc_5` |
| 0x83f | `gpio[4].ios[5]` | 0, `pmodc_6` |
| 0x840 | `uart[0].rx` | 1, `ser0_rx` |
| 0x841 | `uart[1].rx` | 1, `ser1_rx`, `rph_rxd0`, `ah_tmpio0`, `mb8`, `pmod0_3` |
| 0x842 | `uart[2].rx` | 1, `ser1_rx`, `rs232_rx`, `rs485_rx`, `pmod1_3` |
| 0x843 | `spi[0].cipo` | 0, `appspi_d1`, `microsd_dat0` |
| 0x844 | `spi[1].cipo` | 0, `rph_g9`, `ah_tmpio12`, `pmod0_3` |
| 0x845 | `spi[2].cipo` | 0, `rph_g19`, `mb3`, `pmod1_3` |

## Regeneration

If any changes are made to the top configuration, the templates or the bus, you must regenerate the top.
You can do so using the top generation utility, which regenerates the pinmux, the bus and the sonata package which is used by the SystemVerilog generate statements throughout the project.

```sh
./util/top_gen.py
```
