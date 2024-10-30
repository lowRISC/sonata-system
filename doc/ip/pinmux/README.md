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
| 0x001 | `ser1_tx` | 0, `uart[1].tx` |
| 0x002 | `rs232_tx` | 0, `uart[4].tx` |
| 0x003 | `scl0` | 0, `i2c[0].scl` |
| 0x004 | `sda0` | 0, `i2c[0].sda` |
| 0x005 | `scl1` | 0, `i2c[1].scl` |
| 0x006 | `sda1` | 0, `i2c[1].sda` |
| 0x007 | `appspi_d0` | 0, `spi[0].copi` |
| 0x008 | `appspi_clk` | 0, `spi[0].sclk` |
| 0x009 | `appspi_cs` | 0, `spi[0].cs[0]` |
| 0x00a | `lcd_copi` | 0, `spi[1].copi` |
| 0x00b | `lcd_clk` | 0, `spi[1].sclk` |
| 0x00c | `lcd_cs` | 0, `spi[1].cs[0]` |
| 0x00d | `ethmac_copi` | 0, `spi[2].copi` |
| 0x00e | `ethmac_sclk` | 0, `spi[2].sclk` |
| 0x00f | `ethmac_cs` | 0, `spi[2].cs[0]` |
| 0x010 | `rph_g0` | 0, `i2c[0].sda`, `gpio[0].ios[0]` |
| 0x011 | `rph_g1` | 0, `i2c[0].scl`, `gpio[0].ios[1]` |
| 0x012 | `rph_g2_sda` | 0, `i2c[1].sda`, `gpio[0].ios[2]` |
| 0x013 | `rph_g3_scl` | 0, `i2c[1].scl`, `gpio[0].ios[3]` |
| 0x014 | `rph_g4` | 0, `gpio[0].ios[4]` |
| 0x015 | `rph_g5` | 0, `gpio[0].ios[5]` |
| 0x016 | `rph_g6` | 0, `gpio[0].ios[6]` |
| 0x017 | `rph_g7_ce1` | 0, `spi[3].cs[1]`, `gpio[0].ios[7]` |
| 0x018 | `rph_g8_ce0` | 0, `spi[3].cs[0]`, `gpio[0].ios[8]` |
| 0x019 | `rph_g9_cipo` | 0, `gpio[0].ios[9]` |
| 0x01a | `rph_g10_copi` | 0, `spi[3].copi`, `gpio[0].ios[10]` |
| 0x01b | `rph_g11_sclk` | 0, `spi[3].sclk`, `gpio[0].ios[11]` |
| 0x01c | `rph_g12` | 0, `gpio[0].ios[12]` |
| 0x01d | `rph_g13` | 0, `gpio[0].ios[13]` |
| 0x01e | `rph_txd0` | 0, `uart[2].tx`, `gpio[0].ios[14]` |
| 0x01f | `rph_rxd0` | 0, `gpio[0].ios[15]` |
| 0x020 | `rph_g16_ce2` | 0, `spi[4].cs[2]`, `gpio[0].ios[16]` |
| 0x021 | `rph_g17` | 0, `spi[4].cs[1]`, `gpio[0].ios[17]` |
| 0x022 | `rph_g18` | 0, `spi[4].cs[0]`, `gpio[0].ios[18]` |
| 0x023 | `rph_g19_cipo` | 0, `gpio[0].ios[19]` |
| 0x024 | `rph_g20_copi` | 0, `spi[4].copi`, `gpio[0].ios[20]` |
| 0x025 | `rph_g21_sclk` | 0, `spi[4].sclk`, `gpio[0].ios[21]` |
| 0x026 | `rph_g22` | 0, `gpio[0].ios[22]` |
| 0x027 | `rph_g23` | 0, `gpio[0].ios[23]` |
| 0x028 | `rph_g24` | 0, `gpio[0].ios[24]` |
| 0x029 | `rph_g25` | 0, `gpio[0].ios[25]` |
| 0x02a | `rph_g26` | 0, `gpio[0].ios[26]` |
| 0x02b | `rph_g27` | 0, `gpio[0].ios[27]` |
| 0x02c | `ah_tmpio0` | 0, `gpio[1].ios[0]` |
| 0x02d | `ah_tmpio1` | 0, `gpio[1].ios[1]`, `uart[3].tx` |
| 0x02e | `ah_tmpio2` | 0, `gpio[1].ios[2]` |
| 0x02f | `ah_tmpio3` | 0, `gpio[1].ios[3]` |
| 0x030 | `ah_tmpio4` | 0, `gpio[1].ios[4]` |
| 0x031 | `ah_tmpio5` | 0, `gpio[1].ios[5]` |
| 0x032 | `ah_tmpio6` | 0, `gpio[1].ios[6]` |
| 0x033 | `ah_tmpio7` | 0, `gpio[1].ios[7]` |
| 0x034 | `ah_tmpio8` | 0, `gpio[1].ios[8]` |
| 0x035 | `ah_tmpio9` | 0, `gpio[1].ios[9]` |
| 0x036 | `ah_tmpio10` | 0, `spi[3].cs[2]`, `gpio[1].ios[10]` |
| 0x037 | `ah_tmpio11` | 0, `spi[3].copi`, `gpio[1].ios[11]` |
| 0x038 | `ah_tmpio12` | 0, `gpio[1].ios[12]` |
| 0x039 | `ah_tmpio13` | 0, `spi[3].sclk`, `gpio[1].ios[13]` |
| 0x03a | `mb1` | 0, `spi[4].cs[3]` |
| 0x03b | `mb2` | 0, `spi[4].sclk` |
| 0x03c | `mb4` | 0, `spi[4].copi` |
| 0x03d | `mb5` | 0, `i2c[1].sda` |
| 0x03e | `mb6` | 0, `i2c[1].scl` |
| 0x03f | `mb7` | 0, `uart[3].tx` |
| 0x040 | `mb10` | 0, `pwm_out[0]` |
| 0x041 | `pmod0_1` | 0, `gpio[2].ios[0]`, `spi[2].cs[0]` |
| 0x042 | `pmod0_2` | 0, `gpio[2].ios[1]`, `spi[2].copi`, `uart[2].tx` |
| 0x043 | `pmod0_3` | 0, `gpio[2].ios[2]`, `i2c[0].scl` |
| 0x044 | `pmod0_4` | 0, `gpio[2].ios[3]`, `i2c[0].sda`, `spi[2].sclk` |
| 0x045 | `pmod0_5` | 0, `gpio[2].ios[4]` |
| 0x046 | `pmod0_6` | 0, `gpio[2].ios[5]` |
| 0x047 | `pmod0_7` | 0, `gpio[2].ios[6]` |
| 0x048 | `pmod0_8` | 0, `gpio[2].ios[7]` |
| 0x049 | `pmod1_1` | 0, `gpio[3].ios[0]`, `spi[3].cs[0]` |
| 0x04a | `pmod1_2` | 0, `gpio[3].ios[1]`, `spi[3].copi`, `uart[3].tx` |
| 0x04b | `pmod1_3` | 0, `gpio[3].ios[2]`, `i2c[1].scl` |
| 0x04c | `pmod1_4` | 0, `gpio[3].ios[3]`, `i2c[1].sda`, `spi[3].sclk` |
| 0x04d | `pmod1_5` | 0, `gpio[3].ios[4]` |
| 0x04e | `pmod1_6` | 0, `gpio[3].ios[5]` |
| 0x04f | `pmod1_7` | 0, `gpio[3].ios[6]` |
| 0x050 | `pmod1_8` | 0, `gpio[3].ios[7]` |
| 0x051 | `microsd_clk` | 0, `spi[3].sclk` |
| 0x052 | `microsd_cmd` | 0, `spi[3].copi` |
| 0x053 | `microsd_dat3` | 0, `spi[3].cs[3]` |

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
| 0x807 | `gpio[0].ios[7]` | 0, `rph_g7_ce1` |
| 0x808 | `gpio[0].ios[8]` | 0, `rph_g8_ce0` |
| 0x809 | `gpio[0].ios[9]` | 0, `rph_g9_cipo` |
| 0x80a | `gpio[0].ios[10]` | 0, `rph_g10_copi` |
| 0x80b | `gpio[0].ios[11]` | 0, `rph_g11_sclk` |
| 0x80c | `gpio[0].ios[12]` | 0, `rph_g12` |
| 0x80d | `gpio[0].ios[13]` | 0, `rph_g13` |
| 0x80e | `gpio[0].ios[14]` | 0, `rph_txd0` |
| 0x80f | `gpio[0].ios[15]` | 0, `rph_rxd0` |
| 0x810 | `gpio[0].ios[16]` | 0, `rph_g16_ce2` |
| 0x811 | `gpio[0].ios[17]` | 0, `rph_g17` |
| 0x812 | `gpio[0].ios[18]` | 0, `rph_g18` |
| 0x813 | `gpio[0].ios[19]` | 0, `rph_g19_cipo` |
| 0x814 | `gpio[0].ios[20]` | 0, `rph_g20_copi` |
| 0x815 | `gpio[0].ios[21]` | 0, `rph_g21_sclk` |
| 0x816 | `gpio[0].ios[22]` | 0, `rph_g22` |
| 0x817 | `gpio[0].ios[23]` | 0, `rph_g23` |
| 0x818 | `gpio[0].ios[24]` | 0, `rph_g24` |
| 0x819 | `gpio[0].ios[25]` | 0, `rph_g25` |
| 0x81a | `gpio[0].ios[26]` | 0, `rph_g26` |
| 0x81b | `gpio[0].ios[27]` | 0, `rph_g27` |
| 0x81c | `gpio[0].ios[28]` | 0, 0 |
| 0x81d | `gpio[0].ios[29]` | 0, 0 |
| 0x81e | `gpio[0].ios[30]` | 0, 0 |
| 0x81f | `gpio[0].ios[31]` | 0, 0 |
| 0x820 | `gpio[1].ios[0]` | 0, `ah_tmpio0` |
| 0x821 | `gpio[1].ios[1]` | 0, `ah_tmpio1` |
| 0x822 | `gpio[1].ios[2]` | 0, `ah_tmpio2` |
| 0x823 | `gpio[1].ios[3]` | 0, `ah_tmpio3` |
| 0x824 | `gpio[1].ios[4]` | 0, `ah_tmpio4` |
| 0x825 | `gpio[1].ios[5]` | 0, `ah_tmpio5` |
| 0x826 | `gpio[1].ios[6]` | 0, `ah_tmpio6` |
| 0x827 | `gpio[1].ios[7]` | 0, `ah_tmpio7` |
| 0x828 | `gpio[1].ios[8]` | 0, `ah_tmpio8` |
| 0x829 | `gpio[1].ios[9]` | 0, `ah_tmpio9` |
| 0x82a | `gpio[1].ios[10]` | 0, `ah_tmpio10` |
| 0x82b | `gpio[1].ios[11]` | 0, `ah_tmpio11` |
| 0x82c | `gpio[1].ios[12]` | 0, `ah_tmpio12` |
| 0x82d | `gpio[1].ios[13]` | 0, `ah_tmpio13` |
| 0x82e | `gpio[1].ios[14]` | 0, 0 |
| 0x82f | `gpio[1].ios[15]` | 0, 0 |
| 0x830 | `gpio[1].ios[16]` | 0, 0 |
| 0x831 | `gpio[1].ios[17]` | 0, 0 |
| 0x832 | `gpio[1].ios[18]` | 0, 0 |
| 0x833 | `gpio[1].ios[19]` | 0, 0 |
| 0x834 | `gpio[1].ios[20]` | 0, 0 |
| 0x835 | `gpio[1].ios[21]` | 0, 0 |
| 0x836 | `gpio[1].ios[22]` | 0, 0 |
| 0x837 | `gpio[1].ios[23]` | 0, 0 |
| 0x838 | `gpio[1].ios[24]` | 0, 0 |
| 0x839 | `gpio[1].ios[25]` | 0, 0 |
| 0x83a | `gpio[1].ios[26]` | 0, 0 |
| 0x83b | `gpio[1].ios[27]` | 0, 0 |
| 0x83c | `gpio[1].ios[28]` | 0, 0 |
| 0x83d | `gpio[1].ios[29]` | 0, 0 |
| 0x83e | `gpio[1].ios[30]` | 0, 0 |
| 0x83f | `gpio[1].ios[31]` | 0, 0 |
| 0x840 | `gpio[2].ios[0]` | 0, `pmod0_1` |
| 0x841 | `gpio[2].ios[1]` | 0, `pmod0_2` |
| 0x842 | `gpio[2].ios[2]` | 0, `pmod0_3` |
| 0x843 | `gpio[2].ios[3]` | 0, `pmod0_4` |
| 0x844 | `gpio[2].ios[4]` | 0, `pmod0_5` |
| 0x845 | `gpio[2].ios[5]` | 0, `pmod0_6` |
| 0x846 | `gpio[2].ios[6]` | 0, `pmod0_7` |
| 0x847 | `gpio[2].ios[7]` | 0, `pmod0_8` |
| 0x848 | `gpio[2].ios[8]` | 0, 0 |
| 0x849 | `gpio[2].ios[9]` | 0, 0 |
| 0x84a | `gpio[2].ios[10]` | 0, 0 |
| 0x84b | `gpio[2].ios[11]` | 0, 0 |
| 0x84c | `gpio[2].ios[12]` | 0, 0 |
| 0x84d | `gpio[2].ios[13]` | 0, 0 |
| 0x84e | `gpio[2].ios[14]` | 0, 0 |
| 0x84f | `gpio[2].ios[15]` | 0, 0 |
| 0x850 | `gpio[2].ios[16]` | 0, 0 |
| 0x851 | `gpio[2].ios[17]` | 0, 0 |
| 0x852 | `gpio[2].ios[18]` | 0, 0 |
| 0x853 | `gpio[2].ios[19]` | 0, 0 |
| 0x854 | `gpio[2].ios[20]` | 0, 0 |
| 0x855 | `gpio[2].ios[21]` | 0, 0 |
| 0x856 | `gpio[2].ios[22]` | 0, 0 |
| 0x857 | `gpio[2].ios[23]` | 0, 0 |
| 0x858 | `gpio[2].ios[24]` | 0, 0 |
| 0x859 | `gpio[2].ios[25]` | 0, 0 |
| 0x85a | `gpio[2].ios[26]` | 0, 0 |
| 0x85b | `gpio[2].ios[27]` | 0, 0 |
| 0x85c | `gpio[2].ios[28]` | 0, 0 |
| 0x85d | `gpio[2].ios[29]` | 0, 0 |
| 0x85e | `gpio[2].ios[30]` | 0, 0 |
| 0x85f | `gpio[2].ios[31]` | 0, 0 |
| 0x860 | `gpio[3].ios[0]` | 0, `pmod1_1` |
| 0x861 | `gpio[3].ios[1]` | 0, `pmod1_2` |
| 0x862 | `gpio[3].ios[2]` | 0, `pmod1_3` |
| 0x863 | `gpio[3].ios[3]` | 0, `pmod1_4` |
| 0x864 | `gpio[3].ios[4]` | 0, `pmod1_5` |
| 0x865 | `gpio[3].ios[5]` | 0, `pmod1_6` |
| 0x866 | `gpio[3].ios[6]` | 0, `pmod1_7` |
| 0x867 | `gpio[3].ios[7]` | 0, `pmod1_8` |
| 0x868 | `gpio[3].ios[8]` | 0, 0 |
| 0x869 | `gpio[3].ios[9]` | 0, 0 |
| 0x86a | `gpio[3].ios[10]` | 0, 0 |
| 0x86b | `gpio[3].ios[11]` | 0, 0 |
| 0x86c | `gpio[3].ios[12]` | 0, 0 |
| 0x86d | `gpio[3].ios[13]` | 0, 0 |
| 0x86e | `gpio[3].ios[14]` | 0, 0 |
| 0x86f | `gpio[3].ios[15]` | 0, 0 |
| 0x870 | `gpio[3].ios[16]` | 0, 0 |
| 0x871 | `gpio[3].ios[17]` | 0, 0 |
| 0x872 | `gpio[3].ios[18]` | 0, 0 |
| 0x873 | `gpio[3].ios[19]` | 0, 0 |
| 0x874 | `gpio[3].ios[20]` | 0, 0 |
| 0x875 | `gpio[3].ios[21]` | 0, 0 |
| 0x876 | `gpio[3].ios[22]` | 0, 0 |
| 0x877 | `gpio[3].ios[23]` | 0, 0 |
| 0x878 | `gpio[3].ios[24]` | 0, 0 |
| 0x879 | `gpio[3].ios[25]` | 0, 0 |
| 0x87a | `gpio[3].ios[26]` | 0, 0 |
| 0x87b | `gpio[3].ios[27]` | 0, 0 |
| 0x87c | `gpio[3].ios[28]` | 0, 0 |
| 0x87d | `gpio[3].ios[29]` | 0, 0 |
| 0x87e | `gpio[3].ios[30]` | 0, 0 |
| 0x87f | `gpio[3].ios[31]` | 0, 0 |
| 0x880 | `uart[0].rx` | 1, `ser0_rx` |
| 0x881 | `uart[1].rx` | 1, `ser1_rx` |
| 0x882 | `uart[2].rx` | 1, `rph_rxd0`, `pmod0_3` |
| 0x883 | `uart[3].rx` | 1, `ah_tmpio0`, `mb8`, `pmod1_3` |
| 0x884 | `uart[4].rx` | 1, `rs232_rx` |
| 0x885 | `spi[0].cipo` | 0, `appspi_d1` |
| 0x886 | `spi[1].cipo` | 0, 0 |
| 0x887 | `spi[2].cipo` | 0, `ethmac_cipo`, `pmod0_3` |
| 0x888 | `spi[3].cipo` | 0, `rph_g9_cipo`, `ah_tmpio12`, `pmod1_3`, `microsd_dat0` |
| 0x889 | `spi[4].cipo` | 0, `rph_g19_cipo`, `mb3` |

## Regeneration

If any changes are made to the top configuration, the templates or the bus, you must regenerate the top.
You can do so using the top generation utility, which regenerates the pinmux, the bus and the sonata package which is used by the SystemVerilog generate statements throughout the project.

```sh
./util/top_gen.py
```
