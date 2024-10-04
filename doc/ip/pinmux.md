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
| 0x000 | ser0_tx | 0, uart_tx_i(0) |
| 0x001 | ser1_tx | 0, uart_tx_i(1) |
| 0x002 | rs232_tx | 0, uart_tx_i(4) |
| 0x003 | scl0 | 0, i2c_scl_i(0) |
| 0x004 | sda0 | 0, i2c_sda_i(0) |
| 0x005 | scl1 | 0, i2c_scl_i(1) |
| 0x006 | sda1 | 0, i2c_sda_i(1) |
| 0x007 | appspi_d0 | 0, spi_tx_i(0) |
| 0x008 | appspi_clk | 0, spi_sck_i(0) |
| 0x009 | lcd_copi | 0, spi_tx_i(1) |
| 0x00a | lcd_clk | 0, spi_sck_i(1) |
| 0x00b | ethmac_copi | 0, spi_tx_i(2) |
| 0x00c | ethmac_sclk | 0, spi_sck_i(2) |
| 0x00d | rph_g0 | 0, i2c_sda_i(0), gpio_ios_i(0)(0) |
| 0x00e | rph_g1 | 0, i2c_scl_i(0), gpio_ios_i(0)(1) |
| 0x00f | rph_g2_sda | 0, i2c_sda_i(1), gpio_ios_i(0)(2) |
| 0x010 | rph_g3_scl | 0, i2c_scl_i(1), gpio_ios_i(0)(3) |
| 0x011 | rph_g4 | 0, gpio_ios_i(0)(4) |
| 0x012 | rph_g5 | 0, gpio_ios_i(0)(5) |
| 0x013 | rph_g6 | 0, gpio_ios_i(0)(6) |
| 0x014 | rph_g7_ce1 | 0, gpio_ios_i(0)(7) |
| 0x015 | rph_g8_ce0 | 0, gpio_ios_i(0)(8) |
| 0x016 | rph_g9_cipo | 0, gpio_ios_i(0)(9) |
| 0x017 | rph_g10_copi | 0, spi_tx_i(3), gpio_ios_i(0)(10) |
| 0x018 | rph_g11_sclk | 0, spi_sck_i(3), gpio_ios_i(0)(11) |
| 0x019 | rph_g12 | 0, gpio_ios_i(0)(12) |
| 0x01a | rph_g13 | 0, gpio_ios_i(0)(13) |
| 0x01b | rph_txd0 | 0, uart_tx_i(2), gpio_ios_i(0)(14) |
| 0x01c | rph_rxd0 | 0, gpio_ios_i(0)(15) |
| 0x01d | rph_g16_ce2 | 0, gpio_ios_i(0)(16) |
| 0x01e | rph_g17 | 0, gpio_ios_i(0)(17) |
| 0x01f | rph_g18 | 0, gpio_ios_i(0)(18) |
| 0x020 | rph_g19_cipo | 0, gpio_ios_i(0)(19) |
| 0x021 | rph_g20_copi | 0, spi_tx_i(4), gpio_ios_i(0)(20) |
| 0x022 | rph_g21_sclk | 0, spi_sck_i(4), gpio_ios_i(0)(21) |
| 0x023 | rph_g22 | 0, gpio_ios_i(0)(22) |
| 0x024 | rph_g23 | 0, gpio_ios_i(0)(23) |
| 0x025 | rph_g24 | 0, gpio_ios_i(0)(24) |
| 0x026 | rph_g25 | 0, gpio_ios_i(0)(25) |
| 0x027 | rph_g26 | 0, gpio_ios_i(0)(26) |
| 0x028 | rph_g27 | 0, gpio_ios_i(0)(27) |
| 0x029 | ah_tmpio0 | 0, gpio_ios_i(1)(0) |
| 0x02a | ah_tmpio1 | 0, gpio_ios_i(1)(1) |
| 0x02b | ah_tmpio2 | 0, gpio_ios_i(1)(2) |
| 0x02c | ah_tmpio3 | 0, gpio_ios_i(1)(3) |
| 0x02d | ah_tmpio4 | 0, gpio_ios_i(1)(4) |
| 0x02e | ah_tmpio5 | 0, gpio_ios_i(1)(5) |
| 0x02f | ah_tmpio6 | 0, gpio_ios_i(1)(6) |
| 0x030 | ah_tmpio7 | 0, gpio_ios_i(1)(7) |
| 0x031 | ah_tmpio8 | 0, gpio_ios_i(1)(8) |
| 0x032 | ah_tmpio9 | 0, gpio_ios_i(1)(9) |
| 0x033 | ah_tmpio10 | 0, gpio_ios_i(1)(10) |
| 0x034 | ah_tmpio11 | 0, spi_tx_i(3), gpio_ios_i(1)(11) |
| 0x035 | ah_tmpio12 | 0, gpio_ios_i(1)(12) |
| 0x036 | ah_tmpio13 | 0, spi_sck_i(3), gpio_ios_i(1)(13) |
| 0x037 | ah_tmpio14 | 0, gpio_ios_i(1)(14) |
| 0x038 | ah_tmpio15 | 0, gpio_ios_i(1)(15) |
| 0x039 | ah_tmpio16 | 0, gpio_ios_i(1)(16) |
| 0x03a | ah_tmpio17 | 0, gpio_ios_i(1)(17) |
| 0x03b | mb2 | 0, spi_sck_i(4) |
| 0x03c | mb4 | 0, spi_tx_i(4) |
| 0x03d | mb5 | 0, i2c_sda_i(1) |
| 0x03e | mb6 | 0, i2c_scl_i(1) |
| 0x03f | mb7 | 0, uart_tx_i(3) |
| 0x040 | pmod0[0] | 0, gpio_ios_i(2)(0) |
| 0x041 | pmod0[1] | 0, gpio_ios_i(2)(1) |
| 0x042 | pmod0[2] | 0, gpio_ios_i(2)(2) |
| 0x043 | pmod0[3] | 0, gpio_ios_i(2)(3) |
| 0x044 | pmod0[4] | 0, gpio_ios_i(2)(4) |
| 0x045 | pmod0[5] | 0, gpio_ios_i(2)(5) |
| 0x046 | pmod0[6] | 0, gpio_ios_i(2)(6) |
| 0x047 | pmod0[7] | 0, gpio_ios_i(2)(7) |
| 0x048 | pmod1[0] | 0, gpio_ios_i(2)(8) |
| 0x049 | pmod1[1] | 0, gpio_ios_i(2)(9) |
| 0x04a | pmod1[2] | 0, gpio_ios_i(2)(10) |
| 0x04b | pmod1[3] | 0, gpio_ios_i(2)(11) |
| 0x04c | pmod1[4] | 0, gpio_ios_i(2)(12) |
| 0x04d | pmod1[5] | 0, gpio_ios_i(2)(13) |
| 0x04e | pmod1[6] | 0, gpio_ios_i(2)(14) |
| 0x04f | pmod1[7] | 0, gpio_ios_i(2)(15) |

Besides the output pin selectors, there are also selectors for which pin should drive block inputs:

| Address | Block input | Possible pin inputs |
|---------|-------------|---------------------|
| 0x800 | uart_rx_o(0) | 1'b1, ser0_rx, |
| 0x801 | uart_rx_o(1) | 1'b1, ser1_rx, |
| 0x802 | uart_rx_o(2) | 1'b1, rph_rxd0, |
| 0x803 | uart_rx_o(3) | 1'b1, mb8, |
| 0x804 | uart_rx_o(4) | 1'b1, rs232_rx, |
| 0x805 | spi_rx_o(0) | 1'b0, appspi_d1, |
| 0x806 | spi_rx_o(1) | 1'b0, 1'b0, |
| 0x807 | spi_rx_o(2) | 1'b0, ethmac_cipo, |
| 0x808 | spi_rx_o(3) | 1'b0, rph_g9_cipo, ah_tmpio12, |
| 0x809 | spi_rx_o(4) | 1'b0, rph_g19_cipo, mb3, |
| 0x80a | gpio_ios_o(0)(0) | 1'b0, rph_g0, |
| 0x80b | gpio_ios_o(1)(0) | 1'b0, ah_tmpio0, |
| 0x80c | gpio_ios_o(2)(0) | 1'b0, pmod0[0], |
| 0x80d | gpio_ios_o(0)(1) | 1'b0, rph_g1, |
| 0x80e | gpio_ios_o(1)(1) | 1'b0, ah_tmpio1, |
| 0x80f | gpio_ios_o(2)(1) | 1'b0, pmod0[1], |
| 0x810 | gpio_ios_o(0)(2) | 1'b0, rph_g2_sda, |
| 0x811 | gpio_ios_o(1)(2) | 1'b0, ah_tmpio2, |
| 0x812 | gpio_ios_o(2)(2) | 1'b0, pmod0[2], |
| 0x813 | gpio_ios_o(0)(3) | 1'b0, rph_g3_scl, |
| 0x814 | gpio_ios_o(1)(3) | 1'b0, ah_tmpio3, |
| 0x815 | gpio_ios_o(2)(3) | 1'b0, pmod0[3], |
| 0x816 | gpio_ios_o(0)(4) | 1'b0, rph_g4, |
| 0x817 | gpio_ios_o(1)(4) | 1'b0, ah_tmpio4, |
| 0x818 | gpio_ios_o(2)(4) | 1'b0, pmod0[4], |
| 0x819 | gpio_ios_o(0)(5) | 1'b0, rph_g5, |
| 0x81a | gpio_ios_o(1)(5) | 1'b0, ah_tmpio5, |
| 0x81b | gpio_ios_o(2)(5) | 1'b0, pmod0[5], |
| 0x81c | gpio_ios_o(0)(6) | 1'b0, rph_g6, |
| 0x81d | gpio_ios_o(1)(6) | 1'b0, ah_tmpio6, |
| 0x81e | gpio_ios_o(2)(6) | 1'b0, pmod0[6], |
| 0x81f | gpio_ios_o(0)(7) | 1'b0, rph_g7_ce1, |
| 0x820 | gpio_ios_o(1)(7) | 1'b0, ah_tmpio7, |
| 0x821 | gpio_ios_o(2)(7) | 1'b0, pmod0[7], |
| 0x822 | gpio_ios_o(0)(8) | 1'b0, rph_g8_ce0, |
| 0x823 | gpio_ios_o(1)(8) | 1'b0, ah_tmpio8, |
| 0x824 | gpio_ios_o(2)(8) | 1'b0, pmod1[0], |
| 0x825 | gpio_ios_o(0)(9) | 1'b0, rph_g9_cipo, |
| 0x826 | gpio_ios_o(1)(9) | 1'b0, ah_tmpio9, |
| 0x827 | gpio_ios_o(2)(9) | 1'b0, pmod1[1], |
| 0x828 | gpio_ios_o(0)(10) | 1'b0, rph_g10_copi, |
| 0x829 | gpio_ios_o(1)(10) | 1'b0, ah_tmpio10, |
| 0x82a | gpio_ios_o(2)(10) | 1'b0, pmod1[2], |
| 0x82b | gpio_ios_o(0)(11) | 1'b0, rph_g11_sclk, |
| 0x82c | gpio_ios_o(1)(11) | 1'b0, ah_tmpio11, |
| 0x82d | gpio_ios_o(2)(11) | 1'b0, pmod1[3], |
| 0x82e | gpio_ios_o(0)(12) | 1'b0, rph_g12, |
| 0x82f | gpio_ios_o(1)(12) | 1'b0, ah_tmpio12, |
| 0x830 | gpio_ios_o(2)(12) | 1'b0, pmod1[4], |
| 0x831 | gpio_ios_o(0)(13) | 1'b0, rph_g13, |
| 0x832 | gpio_ios_o(1)(13) | 1'b0, ah_tmpio13, |
| 0x833 | gpio_ios_o(2)(13) | 1'b0, pmod1[5], |
| 0x834 | gpio_ios_o(0)(14) | 1'b0, rph_txd0, |
| 0x835 | gpio_ios_o(1)(14) | 1'b0, ah_tmpio14, |
| 0x836 | gpio_ios_o(2)(14) | 1'b0, pmod1[6], |
| 0x837 | gpio_ios_o(0)(15) | 1'b0, rph_rxd0, |
| 0x838 | gpio_ios_o(1)(15) | 1'b0, ah_tmpio15, |
| 0x839 | gpio_ios_o(2)(15) | 1'b0, pmod1[7], |
| 0x83a | gpio_ios_o(0)(16) | 1'b0, rph_g16_ce2, |
| 0x83b | gpio_ios_o(1)(16) | 1'b0, ah_tmpio16, |
| 0x83c | gpio_ios_o(2)(16) | 1'b0, 1'b0, |
| 0x83d | gpio_ios_o(0)(17) | 1'b0, rph_g17, |
| 0x83e | gpio_ios_o(1)(17) | 1'b0, ah_tmpio17, |
| 0x83f | gpio_ios_o(2)(17) | 1'b0, 1'b0, |
| 0x840 | gpio_ios_o(0)(18) | 1'b0, rph_g18, |
| 0x841 | gpio_ios_o(1)(18) | 1'b0, 1'b0, |
| 0x842 | gpio_ios_o(2)(18) | 1'b0, 1'b0, |
| 0x843 | gpio_ios_o(0)(19) | 1'b0, rph_g19_cipo, |
| 0x844 | gpio_ios_o(1)(19) | 1'b0, 1'b0, |
| 0x845 | gpio_ios_o(2)(19) | 1'b0, 1'b0, |
| 0x846 | gpio_ios_o(0)(20) | 1'b0, rph_g20_copi, |
| 0x847 | gpio_ios_o(1)(20) | 1'b0, 1'b0, |
| 0x848 | gpio_ios_o(2)(20) | 1'b0, 1'b0, |
| 0x849 | gpio_ios_o(0)(21) | 1'b0, rph_g21_sclk, |
| 0x84a | gpio_ios_o(1)(21) | 1'b0, 1'b0, |
| 0x84b | gpio_ios_o(2)(21) | 1'b0, 1'b0, |
| 0x84c | gpio_ios_o(0)(22) | 1'b0, rph_g22, |
| 0x84d | gpio_ios_o(1)(22) | 1'b0, 1'b0, |
| 0x84e | gpio_ios_o(2)(22) | 1'b0, 1'b0, |
| 0x84f | gpio_ios_o(0)(23) | 1'b0, rph_g23, |
| 0x850 | gpio_ios_o(1)(23) | 1'b0, 1'b0, |
| 0x851 | gpio_ios_o(2)(23) | 1'b0, 1'b0, |
| 0x852 | gpio_ios_o(0)(24) | 1'b0, rph_g24, |
| 0x853 | gpio_ios_o(1)(24) | 1'b0, 1'b0, |
| 0x854 | gpio_ios_o(2)(24) | 1'b0, 1'b0, |
| 0x855 | gpio_ios_o(0)(25) | 1'b0, rph_g25, |
| 0x856 | gpio_ios_o(1)(25) | 1'b0, 1'b0, |
| 0x857 | gpio_ios_o(2)(25) | 1'b0, 1'b0, |
| 0x858 | gpio_ios_o(0)(26) | 1'b0, rph_g26, |
| 0x859 | gpio_ios_o(1)(26) | 1'b0, 1'b0, |
| 0x85a | gpio_ios_o(2)(26) | 1'b0, 1'b0, |
| 0x85b | gpio_ios_o(0)(27) | 1'b0, rph_g27, |
| 0x85c | gpio_ios_o(1)(27) | 1'b0, 1'b0, |
| 0x85d | gpio_ios_o(2)(27) | 1'b0, 1'b0, |
| 0x85e | gpio_ios_o(0)(28) | 1'b0, 1'b0, |
| 0x85f | gpio_ios_o(1)(28) | 1'b0, 1'b0, |
| 0x860 | gpio_ios_o(2)(28) | 1'b0, 1'b0, |
| 0x861 | gpio_ios_o(0)(29) | 1'b0, 1'b0, |
| 0x862 | gpio_ios_o(1)(29) | 1'b0, 1'b0, |
| 0x863 | gpio_ios_o(2)(29) | 1'b0, 1'b0, |
| 0x864 | gpio_ios_o(0)(30) | 1'b0, 1'b0, |
| 0x865 | gpio_ios_o(1)(30) | 1'b0, 1'b0, |
| 0x866 | gpio_ios_o(2)(30) | 1'b0, 1'b0, |
| 0x867 | gpio_ios_o(0)(31) | 1'b0, 1'b0, |
| 0x868 | gpio_ios_o(1)(31) | 1'b0, 1'b0, |
| 0x869 | gpio_ios_o(2)(31) | 1'b0, 1'b0, |

## Regeneration

If any changes are made to the top configuration, the templates or the bus, you must regenerate the top.
You can do so using the top generation utility, which regenerates the pinmux, the bus and the sonata package which is used by the SystemVerilog generate statements throughout the project.

```sh
./util/top_gen.py
```
