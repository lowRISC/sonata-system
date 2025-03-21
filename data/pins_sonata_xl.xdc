## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

# This file is for physical constraints for the Sonata XL board.

# Using the names in the PCB design, they should match this file with a case-insensitive search:
# https://github.com/newaetech/sonata-pcb/tree/main

## Clocks
set_property -dict { PACKAGE_PIN N21 IOSTANDARD LVCMOS33 } [get_ports mainClk]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets tck_i]

## Reset
set_property -dict { PACKAGE_PIN N3 IOSTANDARD LVCMOS33 } [get_ports {nrst}]

## General purpose LEDs
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports {usrLed[0]}]
set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports {usrLed[1]}]
set_property -dict { PACKAGE_PIN K16 IOSTANDARD LVCMOS33 } [get_ports {usrLed[2]}]
set_property -dict { PACKAGE_PIN K17 IOSTANDARD LVCMOS33 } [get_ports {usrLed[3]}]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports {usrLed[4]}]
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 } [get_ports {usrLed[5]}]
set_property -dict { PACKAGE_PIN M15 IOSTANDARD LVCMOS33 } [get_ports {usrLed[6]}]
set_property -dict { PACKAGE_PIN L15 IOSTANDARD LVCMOS33 } [get_ports {usrLed[7]}]

## User JTAG (marked as USR_JTAG on schematic)
set_property -dict { PACKAGE_PIN G21 IOSTANDARD LVCMOS33 } [get_ports tck_i]
set_property -dict { PACKAGE_PIN F24 IOSTANDARD LVCMOS33 } [get_ports tms_i]
set_property -dict { PACKAGE_PIN K23 IOSTANDARD LVCMOS33 } [get_ports td_i]
set_property -dict { PACKAGE_PIN G24 IOSTANDARD LVCMOS33 } [get_ports td_o]
# Pack TDO register in the matching IOB for best output timings.
# Include parent name to prevent accidental matches.
set_property IOB TRUE [get_cells -hier -filter {NAME =~ */i_dmi_jtag_tap/td_o_reg}]

## Switch and button input
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports {usrSw[0]}]
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33 } [get_ports {usrSw[1]}]
set_property -dict { PACKAGE_PIN J19 IOSTANDARD LVCMOS33 } [get_ports {usrSw[2]}]
set_property -dict { PACKAGE_PIN H19 IOSTANDARD LVCMOS33 } [get_ports {usrSw[3]}]
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports {usrSw[4]}]
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports {usrSw[5]}]
set_property -dict { PACKAGE_PIN K20 IOSTANDARD LVCMOS33 } [get_ports {usrSw[6]}]
set_property -dict { PACKAGE_PIN J20 IOSTANDARD LVCMOS33 } [get_ports {usrSw[7]}]
set_property -dict { PACKAGE_PIN J8  IOSTANDARD LVCMOS18 } [get_ports {navSw[0]}]
set_property -dict { PACKAGE_PIN F8  IOSTANDARD LVCMOS18 } [get_ports {navSw[1]}]
set_property -dict { PACKAGE_PIN F7  IOSTANDARD LVCMOS18 } [get_ports {navSw[2]}]
set_property -dict { PACKAGE_PIN H9  IOSTANDARD LVCMOS18 } [get_ports {navSw[3]}]
set_property -dict { PACKAGE_PIN G9  IOSTANDARD LVCMOS18 } [get_ports {navSw[4]}]
set_property -dict { PACKAGE_PIN F5  IOSTANDARD LVCMOS18 } [get_ports {selSw[0]}]
set_property -dict { PACKAGE_PIN E5  IOSTANDARD LVCMOS18 } [get_ports {selSw[1]}]
set_property -dict { PACKAGE_PIN D5  IOSTANDARD LVCMOS18 } [get_ports {selSw[2]}]
set_property PULLTYPE PULLUP [get_ports usrSw[*]]
set_property PULLTYPE PULLUP [get_ports navSw[*]]
set_property PULLTYPE PULLUP [get_ports selSw[*]]

## CHERI error LEDs
set_property -dict { PACKAGE_PIN N8  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[0]}]
set_property -dict { PACKAGE_PIN K3  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[1]}]
set_property -dict { PACKAGE_PIN J3  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[2]}]
set_property -dict { PACKAGE_PIN M7  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[3]}]
set_property -dict { PACKAGE_PIN L7  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[4]}]
set_property -dict { PACKAGE_PIN M4  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[5]}]
set_property -dict { PACKAGE_PIN L4  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[6]}]
set_property -dict { PACKAGE_PIN L5  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[7]}]
set_property -dict { PACKAGE_PIN K5  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[8]}]

## USRUSB interface
set_property -dict { PACKAGE_PIN C3  IOSTANDARD LVCMOS18 } [get_ports {usrusb_spd}]
set_property -dict { PACKAGE_PIN C2  IOSTANDARD LVCMOS18 } [get_ports {usrusb_v_p}]
set_property -dict { PACKAGE_PIN B2  IOSTANDARD LVCMOS18 } [get_ports {usrusb_v_n}]
set_property -dict { PACKAGE_PIN A3  IOSTANDARD LVCMOS18 } [get_ports {usrusb_vpo}]
set_property -dict { PACKAGE_PIN A2  IOSTANDARD LVCMOS18 } [get_ports {usrusb_vmo}]
set_property -dict { PACKAGE_PIN C1  IOSTANDARD LVCMOS18 } [get_ports {usrusb_rcv}]
set_property -dict { PACKAGE_PIN B1  IOSTANDARD LVCMOS18 } [get_ports {usrusb_softcn}]
set_property -dict { PACKAGE_PIN F2  IOSTANDARD LVCMOS18 } [get_ports {usrusb_oe}]
set_property -dict { PACKAGE_PIN E1  IOSTANDARD LVCMOS18 } [get_ports {usrusb_sus}]
set_property -dict { PACKAGE_PIN D1  IOSTANDARD LVCMOS18 } [get_ports {usrusb_vbusdetect}]

## PMOD0
set_property -dict { PACKAGE_PIN G22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[0]}] ;# PMOD0_1 on schematic
set_property -dict { PACKAGE_PIN H23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[1]}] ;# PMOD0_2
set_property -dict { PACKAGE_PIN J23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[2]}] ;# ...
set_property -dict { PACKAGE_PIN F22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[3]}]
set_property -dict { PACKAGE_PIN E23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[4]}]
set_property -dict { PACKAGE_PIN H24 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[5]}]
set_property -dict { PACKAGE_PIN J24 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[6]}]
set_property -dict { PACKAGE_PIN F23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod0[7]}]

## PMODC (pins between PMOD0 and PMOD1 on shared connector)
set_property -dict { PACKAGE_PIN J25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmodc[0]}] ;# PMODC_1 on schematic
set_property -dict { PACKAGE_PIN J26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmodc[1]}] ;# PMODC_2
set_property -dict { PACKAGE_PIN L19 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmodc[2]}] ;# ...
set_property -dict { PACKAGE_PIN F25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmodc[3]}]
set_property -dict { PACKAGE_PIN G26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmodc[4]}]
set_property -dict { PACKAGE_PIN G25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmodc[5]}]

## PMOD1
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[0]}]
set_property -dict { PACKAGE_PIN H18 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[1]}]
set_property -dict { PACKAGE_PIN G20 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[2]}]
set_property -dict { PACKAGE_PIN K22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[3]}]
set_property -dict { PACKAGE_PIN K21 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[4]}]
set_property -dict { PACKAGE_PIN J21 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[5]}]
set_property -dict { PACKAGE_PIN H21 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[6]}]
set_property -dict { PACKAGE_PIN H22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports {pmod1[7]}]

## Status LEDs
set_property -dict { PACKAGE_PIN N7  IOSTANDARD LVCMOS33 } [get_ports led_legacy]
set_property -dict { PACKAGE_PIN N6  IOSTANDARD LVCMOS33 } [get_ports led_cheri]
set_property -dict { PACKAGE_PIN M6  IOSTANDARD LVCMOS33 } [get_ports led_halted]
set_property -dict { PACKAGE_PIN M5  IOSTANDARD LVCMOS33 } [get_ports led_bootok]

## LCD display
set_property -dict { PACKAGE_PIN P6  IOSTANDARD LVCMOS33 } [get_ports lcd_rst]
set_property -dict { PACKAGE_PIN L3  IOSTANDARD LVCMOS33 } [get_ports lcd_dc]
set_property -dict { PACKAGE_PIN M2  IOSTANDARD LVCMOS33 } [get_ports lcd_copi]
set_property -dict { PACKAGE_PIN P5  IOSTANDARD LVCMOS33 } [get_ports lcd_clk]
set_property -dict { PACKAGE_PIN P3  IOSTANDARD LVCMOS33 } [get_ports lcd_cs]
set_property -dict { PACKAGE_PIN R3  IOSTANDARD LVCMOS33 } [get_ports lcd_backlight]

## UART 0
set_property -dict { PACKAGE_PIN D25 IOSTANDARD LVCMOS33 } [get_ports ser0_tx]
set_property -dict { PACKAGE_PIN D26 IOSTANDARD LVCMOS33 } [get_ports ser0_rx]

## UART 1
set_property -dict { PACKAGE_PIN E26 IOSTANDARD LVCMOS33 } [get_ports ser1_tx]
set_property -dict { PACKAGE_PIN H26 IOSTANDARD LVCMOS33 } [get_ports ser1_rx]

## UART RS-232
set_property -dict { PACKAGE_PIN N4  IOSTANDARD LVCMOS33 } [get_ports rs232_tx]
set_property -dict { PACKAGE_PIN U1  IOSTANDARD LVCMOS33 } [get_ports rs232_rx]

## UART RS-485
set_property -dict { PACKAGE_PIN P1  IOSTANDARD LVCMOS33 } [get_ports rs485_ro]
set_property -dict { PACKAGE_PIN T4  IOSTANDARD LVCMOS33 } [get_ports rs485_de]
set_property -dict { PACKAGE_PIN T3  IOSTANDARD LVCMOS33 } [get_ports rs485_ren]
set_property -dict { PACKAGE_PIN U2  IOSTANDARD LVCMOS33 } [get_ports rs485_di]

## QWIIC and Arduino Shield
set_property -dict { PACKAGE_PIN R8 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports sda0]
set_property -dict { PACKAGE_PIN U5 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports scl0]

## QWIIC
set_property -dict { PACKAGE_PIN T5 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports sda1]
set_property -dict { PACKAGE_PIN U6 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports scl1]

## mikroBUS Click
## Reset (connected to GPO)
set_property -dict { PACKAGE_PIN R1  IOSTANDARD LVCMOS33 } [get_ports mb0]
## SPI chip select
set_property -dict { PACKAGE_PIN T2  IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports mb1]
## SPI SCLK
set_property -dict { PACKAGE_PIN R2  IOSTANDARD LVCMOS33 } [get_ports mb2]
## SPI CIPO
set_property -dict { PACKAGE_PIN K1  IOSTANDARD LVCMOS33 } [get_ports mb3]
## SPI COPI
set_property -dict { PACKAGE_PIN L2  IOSTANDARD LVCMOS33 } [get_ports mb4]
## I2C SDA
set_property -dict { PACKAGE_PIN J1  IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports mb5]
## I2C SCL
set_property -dict { PACKAGE_PIN N1  IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports mb6]
## Enable pull-ups because this I2C bus will often be undriven externally.
set_property PULLTYPE PULLUP [get_ports mb5]
set_property PULLTYPE PULLUP [get_ports mb6]
## UART TX
set_property -dict { PACKAGE_PIN M1  IOSTANDARD LVCMOS33 } [get_ports mb7]
## UART RX
set_property -dict { PACKAGE_PIN U4  IOSTANDARD LVCMOS33 } [get_ports mb8]
## Interrupt (connected to GPI)
set_property -dict { PACKAGE_PIN R6  IOSTANDARD LVCMOS33 } [get_ports mb9]
## PWM
set_property -dict { PACKAGE_PIN R5  IOSTANDARD LVCMOS33 } [get_ports mb10]

## R-Pi Header

## GPIO/SPI1 bus
set_property -dict { PACKAGE_PIN T25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g21_sclk]
set_property -dict { PACKAGE_PIN R23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g20_copi]
set_property -dict { PACKAGE_PIN P26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g19_cipo]
## SPI1 CE0
set_property -dict { PACKAGE_PIN L25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g18]
## SPI1 CE1
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g17]
set_property -dict { PACKAGE_PIN T23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g16_ce2]

## GPIO/SPI0 bus
set_property -dict { PACKAGE_PIN M21 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g11_sclk]
set_property -dict { PACKAGE_PIN N19 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g10_copi]
set_property -dict { PACKAGE_PIN R21 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g9_cipo]
set_property -dict { PACKAGE_PIN P19 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g8_ce0]
set_property -dict { PACKAGE_PIN M19 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g7_ce1]

## GPIO/I2C bus
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g2_sda]
set_property -dict { PACKAGE_PIN M24 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g3_scl]

## UART
set_property -dict { PACKAGE_PIN R14 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_txd0]
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_rxd0]

## Other GPIO
set_property -dict { PACKAGE_PIN L22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g4]
set_property -dict { PACKAGE_PIN T22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g5]
set_property -dict { PACKAGE_PIN R26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g6]
set_property -dict { PACKAGE_PIN T24 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g12]
set_property -dict { PACKAGE_PIN R17 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g13]
set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g22]
set_property -dict { PACKAGE_PIN M20 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g23]
set_property -dict { PACKAGE_PIN L23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g24]
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g25]
set_property -dict { PACKAGE_PIN N22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g26]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g27]

## ID_SC/SD - I2C bus for HAT ID EEPROM; pull-ups are on the HAT itself
set_property -dict { PACKAGE_PIN P21 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g1] ;# ID_SC
set_property -dict { PACKAGE_PIN P23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports rph_g0] ;# ID_SD
## Enable pull-ups because this I2C bus will often be undriven externally.
set_property PULLTYPE PULLUP [get_ports rph_g1]
set_property PULLTYPE PULLUP [get_ports rph_g0]

## Arduino Shield
## SPI SCLK
set_property -dict { PACKAGE_PIN M22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio13]
## SPI CIPO
set_property -dict { PACKAGE_PIN N23 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio12]
## SPI COPI
set_property -dict { PACKAGE_PIN K26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio11]
## SPI chip select
set_property -dict { PACKAGE_PIN R20 IOSTANDARD LVCMOS33 } [get_ports ah_tmpio10]
## GPIO
set_property -dict { PACKAGE_PIN N24 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio9]
set_property -dict { PACKAGE_PIN P24 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio8]
set_property -dict { PACKAGE_PIN N17 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio7]
set_property -dict { PACKAGE_PIN P25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio6]
set_property -dict { PACKAGE_PIN M26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio5]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio4]
set_property -dict { PACKAGE_PIN R22 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio3]
set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio2]
set_property -dict { PACKAGE_PIN N26 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio1]
set_property -dict { PACKAGE_PIN R25 IOSTANDARD LVCMOS33 IO_BUFFER_TYPE NONE } [get_ports ah_tmpio0]
## Digital inputs from analog(ue) pins via 1k resistor
set_property -dict { PACKAGE_PIN K18 IOSTANDARD LVCMOS33 } [get_ports {ard_an_di[0]}] ;# ANALOG0_DIGITAL
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports {ard_an_di[1]}] ;# ANALOG1_DIGITAL
set_property -dict { PACKAGE_PIN J16 IOSTANDARD LVCMOS33 } [get_ports {ard_an_di[2]}] ;# ...
set_property -dict { PACKAGE_PIN E17 IOSTANDARD LVCMOS33 } [get_ports {ard_an_di[3]}]
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports {ard_an_di[4]}]
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports {ard_an_di[5]}]
## Analog(ue) input pairs from analog(ue) pins via buffer and impedance network
## UG480: "...an IOSTANDARD must be selected that is compatible for the bank
##         even though the IOSTANDARD does not affect the input programming."
set_property -dict { PACKAGE_PIN E6  IOSTANDARD LVCMOS18 } [get_ports {ard_an_p[0]}] ;# VAUX4P, ANALOG0_P
set_property -dict { PACKAGE_PIN D6  IOSTANDARD LVCMOS18 } [get_ports {ard_an_n[0]}] ;# VAUX4N, ANALOG0_N
set_property -dict { PACKAGE_PIN H8  IOSTANDARD LVCMOS18 } [get_ports {ard_an_p[1]}] ;# VAUX12P, ANALOG1_P
set_property -dict { PACKAGE_PIN G8  IOSTANDARD LVCMOS18 } [get_ports {ard_an_n[1]}] ;# VAUX12N, ANALOG1_N
set_property -dict { PACKAGE_PIN H7  IOSTANDARD LVCMOS18 } [get_ports {ard_an_p[2]}] ;# VAUX5P, ...
set_property -dict { PACKAGE_PIN G7  IOSTANDARD LVCMOS18 } [get_ports {ard_an_n[2]}] ;# VAUX5N
set_property -dict { PACKAGE_PIN H6  IOSTANDARD LVCMOS18 } [get_ports {ard_an_p[3]}] ;# VAUX13P
set_property -dict { PACKAGE_PIN G6  IOSTANDARD LVCMOS18 } [get_ports {ard_an_n[3]}] ;# VAUX13N
set_property -dict { PACKAGE_PIN J6  IOSTANDARD LVCMOS18 } [get_ports {ard_an_p[4]}] ;# VAUX6P
set_property -dict { PACKAGE_PIN J5  IOSTANDARD LVCMOS18 } [get_ports {ard_an_n[4]}] ;# VAUX6N
set_property -dict { PACKAGE_PIN L8  IOSTANDARD LVCMOS18 } [get_ports {ard_an_p[5]}] ;# VAUX14P
set_property -dict { PACKAGE_PIN K8  IOSTANDARD LVCMOS18 } [get_ports {ard_an_n[5]}] ;# VAUX14N

## RGB LED
set_property -dict { PACKAGE_PIN G16 IOSTANDARD LVCMOS33 } [get_ports rgbled0]
set_property -dict { PACKAGE_PIN H16 IOSTANDARD LVCMOS33 } [get_ports rgbled_en]

## SPI Flash
set_property -dict { PACKAGE_PIN D19 IOSTANDARD LVCMOS33 } [get_ports appspi_clk]
set_property -dict { PACKAGE_PIN C19 IOSTANDARD LVCMOS33 } [get_ports appspi_d0]
set_property -dict { PACKAGE_PIN E20 IOSTANDARD LVCMOS33 } [get_ports appspi_d1]
set_property -dict { PACKAGE_PIN D20 IOSTANDARD LVCMOS33 } [get_ports appspi_d2]
set_property -dict { PACKAGE_PIN B20 IOSTANDARD LVCMOS33 } [get_ports appspi_d3]
set_property -dict { PACKAGE_PIN A20 IOSTANDARD LVCMOS33 } [get_ports appspi_cs]
set_property PULLTYPE PULLUP [get_ports appspi_d1]

## Ethernet MAC
set_property -dict { PACKAGE_PIN H3  IOSTANDARD LVCMOS18 } [get_ports ethmac_rst]
set_property -dict { PACKAGE_PIN G4  IOSTANDARD LVCMOS18 } [get_ports ethmac_copi]
set_property -dict { PACKAGE_PIN G5  IOSTANDARD LVCMOS18 } [get_ports ethmac_sclk]
set_property -dict { PACKAGE_PIN F4  IOSTANDARD LVCMOS18 } [get_ports ethmac_cipo]
set_property -dict { PACKAGE_PIN G2  IOSTANDARD LVCMOS18 } [get_ports ethmac_intr]
set_property PULLTYPE PULLUP [get_ports ethmac_intr]
set_property -dict { PACKAGE_PIN G1  IOSTANDARD LVCMOS18 } [get_ports ethmac_cs]

## MicroSD card slot
set_property -dict { PACKAGE_PIN P8  IOSTANDARD LVCMOS33 } [get_ports microsd_clk]
set_property -dict { PACKAGE_PIN H1  IOSTANDARD LVCMOS33 } [get_ports microsd_dat0]
# set_property -dict { PACKAGE_PIN R7  IOSTANDARD LVCMOS33 } [get_ports microsd_dat1] // unused in SPI bus mode
# set_property -dict { PACKAGE_PIN H2  IOSTANDARD LVCMOS33 } [get_ports microsd_dat2] // unused in SPI bus mode
set_property -dict { PACKAGE_PIN T7  IOSTANDARD LVCMOS33 } [get_ports microsd_dat3]
set_property -dict { PACKAGE_PIN T8  IOSTANDARD LVCMOS33 } [get_ports microsd_cmd]
set_property -dict { PACKAGE_PIN K7  IOSTANDARD LVCMOS18 } [get_ports microsd_det]

# HyperRAM
# No HyperRAM on Sonata XL

## XL expansion headers
set_property -dict { PACKAGE_PIN AE5  IOSTANDARD LVCMOS33 } [get_ports ex0[0]]
set_property -dict { PACKAGE_PIN V9   IOSTANDARD LVCMOS33 } [get_ports ex0[1]]
set_property -dict { PACKAGE_PIN AF5  IOSTANDARD LVCMOS33 } [get_ports ex0[2]]
set_property -dict { PACKAGE_PIN V8   IOSTANDARD LVCMOS33 } [get_ports ex0[3]]
set_property -dict { PACKAGE_PIN AD5  IOSTANDARD LVCMOS33 } [get_ports ex0[4]]
set_property -dict { PACKAGE_PIN U7   IOSTANDARD LVCMOS33 } [get_ports ex0[5]]
set_property -dict { PACKAGE_PIN AF4  IOSTANDARD LVCMOS33 } [get_ports ex0[6]]
set_property -dict { PACKAGE_PIN V7   IOSTANDARD LVCMOS33 } [get_ports ex0[7]]
set_property -dict { PACKAGE_PIN AF3  IOSTANDARD LVCMOS33 } [get_ports ex0[8]]
set_property -dict { PACKAGE_PIN Y8   IOSTANDARD LVCMOS33 } [get_ports ex0[9]]
set_property -dict { PACKAGE_PIN AE3  IOSTANDARD LVCMOS33 } [get_ports ex0[10]]
set_property -dict { PACKAGE_PIN AA8  IOSTANDARD LVCMOS33 } [get_ports ex0[11]]
set_property -dict { PACKAGE_PIN AF2  IOSTANDARD LVCMOS33 } [get_ports ex0[12]]
set_property -dict { PACKAGE_PIN V6   IOSTANDARD LVCMOS33 } [get_ports ex0[13]]
set_property -dict { PACKAGE_PIN AE2  IOSTANDARD LVCMOS33 } [get_ports ex0[14]]
set_property -dict { PACKAGE_PIN AA7  IOSTANDARD LVCMOS33 } [get_ports ex0[15]]
set_property -dict { PACKAGE_PIN AE1  IOSTANDARD LVCMOS33 } [get_ports ex0[16]]
set_property -dict { PACKAGE_PIN W6   IOSTANDARD LVCMOS33 } [get_ports ex0[17]]
set_property -dict { PACKAGE_PIN AD1  IOSTANDARD LVCMOS33 } [get_ports ex0[18]]
set_property -dict { PACKAGE_PIN AC6  IOSTANDARD LVCMOS33 } [get_ports ex0[19]]
set_property -dict { PACKAGE_PIN AC2  IOSTANDARD LVCMOS33 } [get_ports ex0[20]]
set_property -dict { PACKAGE_PIN AB5  IOSTANDARD LVCMOS33 } [get_ports ex0[21]]
set_property -dict { PACKAGE_PIN AC1  IOSTANDARD LVCMOS33 } [get_ports ex0[22]]
set_property -dict { PACKAGE_PIN AC4  IOSTANDARD LVCMOS33 } [get_ports ex0[23]]
set_property -dict { PACKAGE_PIN AB2  IOSTANDARD LVCMOS33 } [get_ports ex0[24]]
set_property -dict { PACKAGE_PIN AD4  IOSTANDARD LVCMOS33 } [get_ports ex0[25]]
set_property -dict { PACKAGE_PIN AB1  IOSTANDARD LVCMOS33 } [get_ports ex0[26]]
set_property -dict { PACKAGE_PIN W1   IOSTANDARD LVCMOS33 } [get_ports ex0[27]]
set_property -dict { PACKAGE_PIN AA2  IOSTANDARD LVCMOS33 } [get_ports ex0[28]]
set_property -dict { PACKAGE_PIN V1   IOSTANDARD LVCMOS33 } [get_ports ex0[29]]
set_property -dict { PACKAGE_PIN V4   IOSTANDARD LVCMOS33 } [get_ports ex0[30]]
set_property -dict { PACKAGE_PIN AD3  IOSTANDARD LVCMOS33 } [get_ports ex0[31]]
set_property -dict { PACKAGE_PIN Y1   IOSTANDARD LVCMOS33 } [get_ports ex0[32]]
set_property -dict { PACKAGE_PIN AC3  IOSTANDARD LVCMOS33 } [get_ports ex0[33]]
set_property -dict { PACKAGE_PIN AB4  IOSTANDARD LVCMOS33 } [get_ports ex0[34]]
set_property -dict { PACKAGE_PIN AA5  IOSTANDARD LVCMOS33 } [get_ports ex0[35]]
set_property -dict { PACKAGE_PIN V3   IOSTANDARD LVCMOS33 } [get_ports ex0[36]]
set_property -dict { PACKAGE_PIN AA4  IOSTANDARD LVCMOS33 } [get_ports ex0[37]]
set_property -dict { PACKAGE_PIN Y2   IOSTANDARD LVCMOS33 } [get_ports ex0[38]]
set_property -dict { PACKAGE_PIN AA3  IOSTANDARD LVCMOS33 } [get_ports ex0[39]]
set_property -dict { PACKAGE_PIN V2   IOSTANDARD LVCMOS33 } [get_ports ex0[40]]
set_property -dict { PACKAGE_PIN Y6   IOSTANDARD LVCMOS33 } [get_ports ex0[41]]
set_property -dict { PACKAGE_PIN Y5   IOSTANDARD LVCMOS33 } [get_ports ex0[42]]
set_property -dict { PACKAGE_PIN Y3   IOSTANDARD LVCMOS33 } [get_ports ex0[43]]
set_property -dict { PACKAGE_PIN AB6  IOSTANDARD LVCMOS33 } [get_ports ex0[44]]
set_property -dict { PACKAGE_PIN W5   IOSTANDARD LVCMOS33 } [get_ports ex0[45]]
set_property -dict { PACKAGE_PIN Y7   IOSTANDARD LVCMOS33 } [get_ports ex0[46]]
set_property -dict { PACKAGE_PIN W3   IOSTANDARD LVCMOS33 } [get_ports ex0[47]]
set_property -dict { PACKAGE_PIN W8   IOSTANDARD LVCMOS33 } [get_ports ex0[48]]
set_property -dict { PACKAGE_PIN W4   IOSTANDARD LVCMOS33 } [get_ports ex0[49]]
set_property -dict { PACKAGE_PIN AC19 IOSTANDARD LVCMOS33 } [get_ports ex0[50]]
set_property -dict { PACKAGE_PIN AD26 IOSTANDARD LVCMOS33 } [get_ports ex0[51]]
set_property -dict { PACKAGE_PIN AC18 IOSTANDARD LVCMOS33 } [get_ports ex0[52]]
set_property -dict { PACKAGE_PIN AD25 IOSTANDARD LVCMOS33 } [get_ports ex0[53]]
set_property -dict { PACKAGE_PIN AB19 IOSTANDARD LVCMOS33 } [get_ports ex0[54]]
set_property -dict { PACKAGE_PIN AB21 IOSTANDARD LVCMOS33 } [get_ports ex0[55]]
set_property -dict { PACKAGE_PIN AB20 IOSTANDARD LVCMOS33 } [get_ports ex0[56]]
set_property -dict { PACKAGE_PIN AF18 IOSTANDARD LVCMOS33 } [get_ports ex0[57]]
set_property -dict { PACKAGE_PIN AD19 IOSTANDARD LVCMOS33 } [get_ports ex0[58]]
set_property -dict { PACKAGE_PIN AD20 IOSTANDARD LVCMOS33 } [get_ports ex0[59]]
set_property -dict { PACKAGE_PIN AA20 IOSTANDARD LVCMOS33 } [get_ports ex0[60]]
set_property -dict { PACKAGE_PIN AA19 IOSTANDARD LVCMOS33 } [get_ports ex0[61]]
set_property -dict { PACKAGE_PIN AE18 IOSTANDARD LVCMOS33 } [get_ports ex0[62]]
set_property -dict { PACKAGE_PIN AE20 IOSTANDARD LVCMOS33 } [get_ports ex0[63]]
set_property -dict { PACKAGE_PIN T17  IOSTANDARD LVCMOS33 } [get_ports ex1[0]]
set_property -dict { PACKAGE_PIN T15  IOSTANDARD LVCMOS33 } [get_ports ex1[1]]
set_property -dict { PACKAGE_PIN T19  IOSTANDARD LVCMOS33 } [get_ports ex1[2]]
set_property -dict { PACKAGE_PIN T18  IOSTANDARD LVCMOS33 } [get_ports ex1[3]]
set_property -dict { PACKAGE_PIN T20  IOSTANDARD LVCMOS33 } [get_ports ex1[4]]
set_property -dict { PACKAGE_PIN V16  IOSTANDARD LVCMOS33 } [get_ports ex1[5]]
set_property -dict { PACKAGE_PIN U19  IOSTANDARD LVCMOS33 } [get_ports ex1[6]]
set_property -dict { PACKAGE_PIN V17  IOSTANDARD LVCMOS33 } [get_ports ex1[7]]
set_property -dict { PACKAGE_PIN U21  IOSTANDARD LVCMOS33 } [get_ports ex1[8]]
set_property -dict { PACKAGE_PIN W18  IOSTANDARD LVCMOS33 } [get_ports ex1[9]]
set_property -dict { PACKAGE_PIN V22  IOSTANDARD LVCMOS33 } [get_ports ex1[10]]
set_property -dict { PACKAGE_PIN Y23  IOSTANDARD LVCMOS33 } [get_ports ex1[11]]
set_property -dict { PACKAGE_PIN W20  IOSTANDARD LVCMOS33 } [get_ports ex1[12]]
set_property -dict { PACKAGE_PIN AA24 IOSTANDARD LVCMOS33 } [get_ports ex1[13]]
set_property -dict { PACKAGE_PIN V14  IOSTANDARD LVCMOS33 } [get_ports ex1[14]]
set_property -dict { PACKAGE_PIN AA23 IOSTANDARD LVCMOS33 } [get_ports ex1[15]]
set_property -dict { PACKAGE_PIN U16  IOSTANDARD LVCMOS33 } [get_ports ex1[16]]
set_property -dict { PACKAGE_PIN AB24 IOSTANDARD LVCMOS33 } [get_ports ex1[17]]
set_property -dict { PACKAGE_PIN U17  IOSTANDARD LVCMOS33 } [get_ports ex1[18]]
set_property -dict { PACKAGE_PIN AC24 IOSTANDARD LVCMOS33 } [get_ports ex1[19]]
set_property -dict { PACKAGE_PIN V18  IOSTANDARD LVCMOS33 } [get_ports ex1[20]]
set_property -dict { PACKAGE_PIN AA22 IOSTANDARD LVCMOS33 } [get_ports ex1[21]]
set_property -dict { PACKAGE_PIN V19  IOSTANDARD LVCMOS33 } [get_ports ex1[22]]
set_property -dict { PACKAGE_PIN U15  IOSTANDARD LVCMOS33 } [get_ports ex1[23]]
set_property -dict { PACKAGE_PIN W19  IOSTANDARD LVCMOS33 } [get_ports ex1[24]]
set_property -dict { PACKAGE_PIN T14  IOSTANDARD LVCMOS33 } [get_ports ex1[25]]
set_property -dict { PACKAGE_PIN U26  IOSTANDARD LVCMOS33 } [get_ports ex1[26]]
set_property -dict { PACKAGE_PIN U25  IOSTANDARD LVCMOS33 } [get_ports ex1[27]]
set_property -dict { PACKAGE_PIN V26  IOSTANDARD LVCMOS33 } [get_ports ex1[28]]
set_property -dict { PACKAGE_PIN W26  IOSTANDARD LVCMOS33 } [get_ports ex1[29]]
set_property -dict { PACKAGE_PIN W25  IOSTANDARD LVCMOS33 } [get_ports ex1[30]]
set_property -dict { PACKAGE_PIN V21  IOSTANDARD LVCMOS33 } [get_ports ex1[31]]
set_property -dict { PACKAGE_PIN Y26  IOSTANDARD LVCMOS33 } [get_ports ex1[32]]
set_property -dict { PACKAGE_PIN W21  IOSTANDARD LVCMOS33 } [get_ports ex1[33]]
set_property -dict { PACKAGE_PIN U22  IOSTANDARD LVCMOS33 } [get_ports ex1[34]]
set_property -dict { PACKAGE_PIN U24  IOSTANDARD LVCMOS33 } [get_ports ex1[35]]
set_property -dict { PACKAGE_PIN Y25  IOSTANDARD LVCMOS33 } [get_ports ex1[36]]
set_property -dict { PACKAGE_PIN V23  IOSTANDARD LVCMOS33 } [get_ports ex1[37]]
set_property -dict { PACKAGE_PIN AA25 IOSTANDARD LVCMOS33 } [get_ports ex1[38]]
set_property -dict { PACKAGE_PIN V24  IOSTANDARD LVCMOS33 } [get_ports ex1[39]]
set_property -dict { PACKAGE_PIN AB26 IOSTANDARD LVCMOS33 } [get_ports ex1[40]]
set_property -dict { PACKAGE_PIN W23  IOSTANDARD LVCMOS33 } [get_ports ex1[41]]
set_property -dict { PACKAGE_PIN AB25 IOSTANDARD LVCMOS33 } [get_ports ex1[42]]
set_property -dict { PACKAGE_PIN W24  IOSTANDARD LVCMOS33 } [get_ports ex1[43]]
set_property -dict { PACKAGE_PIN AC26 IOSTANDARD LVCMOS33 } [get_ports ex1[44]]
set_property -dict { PACKAGE_PIN Y22  IOSTANDARD LVCMOS33 } [get_ports ex1[45]]
set_property -dict { PACKAGE_PIN U14  IOSTANDARD LVCMOS33 } [get_ports ex1[46]]
set_property -dict { PACKAGE_PIN Y21  IOSTANDARD LVCMOS33 } [get_ports ex1[47]]
set_property -dict { PACKAGE_PIN U20  IOSTANDARD LVCMOS33 } [get_ports ex1[48]]
set_property -dict { PACKAGE_PIN Y20  IOSTANDARD LVCMOS33 } [get_ports ex1[49]]
set_property -dict { PACKAGE_PIN AE26 IOSTANDARD LVCMOS33 } [get_ports ex1[50]]
set_property -dict { PACKAGE_PIN AB22 IOSTANDARD LVCMOS33 } [get_ports ex1[51]]
set_property -dict { PACKAGE_PIN AE25 IOSTANDARD LVCMOS33 } [get_ports ex1[52]]
set_property -dict { PACKAGE_PIN AD24 IOSTANDARD LVCMOS33 } [get_ports ex1[53]]
set_property -dict { PACKAGE_PIN AF25 IOSTANDARD LVCMOS33 } [get_ports ex1[54]]
set_property -dict { PACKAGE_PIN AC23 IOSTANDARD LVCMOS33 } [get_ports ex1[55]]
set_property -dict { PACKAGE_PIN AF24 IOSTANDARD LVCMOS33 } [get_ports ex1[56]]
set_property -dict { PACKAGE_PIN AD23 IOSTANDARD LVCMOS33 } [get_ports ex1[57]]
set_property -dict { PACKAGE_PIN AE23 IOSTANDARD LVCMOS33 } [get_ports ex1[58]]
set_property -dict { PACKAGE_PIN AC22 IOSTANDARD LVCMOS33 } [get_ports ex1[59]]
set_property -dict { PACKAGE_PIN AF23 IOSTANDARD LVCMOS33 } [get_ports ex1[60]]
set_property -dict { PACKAGE_PIN AC21 IOSTANDARD LVCMOS33 } [get_ports ex1[61]]
set_property -dict { PACKAGE_PIN AE21 IOSTANDARD LVCMOS33 } [get_ports ex1[62]]
set_property -dict { PACKAGE_PIN AD21 IOSTANDARD LVCMOS33 } [get_ports ex1[63]]

## Voltage and bitstream
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

## Clock-Domain Crossing (CDC) primitives.
# Set DONT_TOUCH on CDC primitives to prevent pin or boundary changes that
# could make it difficult to find and apply timing exceptions to them or
# perform CDC/RDC analysis on them later.
# May be able to be downgraded to KEEP_HIERARCHY, but play it safe for now.
set_property DONT_TOUCH TRUE [get_cells -hier -filter {ORIG_REF_NAME == prim_flop_2sync}]
set_property DONT_TOUCH TRUE [get_cells -hier -filter {ORIG_REF_NAME == prim_fifo_async}]
set_property DONT_TOUCH TRUE [get_cells -hier -filter {ORIG_REF_NAME == prim_fifo_async_simple}]
# Set ASYNC_REG on the flops our flop-based CDC synchronisers to get
# special place&route to reduce the MTBF from metastability, and to prevent
# dangerous optimisations. Note, this appears NOT to include timing exceptions.
# See the ASYNC_REG sections of UG901 or UG912 for details.
set sync_cells [get_cells -hier -filter {ORIG_REF_NAME == prim_flop_2sync}]
set sync_clk_in [get_pins -of $sync_cells -filter {REF_PIN_NAME == clk_i}]
set sync_flops [all_fanout -flat -only_cells -endpoints_only $sync_clk_in]
set_property ASYNC_REG TRUE $sync_flops
