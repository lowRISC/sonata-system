## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

# Using the names in the PCB design, they should match this file with a case-insensitive search:
# https://github.com/newaetech/sonata-pcb/tree/main

## Clocks
create_clock -period 40.000 -name mainClk -waveform {0.000 20.000} [get_ports mainClk]

set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports mainClk];

## Reset
## PCB revision 0.3 and above
set_property -dict { PACKAGE_PIN T5 IOSTANDARD LVCMOS33 } [get_ports {nrst}];
## PCB revision 0.2 and below
#set_property -dict { PACKAGE_PIN R11 IOSTANDARD LVCMOS33 } [get_ports {nrst}];

## General purpose LEDs
set_property -dict { PACKAGE_PIN B13 IOSTANDARD LVCMOS33 } [get_ports {usrLed[0]}];
set_property -dict { PACKAGE_PIN B14 IOSTANDARD LVCMOS33 } [get_ports {usrLed[1]}];
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } [get_ports {usrLed[2]}];
set_property -dict { PACKAGE_PIN B12 IOSTANDARD LVCMOS33 } [get_ports {usrLed[3]}];
set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports {usrLed[4]}];
set_property -dict { PACKAGE_PIN A11 IOSTANDARD LVCMOS33 } [get_ports {usrLed[5]}];
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports {usrLed[6]}];
set_property -dict { PACKAGE_PIN F14 IOSTANDARD LVCMOS33 } [get_ports {usrLed[7]}];

set_output_delay -clock mainClk 0.000 [get_ports usrLed]

## Switch and button input
set_property -dict { PACKAGE_PIN D12 IOSTANDARD LVCMOS33 } [get_ports {usrSw[0]}];
set_property -dict { PACKAGE_PIN D13 IOSTANDARD LVCMOS33 } [get_ports {usrSw[1]}];
set_property -dict { PACKAGE_PIN B16 IOSTANDARD LVCMOS33 } [get_ports {usrSw[2]}];
set_property -dict { PACKAGE_PIN B17 IOSTANDARD LVCMOS33 } [get_ports {usrSw[3]}];
set_property -dict { PACKAGE_PIN A15 IOSTANDARD LVCMOS33 } [get_ports {usrSw[4]}];
set_property -dict { PACKAGE_PIN A16 IOSTANDARD LVCMOS33 } [get_ports {usrSw[5]}];
set_property -dict { PACKAGE_PIN A13 IOSTANDARD LVCMOS33 } [get_ports {usrSw[6]}];
set_property -dict { PACKAGE_PIN A14 IOSTANDARD LVCMOS33 } [get_ports {usrSw[7]}];
set_property -dict { PACKAGE_PIN F5  IOSTANDARD LVCMOS18 } [get_ports {navSw[0]}];
set_property -dict { PACKAGE_PIN D8  IOSTANDARD LVCMOS18 } [get_ports {navSw[1]}];
set_property -dict { PACKAGE_PIN C7  IOSTANDARD LVCMOS18 } [get_ports {navSw[2]}];
set_property -dict { PACKAGE_PIN E7  IOSTANDARD LVCMOS18 } [get_ports {navSw[3]}];
set_property -dict { PACKAGE_PIN D7  IOSTANDARD LVCMOS18 } [get_ports {navSw[4]}];

## CHERI error LEDs
set_property -dict { PACKAGE_PIN K6  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[0]}];
set_property -dict { PACKAGE_PIN L1  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[1]}];
set_property -dict { PACKAGE_PIN M1  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[2]}];
set_property -dict { PACKAGE_PIN K3  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[3]}];
set_property -dict { PACKAGE_PIN L3  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[4]}];
set_property -dict { PACKAGE_PIN N2  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[5]}];
set_property -dict { PACKAGE_PIN N1  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[6]}];
set_property -dict { PACKAGE_PIN M3  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[7]}];
set_property -dict { PACKAGE_PIN M2  IOSTANDARD LVCMOS33 } [get_ports {cheriErr[8]}];

## Status LEDs
set_property -dict { PACKAGE_PIN K5  IOSTANDARD LVCMOS33 } [get_ports led_legacy];
set_property -dict { PACKAGE_PIN L4  IOSTANDARD LVCMOS33 } [get_ports led_cheri];
set_property -dict { PACKAGE_PIN L6  IOSTANDARD LVCMOS33 } [get_ports led_halted];
set_property -dict { PACKAGE_PIN L5  IOSTANDARD LVCMOS33 } [get_ports led_bootok];

## LCD display
set_property -dict { PACKAGE_PIN R6  IOSTANDARD LVCMOS33 } [get_ports lcd_rst];
set_property -dict { PACKAGE_PIN U4  IOSTANDARD LVCMOS33 } [get_ports lcd_dc];
set_property -dict { PACKAGE_PIN R3  IOSTANDARD LVCMOS33 } [get_ports lcd_copi];
set_property -dict { PACKAGE_PIN R5  IOSTANDARD LVCMOS33 } [get_ports lcd_clk];
set_property -dict { PACKAGE_PIN P5  IOSTANDARD LVCMOS33 } [get_ports lcd_cs];
set_property -dict { PACKAGE_PIN N5  IOSTANDARD LVCMOS33 } [get_ports lcd_backlight];

## UART
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports ser0_tx];
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports ser0_rx];

## Switches
set_property PULLTYPE PULLUP [get_ports usrSw[*]]
set_property PULLTYPE PULLUP [get_ports navSw[*]]

## Voltage
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
