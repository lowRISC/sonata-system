## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

# Sonata One-specific, to be sourced after synth_timing_common.xdc

# This file is for timing constraints to be applied *before* synthesis.
# i.e. timing constraints on top-level ports.
#
# See UG949 and UG903 for information on setting various timing constraints.

#### Recommended timing constraints sequence from UG949 ####
## Timing Assertions Section
# Primary clocks
# Virtual clocks
# Generated clocks
# Delay for external MMCM/PLL feedback loop
# Clock Uncertainty and Jitter
# Input and output delay constraints
# Clock Groups and Clock False Paths
## Timing Exceptions Section
# False Paths
# Max Delay / Min Delay
# Multicycle Paths
# Case Analysis
# Disable Timing

### Generated clocks ###
# PLL clocks - name only; period will be derived from RTL parameters.
# All are generated from mainClk.
set clk_hr_source_pin     [get_pins [all_fanin -flat [get_nets clk_hr]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
set clk_hr90p_source_pin  [get_pins [all_fanin -flat [get_nets clk_hr90p]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
set clk_hr3x_source_pin   [get_pins [all_fanin -flat [get_nets clk_hr3x]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
create_generated_clock -name clk_hr    $clk_hr_source_pin
create_generated_clock -name clk_hr90p $clk_hr90p_source_pin
create_generated_clock -name clk_hr3x  $clk_hr3x_source_pin
# I/O clocks
create_generated_clock -source $clk_hr90p_source_pin -divide_by 1 \
                       -name clk_exthr [get_port hyperram_ckp] ;# HyperRAM clk

### Input and output delay constraints ###
## HyperRAM
# W956D8MBYA(5I variant) datasheet:
# - 200 MHz max clock frequency
# - 400 MT/s max with Double-Data Rate (DDR)
# - Key timing values by operating frequency:
#
# |                                   | 200 MHz | 166 MHz | 133 MHz | 100 MHz |
# |-----------------------------------|---------|---------|---------|---------|
# | CS input setup (rise CK)          | 4.0  ns | 3    ns | 3    ns | 3    ns |
# | CS input hold (fall CK)           | 0    ns | 0    ns | 0    ns | 0    ns |
# | DQ/RWDS input setup (rise/fall CK)| 0.5  ns | 0.6  ns | 0.8  ns | 1.0  ns |
# | DQ/RWDS input hold (rise/fall CK) | 0.5  ns | 0.6  ns | 0.8  ns | 1.0  ns |
# | DQ output setup (rise/fall CK)    | 5.0  ns | 5.5  ns | 5.5  ns | 5.5  ns |
# | DQ output hold (rise/fall CK)     | 0    ns | 0    ns | 0    ns | 0    ns |
# | DQ output min-period valid     *1 | 1.45 ns | 1.8  ns | 2.37 ns | 3.3  ns |
# | RWDS output setup (rise/fall CK)  | 5.0  ns | 5.5  ns | 5.5  ns | 5.5  ns |
# | RWDS out-edge to DQ valid     +/- | 0.4  ns | 0.45 ns | 0.6  ns | 0.8  ns |
# | RWDS out-edge to DQ invalid   +/- | 0.4  ns | 0.45 ns | 0.6  ns | 0.8  ns |
#
# *1: Data Valid minimum period = the lesser of:
#   (CK half-period min - output max-dly to valid + output max-dly to valid) or
#   (CK half-period min - output min-dly to valid + output min-dly to valid)
#
# Currently using set_false_path on HyperRAM signals in exceptions section.
# Specify zero-value I/O delays here in order to associate each port
# with a clock for the purpose of CDC checking.
#
# TODO: add 'real' (non-zero) constraints below and remove set_false_path's
#       so we know if something has not been instantiated/inferred correctly.
set_output_delay -clock clk_hr90p -max 0 [get_ports hyperram_ckp]
set_output_delay -clock clk_hr90p -min 0 [get_ports hyperram_ckp]
#
set_output_delay -clock clk_exthr -max 0 [get_ports hyperram_ckn]
set_output_delay -clock clk_exthr -min 0 [get_ports hyperram_ckn]
#
set_input_delay -clock clk_exthr -max 0 [get_ports {hyperram_dq[*]}]
set_input_delay -clock clk_exthr -min 0 [get_ports {hyperram_dq[*]}]
#
set_output_delay -clock clk_exthr -max 0 [get_ports {hyperram_dq[*]}]
set_output_delay -clock clk_exthr -min 0 [get_ports {hyperram_dq[*]}]
#
set_input_delay -clock clk_exthr -max 0 [get_ports hyperram_rwds]
set_input_delay -clock clk_exthr -min 0 [get_ports hyperram_rwds]
#
set_output_delay -clock clk_exthr -max 0 [get_ports hyperram_rwds]
set_output_delay -clock clk_exthr -min 0 [get_ports hyperram_rwds]
#
set_output_delay -clock clk_exthr -max 0 [get_ports hyperram_cs]
set_output_delay -clock clk_exthr -min 0 [get_ports hyperram_cs]
#
set_output_delay -clock clk_exthr -max 0 [get_ports hyperram_nrst]
set_output_delay -clock clk_exthr -min 0 [get_ports hyperram_nrst]

### Clock Groups and Clock False Paths ###
# JTAG tck is completely asynchronous to FPGA mainClk and
# internal clocks generated from it (and thus synchronous with it).
set_clock_groups -asynchronous -group tck -group {mainClk clk_sys clk_usb clk_hr clk_hr90p clk_hr3x}


#### Timing Exceptions Section ####

### False Paths ###
## HyperRAM
# Constraints (and RTL) are adapted from OpenHBMC:
# https://github.com/OVGN/OpenHBMC/blob/master/OpenHBMC/constrs/OpenHBMC.xdc
# The RTL instantiates I/O primitives itself and places flop in IOBs,
# reducing the need for I/O timing constraints.
#
# TODO: replace these with 'real' constraints so we know if something has
#       not been instantiated/inferred correctly.
#
# Set output false path, timings are met by design
set_false_path -to [get_ports hyperram_ckp]
set_false_path -to [get_ports hyperram_ckn]
set_false_path -to [get_ports hyperram_rwds]
set_false_path -to [get_ports {hyperram_dq[*]}]
# set input false path. dq[*] and rwds are supposed to
# be fully asynchronous for the data recovery logic
set_false_path -from [get_ports hyperram_rwds]
set_false_path -from [get_ports {hyperram_dq[*]}]
# False path for 'hb_cs_n' and 'hb_reset_n'
set_false_path -to [get_ports hyperram_cs]
set_false_path -to [get_ports hyperram_nrst]
