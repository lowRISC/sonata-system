## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

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


#### Timing Assertions Section ####

### Primary clocks ###
create_clock -name mainClk -period 40.0   [get_ports mainClk] ;# 40 ns = 25 MHz
# create_clock -name tck     -period 33.333 [get_ports tck_i]   ;# 33 ns = 30 MHz (FT4232HQ max, too fast for us?)
create_clock -name tck     -period 66.666 [get_ports tck_i]   ;# 66 ns = 15 MHz (next step down from FT4232HQ max)

### Virtual clocks ###
create_clock -name vclk_extusb -period 83.333 ;# 83 ns = 12 MHz (full-speed USB)

### Generated clocks ###
# PLL clocks - name only; period will be derived from RTL parameters.
# All are generated from mainClk.
set clk_sys_source_pin    [get_pins [all_fanin -flat [get_nets clk_sys]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
set clk_usb_source_pin    [get_pins [all_fanin -flat [get_nets clk_usb]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
set clk_hr_source_pin     [get_pins [all_fanin -flat [get_nets clk_hr]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
set clk_hr90p_source_pin  [get_pins [all_fanin -flat [get_nets clk_hr90p]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
set clk_hr3x_source_pin   [get_pins [all_fanin -flat [get_nets clk_hr3x]] \
                           -filter {name =~ u_clkgen/pll/CLKOUT*}]
create_generated_clock -name clk_sys   $clk_sys_source_pin
create_generated_clock -name clk_usb   $clk_usb_source_pin
create_generated_clock -name clk_hr    $clk_hr_source_pin
create_generated_clock -name clk_hr90p $clk_hr90p_source_pin
create_generated_clock -name clk_hr3x  $clk_hr3x_source_pin
# mainClk and internal clocks generated from it (and thus synchronous with it)
set mainClk_and_generated {mainClk clk_sys clk_usb clk_hr clk_hr90p clk_hr3x}
# I/O clocks
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_lcd [get_port lcd_clk] ;# LCD SPI clk
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_ah_spi [get_port ah_tmpio13] ;# Arduino Header SPI clk
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_mb_spi [get_port mb2] ;# mikroBUS SPI clk
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_rpi_spi0 [get_port rph_g11_sclk] ;# R-Pi SPI0 clk
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_rpi_spi1 [get_port rph_g21_sclk] ;# R-Pi SPI1 clk
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_appspi [get_port appspi_clk] ;# Flash SPI clk
create_generated_clock -source $clk_sys_source_pin -divide_by 2 \
                       -name clk_ethmac [get_port ethmac_sclk] ;# Ethernet SPI clk
create_generated_clock -source $clk_hr90p_source_pin -divide_by 1 \
                       -name clk_exthr [get_port hyperram_ckp] ;# HyperRAM clk

## Virtual clocks based on generated clocks.
# Defined here (after generated clocks) to avoid code constant duplication
create_clock -name vclk_sys -period [get_property period [get_clocks clk_sys]]

### Delay for external MMCM/PLL feedback loop ###
# N/A - clkgen_sonata PLL feedback loop is internal

### Clock Uncertainty and Jitter ###
# Primary clock comes from ASE-25.000MHZ-L-C-T crystal oscillator.
# Reference Peak to Peak Jitter is 28 ps (with a note to contact ABRACOM)
# and maximum RMS Jitter is 5 ps.
# Use the worst value for now.
set_input_jitter mainClk 0.028

### Input and output delay constraints prep ###
#
## Make handy clock period variables for use in I/O delay constraints
set mainClk_ns [get_property period [get_clocks mainClk]]
set tck_ns     [get_property period [get_clocks tck]]
# set extusb_ns  [get_property period [get_clocks vclk_extusb]]
set clk_sys_ns [get_property period [get_clocks clk_sys]]
set clk_usb_ns [get_property period [get_clocks clk_usb]]
set clk_hr_ns  [get_property period [get_clocks clk_hr]]
# SPI hosts can run as fast as half the rate of clk_sys
set sclk_ns [expr {2 * [get_property period [get_clocks clk_sys]]}]
#
## Basics
#
# I/O delay constraints tell the tool the earliest and latest point in time
# that a signal is allowed to leave a particular port relative to a clock.
# Setting the delays can be difficult and take multiple iterations.
# Under-constraining (insufficient margin) can result in system instability.
# Over-constraining (excessive margin) can cause the tool to
# waste cell/routing resources and/or waste its time trying to do
# the impossible at the expense of the rest of the design.
#
# Setup (max) timing margin is increased by:
#   - Shifting *output* delay *earlier* (less delay allowed on launch path)
#   - Shifting *input* delay *later* (less delay allowed on capture path)
#
# Setup (max) delay diagram:
#               time -->
#                        launch                capture
#               clock edges |                      |
#                output max  - - - |
#              setup margin         <------>
#                 input max                 | - - -
#
#   ...from the perspective of the input:
#                           ^               ^      ^
#        external max-delay |----------->|  |      |
#                        clock pessimism |->|      |
#       resulting internal max-delay budget |> -- <|
#
#   ...from the perspective of the output:
#                           ^      ^               ^
#                           |      |  |<-----------| external max-delay
#                           |      |<-| clock pessimism
#                           |> -- <| resulting internal max-delay budget
#
# ----------------------> internal path must be no slower than max-delay budget
#
# Hold (min) timing margin is increased by:
#   - Shifting *output* delay *later* (more delay required on launch path)
#   - Shifting *input* delay *earlier* (more delay required on capture path)
#
# Hold (min) delay diagram:
#                time -->
#                                   launch
#                clock edge            |
#                output min             - - |
#               hold margin       <-------->
#                 input min      | - -
#
#   ...from the perspective of the input:
#                                ^     ^
#                                      |->| external min-delay
#        clock delay + pessimism |<-------|
#                                |< - >| resulting internal min-delay target
#
#   ...from the perspective of the output:
#                                      ^     ^
#                external min-delay |<-|
#                                   |------->| clock delay + pessimism
#  resulting internal min-delay target |< - >|
#
# ----------------------> internal path must be no faster than min-delay target
#
#
# Insufficient setup margin can cause instability at high speeds.
# Insufficient hold margin can cause instability at *any* speed!
# Excessive setup margin can waste time on over-optimisation, or be impossible.
# Excessive hold margin can waste resources making paths overly-long.
# Individually moderate setup and hold margins can be be impossible together.
#
#
## Trace delays
#
# Signal delays from PCB traces and attached cables can affect timing
# constraints, but should be negligible for most Sonata interfaces.
#
# Rough estimating method:
#   Raw calculated values:
#     FR-4 PCB signal delay per trace length  = 7.1 ps/mm (approx.)
#     Free-space signal delay per distance    = 3.3 ps/mm
#   Nice rounded numbers:
#     Slowest = 0.010 ns/mm  --> for use in setup (max-delay) constraints
#     Fastest = 0.004 ns/mm  --> for use in hold  (min-delay) constraints
#
# FR-4 PCB signal speed based on:
# https://flexpcb.org/calculate-trace-length-from-time-delay-value-for-high-speed-pcb-design/
#
#
## Coding it up
#
# It can be difficult to figure out how to express I/O constraints.
# Different interfaces require different approaches.
# The following is a quick reference, mostly based on Vivado templates.
# Also see "Constraining Input and Output Ports" in AMD doc UG949
# and "Constraining I/O Delay" in UG903.
#
# Clocked interfaces can be broadly grouped into System or Source synchronous.
# System-Synchronous: independent clock supplied to both ends of interface.
# Source-Synchronous: clock provided/defined by one/both end(s) of interface.
#                     e.g. JTAG, SPI
#
# Interfaces with an external reference clock that is not provided to us
# can be constrained against a virtual clock.
# The period of the virtual clock should be either the same as the internal
# clock or an integer multiple to keep the timing path requirement realistic.
# e.g. USB, UART
#
# Center-Aligned Rising Edge Source Synchronous Inputs
# input    ____           __________
# clock        |_________|          |_____
#                        |
#                 dv_bre | dv_are
#                <------>|<------>
#          __    ________|________    __
# data     __XXXX____Rise_Data____XXXX__
# dv_bre: period data valid before the rising clock edge
# dv_are: period data valid after the rising clock edge
# set_input_delay -clock $input_clock -max [expr {$input_clock_period - $dv_bre}] [get_ports $input_ports];
# set_input_delay -clock $input_clock -min $dv_are
#
# Rising Edge Source Synchronous Outputs
# forwarded         ____                      ___________________
# clock                 |____________________|                   |____________
#                                            |
#                                     tsu    |    thd
#                                <---------->|<--------->
#                                ____________|___________
# data @ destination    XXXXXXXXX________________________XXXXX
# fwclk: forwarded clock (generated using create_generated_clock at output clock port)
# tsu: destination device setup time requirement
# thd: destination device hold time requirement
# trce_dly_max: maximum board trace delay
# trce_dly_min: minimum board trace delay
# set_output_delay -clock $fwclk -max [expr {$trce_dly_max + $tsu}] [get_ports $output_ports];
# set_output_delay -clock $fwclk -min [expr {$trce_dly_min - $thd}] [get_ports $output_ports];
#
# Note that enclosing `expr` expressions in curly braces increases performance.
# See https://www.tcl-lang.org/man/tcl8.6/TclCmd/expr.htm for details.

### Input and output delay constraints proper ###

## User JTAG (marked as USR_JTAG on schematic).
# Synchronous w/tck, source could be on-board or off-board via headers.
# Inputs to be sampled on rising edge. Output to be driven on falling edge.
# Assume inputs valid from only a few nanoseconds before rising clk edge
# (as Greg has observed external signals changing only 3ns ahead of clk edge)
# to just before the falling clock edge (falling-edge launch less some margin).
set_input_delay -clock tck -max [expr {$tck_ns - 3.0              }] [get_ports td_i]
set_input_delay -clock tck -min [expr {          $tck_ns/2.0 - 2.0}] [get_ports td_i]
set_input_delay -clock tck -max [expr {$tck_ns - 3.0              }] [get_ports tms_i]
set_input_delay -clock tck -min [expr {          $tck_ns/2.0 - 2.0}] [get_ports tms_i]
# Require output signal to be valid at other end of the interface (FT4232HQ)
# from 11 ns (FT4232HQ setup requirement) before the rising clk edge
# to 1/4th of a clock period after the rising clk edge for some healthy margin.
# Assume an off-board JTAG driver will account for cable delay or run slower.
# Distance to on-board FTDI chip = ~30 mm  x2(clk here + data back)
set jtag_trce_dly_max [expr { 30 * 2 * 0.010}]
set jtag_trce_dly_min [expr { 30 * 2 * 0.004}]
set_output_delay -clock tck -max [expr {$jtag_trce_dly_max + 11.0       }] [get_ports td_o]
set_output_delay -clock tck -min [expr {$jtag_trce_dly_min - $tck_ns/4.0}] [get_ports td_o]

## USRUSB interface - asynchronous but need to avoid skew between sets of ports
# Use a virtual clock to keep the interface signals from becoming skewed
# relative to each other or from being excessively delayed.
#
# Data inputs are 4x oversampled and can tolerate some inputs being captured
# in the cycle after others are captured, but no more.
# So, need an arrival window no greater than the capture clk period.
# Allow only 80% a clock period for good measure, and split it across
# both setup and hold to avoid the tool adding superfluous delay to the path.
set_input_delay -clock vclk_extusb -max [expr {$clk_usb_ns - $clk_usb_ns*0.7}] [get_ports usrusb_v_p] ;# data in lines (diff), x4 oversamp
set_input_delay -clock vclk_extusb -min [expr {              $clk_usb_ns*0.1}] [get_ports usrusb_v_p]
set_input_delay -clock vclk_extusb -max [expr {$clk_usb_ns - $clk_usb_ns*0.7}] [get_ports usrusb_v_n] ;# data in lines (diff), x4 oversamp
set_input_delay -clock vclk_extusb -min [expr {              $clk_usb_ns*0.1}] [get_ports usrusb_v_n]
set_input_delay -clock vclk_extusb -max [expr {$clk_usb_ns - $clk_usb_ns*0.7}] [get_ports usrusb_rcv] ;# data in lines (single), x4 oversamp
set_input_delay -clock vclk_extusb -min [expr {              $clk_usb_ns*0.1}] [get_ports usrusb_rcv]
# Data outputs are only being captured every 4 launch cycles, but we want to
# avoid massive skew between them.
# So, use zero output delays so they are output within one launch clk period.
set_output_delay -clock vclk_extusb -max 0 [get_ports usrusb_vpo] ;# data out lines, only changing every 4th cycle
set_output_delay -clock vclk_extusb -min 0 [get_ports usrusb_vpo]
set_output_delay -clock vclk_extusb -max 0 [get_ports usrusb_vmo] ;# data out lines, only changing every 4th cycle
set_output_delay -clock vclk_extusb -min 0 [get_ports usrusb_vmo]
set_output_delay -clock vclk_extusb -max 0 [get_ports usrusb_oe]  ;# data out enable, only changing every 4th cycle
set_output_delay -clock vclk_extusb -min 0 [get_ports usrusb_oe]
# Configuration outputs can be allowed to take their time.
# - usrusb_softcn: phy data-line pull-up (connection) enable
# - usrusb_sus: suspend when physical bus inactive
# - usrusb_spd: speed config
# Give them an extra launch clock period to work with by making them multicycle
# paths in the timing exceptions section and zero output delays here.
set usb_conf_names {usrusb_softcn usrusb_sus usrusb_spd}
set_output_delay -clock vclk_extusb -max 0 [get_ports $usb_conf_names]
set_output_delay -clock vclk_extusb -min 0 [get_ports $usb_conf_names]

## PMOD 0 - asynchronous GPIO
# Could be human-scale or processor-scale (bit-bashing) time periods.
# Put a multicycle constraint on them both in the timing exceptions section,
# and zero I/O delays here.
set_input_delay -clock vclk_sys -max 0 [get_ports {pmod0[*]}]
set_input_delay -clock vclk_sys -min 0 [get_ports {pmod0[*]}]
set_output_delay -clock vclk_sys -max 0 [get_ports {pmod0[*]}]
set_output_delay -clock vclk_sys -min 0 [get_ports {pmod0[*]}]
## PMOD 1 - asynchronous GPIO
# Use same methodology as PMOD 0
set_input_delay -clock vclk_sys -max 0 [get_ports {pmod1[*]}]
set_input_delay -clock vclk_sys -min 0 [get_ports {pmod1[*]}]
set_output_delay -clock vclk_sys -max 0 [get_ports {pmod1[*]}]
set_output_delay -clock vclk_sys -min 0 [get_ports {pmod1[*]}]

## LCD display
# ST7735R LCD driver datasheet:
# - Write transactions can be clocked as fast as 15 MHz (66 ns).
# - Data signal has setup & hold requirements of 10 ns each.
# - Data is captured on rising clock edge.
#
# Sonata SPI host can only go as fast as half the system clock.
# Easier to constrain against the Sonata SPI clock speed than arbitrary 15 MHz.
# As we output both clock + data *and do not receive any data back*,
# the clock can take as long as it likes so long as the data is constrained
# against it using the device setup and hold requirements (and some margin).
# Specify multicycle path constraint in the timing exceptions section
# to account for the halving of the clock rate.
set_output_delay -clock clk_sys -max 0 [get_ports lcd_clk]
set_output_delay -clock clk_sys -min 0 [get_ports lcd_clk]
set_output_delay -clock clk_lcd -max [expr { 10 * 1.1}] [get_ports lcd_copi]
set_output_delay -clock clk_lcd -min [expr {-10 * 1.1}] [get_ports lcd_copi]
# Other interface signals change once for every transaction at most,
# but some have high setup requirements.
# Set a multicycle path constraint in timing exceptions section based on
# a reasonable delay between writing to GPIO and starting the transaction.
# Use output delays based on device setup requirements to push the chip select
# output a bit faster than the data/command select, but do not include the
# hold constraint, as this is much more at the mercy of software timings
# than hardware timings (and impossible to achieve in the chip select case).
set_output_delay -clock clk_lcd -max [expr { 45 * 1.1}] [get_ports lcd_cs] ;# chip select
set_output_delay -clock clk_lcd -min 0                  [get_ports lcd_cs]
set_output_delay -clock clk_lcd -max [expr { 10 * 1.1}] [get_ports lcd_dc] ;# data/command select
set_output_delay -clock clk_lcd -min 0                  [get_ports lcd_dc]

## UART 0 - essentially asynchronous, x16 oversampled
# UART RX is 16x oversampled and uses a 2-flop synchroniser.
# UART TX matches the RX baud rate.
# Put a multicycle constraint on them both in the timing exceptions section,
# and zero I/O delays here.
set_input_delay -clock vclk_sys -max 0 [get_ports ser0_rx]
set_input_delay -clock vclk_sys -min 0 [get_ports ser0_rx]
set_output_delay -clock vclk_sys -max 0 [get_ports ser0_tx]
set_output_delay -clock vclk_sys -min 0 [get_ports ser0_tx]
## UART 1
# Use same methodology as UART 0
set_input_delay -clock vclk_sys -max 0 [get_ports ser1_rx]
set_input_delay -clock vclk_sys -min 0 [get_ports ser1_rx]
set_output_delay -clock vclk_sys -max 0 [get_ports ser1_tx]
set_output_delay -clock vclk_sys -min 0 [get_ports ser1_tx]
## UART RS232
# Use same methodology as UART 0
set_input_delay -clock vclk_sys -max 0 [get_ports rs232_rx]
set_input_delay -clock vclk_sys -min 0 [get_ports rs232_rx]
set_output_delay -clock vclk_sys -max 0 [get_ports rs232_tx]
set_output_delay -clock vclk_sys -min 0 [get_ports rs232_tx]

## QWIIC and Arduino Shield I2C
# I2C Fast-mode Plus has a maximum speed of 1 Mbps using a 1 MHz clock
# with an abnormal waveform (narrow high pulses).
# This is slow enough compared to the system clock that we use zero I/O delays
# here and specify multicycle constraints in the timing exceptions section.
set_input_delay -clock vclk_sys -max 0 [get_ports {sda0 scl0}]
set_input_delay -clock vclk_sys -min 0 [get_ports {sda0 scl0}]
set_output_delay -clock vclk_sys -max 0 [get_ports {sda0 scl0}]
set_output_delay -clock vclk_sys -min 0 [get_ports {sda0 scl0}]
## QWIIC-only I2C
# Use same methodology as the other QWIIC I2C
set_input_delay -clock vclk_sys -max 0 [get_ports {sda1 scl1}]
set_input_delay -clock vclk_sys -min 0 [get_ports {sda1 scl1}]
set_output_delay -clock vclk_sys -max 0 [get_ports {sda1 scl1}]
set_output_delay -clock vclk_sys -min 0 [get_ports {sda1 scl1}]

## Arduino Shield
##   SPI
# We do not know what device will be connected to this interface
# nor the limit of how far away it could be from ourselves,
# so use tight constraints on clock output and return data input
# to allow more time for on-board and in-peripheral signal propagation.
#
# Sonata SPI host can only go as fast as half the system clock,
# so constrain the other SPI signals relative to the output SPI clock
# and create multicycle path constraints in the timing exceptions section.
#
# SPI data is launched and sampled on opposite clock edges
# (i.e. sample on rising edge, launch on falling edge).
# The mapping of rise/fall to launch/sample is configurable for this host,
# but for the purposes of these timing constraints we will assume that
# *we* launch and capture data on the SPI *rising* edge and the
# *peripheral* launches and captures on the *falling* edge, to give us the
# required half-cycle within the everything-on-rising-edge framework.
# In other words, we can model the half-cycle relationship between
# launch and capture by adding a half-cycle of I/O delay on data signals.
# For setup (max-delay), we do not know the timing requirements of the device
# so we instead specify the inverse (the delay we allow ourselves to take).
#
# Distance to Arduino SPI pins (D10-13) = ~5-10 mm  x2(clk there + data back)
set ah_spi_trce_dly_min [expr {5 * 2 * 0.004}]
# For the return data signal, expect input from the falling SPI clk edge
# (taking trace delay into account) to as late as possible around the rising
# clk edge (modeled by allocating nearly all the cycle to external delay)
# to allow for a greater trace or device delay on the SCLK->device->CIPO path.
set_input_delay -clock clk_ah_spi -max [expr {$sclk_ns - 14                      }] [get_ports ah_tmpio12] ;# CIPO
set_input_delay -clock clk_ah_spi -min [expr {$sclk_ns/2.0 + $ah_spi_trce_dly_min}] [get_ports ah_tmpio12]
# For the clock, require output from as soon as reasonably possible after rise
# clk edge (modeled by allocating nearly all the cycle to external delay).
set_output_delay -clock clk_sys -max [expr {$clk_sys_ns - 12}] [get_ports ah_tmpio13] ;# SCLK
set_output_delay -clock clk_sys -min 0                         [get_ports ah_tmpio13]
# For the outgoing data signals, require output from shortly after rising
# (launch) SPI clk edge (modeled by allocating most of the cycle to external
# delay) to the falling (capture) clk edge (half-cycle of external delay).
set_output_delay -clock clk_ah_spi -max [expr {$sclk_ns - 6           }] [get_ports ah_tmpio11] ;# COPI
set_output_delay -clock clk_ah_spi -min [expr {           $sclk_ns/2.0}] [get_ports ah_tmpio11]
set_output_delay -clock clk_ah_spi -max [expr {$sclk_ns - 6           }] [get_ports ah_tmpio10] ;# chip select
set_output_delay -clock clk_ah_spi -min [expr {           $sclk_ns/2.0}] [get_ports ah_tmpio10]
##   GPIO
# Use same methodology as PMOD 0
set ah_gpio_names {ah_tmpio16 ah_tmpio9 ah_tmpio8 ah_tmpio7 ah_tmpio6 ah_tmpio5 ah_tmpio4 ah_tmpio3 ah_tmpio2 ah_tmpio1 ah_tmpio0}
set_input_delay -clock vclk_sys -max 0 [get_ports $ah_gpio_names]
set_input_delay -clock vclk_sys -min 0 [get_ports $ah_gpio_names]
set_output_delay -clock vclk_sys -max 0 [get_ports $ah_gpio_names]
set_output_delay -clock vclk_sys -min 0 [get_ports $ah_gpio_names]

## mikroBUS Click
#   SPI
# Use same methodology as Arduino Shield SPI.
# Distance to mikroBUS SPI pins = ~10-20 mm  x2(clk there + data back)
set mb_spi_trce_dly_min [expr {10 * 2 * 0.004}]
set_input_delay -clock clk_mb_spi -max [expr {$sclk_ns - 14                      }] [get_ports mb3] ;# CIPO
set_input_delay -clock clk_mb_spi -min [expr {$sclk_ns/2.0 + $mb_spi_trce_dly_min}] [get_ports mb3]
set_output_delay -clock clk_sys -max [expr {$clk_sys_ns - 12}] [get_ports mb2] ;# SCLK
set_output_delay -clock clk_sys -min 0                         [get_ports mb2]
set_output_delay -clock clk_mb_spi -max [expr {$sclk_ns - 6           }] [get_ports mb4] ;# COPI
set_output_delay -clock clk_mb_spi -min [expr {           $sclk_ns/2.0}] [get_ports mb4]
set_output_delay -clock clk_mb_spi -max [expr {$sclk_ns - 6           }] [get_ports mb1] ;# chip select
set_output_delay -clock clk_mb_spi -min [expr {           $sclk_ns/2.0}] [get_ports mb1]
#   I2C
# Use same methodology as the QWIIC I2C
set_input_delay -clock vclk_sys -max 0 [get_ports {mb5 mb6}] ;# mb5 = SDA
set_input_delay -clock vclk_sys -min 0 [get_ports {mb5 mb6}] ;# mb6 = SCL
set_output_delay -clock vclk_sys -max 0 [get_ports {mb5 mb6}]
set_output_delay -clock vclk_sys -min 0 [get_ports {mb5 mb6}]
#   UART
# Use same methodology as UART 0
set_input_delay -clock vclk_sys -max 0 [get_ports mb8] ;# UART RX
set_input_delay -clock vclk_sys -min 0 [get_ports mb8]
set_output_delay -clock vclk_sys -max 0 [get_ports mb7] ;# UART TX
set_output_delay -clock vclk_sys -min 0 [get_ports mb7]
#   GPI/O
# Use same methodology as PMOD 0
set_input_delay -clock vclk_sys -max 0 [get_ports mb9] ;# Interrupt input (connected to GPI)
set_input_delay -clock vclk_sys -min 0 [get_ports mb9]
set_output_delay -clock vclk_sys -max 0 [get_ports mb0] ;# Reset output (connected to GPO)
set_output_delay -clock vclk_sys -min 0 [get_ports mb0]
#   PWM
# Asynchronous but want to keep timing variation somewhat low to avoid
# adding jitter to the output waveform.
# Go for a single clock cycle with zero output delay.
set_output_delay -clock clk_sys -max 0 [get_ports mb10]
set_output_delay -clock clk_sys -min 0 [get_ports mb10]

## R-Pi Header
##   GPIO/SPI0 bus
# Use same methodology as Arduino Shield SPI.
# Distance to R-Pi SPI0 pins = ~3-10 mm  x2(clk there + data back)
set rph_spi0_trce_dly_min [expr {3 * 2 * 0.004}]
set_input_delay -clock clk_rpi_spi0 -max [expr {$sclk_ns - 14                       }] [get_ports rph_g9_cipo] ;# CIPO
set_input_delay -clock clk_rpi_spi0 -min [expr {$sclk_ns/2.0 + $rph_spi0_trce_dly_min}] [get_ports rph_g9_cipo]
set_output_delay -clock clk_sys -max [expr {$clk_sys_ns - 12}] [get_ports rph_g11_sclk] ;# SCLK
set_output_delay -clock clk_sys -min 0                         [get_ports rph_g11_sclk]
set_output_delay -clock clk_rpi_spi0 -max [expr {$sclk_ns - 6           }] [get_ports rph_g10_copi] ;# COPI
set_output_delay -clock clk_rpi_spi0 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g10_copi]
set_output_delay -clock clk_rpi_spi0 -max [expr {$sclk_ns - 6           }] [get_ports rph_g8_ce0] ;# chip select 0
set_output_delay -clock clk_rpi_spi0 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g8_ce0]
set_output_delay -clock clk_rpi_spi0 -max [expr {$sclk_ns - 6           }] [get_ports rph_g7_ce1] ;# chip select 1
set_output_delay -clock clk_rpi_spi0 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g7_ce1] ;# In schematic v0.8 and below, this pin is called rpg_g8_ce1.
##   GPIO/SPI1 bus
# Use same methodology as Arduino Shield SPI.
# Distance to R-Pi SPI1 pins = ~5-15 mm  x2(clk there + data back)
set rph_spi1_trce_dly_min [expr {5 * 2 * 0.004}]
set_input_delay -clock clk_rpi_spi1 -max [expr {$sclk_ns - 14                       }] [get_ports rph_g19_cipo] ;# CIPO
set_input_delay -clock clk_rpi_spi1 -min [expr {$sclk_ns/2.0 + $rph_spi1_trce_dly_min}] [get_ports rph_g19_cipo]
set_output_delay -clock clk_sys -max [expr {$clk_sys_ns - 12}] [get_ports rph_g21_sclk] ;# SCLK
set_output_delay -clock clk_sys -min 0                         [get_ports rph_g21_sclk]
set_output_delay -clock clk_rpi_spi1 -max [expr {$sclk_ns - 6           }] [get_ports rph_g20_copi] ;# COPI
set_output_delay -clock clk_rpi_spi1 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g20_copi]
set_output_delay -clock clk_rpi_spi1 -max [expr {$sclk_ns - 6           }] [get_ports rph_g18] ;# chip select 0
set_output_delay -clock clk_rpi_spi1 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g18]
set_output_delay -clock clk_rpi_spi1 -max [expr {$sclk_ns - 6           }] [get_ports rph_g17] ;# chip select 1
set_output_delay -clock clk_rpi_spi1 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g17]
set_output_delay -clock clk_rpi_spi1 -max [expr {$sclk_ns - 6           }] [get_ports rph_g16_ce2] ;# chip select 2
set_output_delay -clock clk_rpi_spi1 -min [expr {           $sclk_ns/2.0}] [get_ports rph_g16_ce2]
##   GPIO/I2C bus
# Use same methodology as the QWIIC I2C
set_input_delay -clock vclk_sys -max 0 [get_ports {rph_g2_sda rph_g3_scl}]
set_input_delay -clock vclk_sys -min 0 [get_ports {rph_g2_sda rph_g3_scl}]
set_output_delay -clock vclk_sys -max 0 [get_ports {rph_g2_sda rph_g3_scl}]
set_output_delay -clock vclk_sys -min 0 [get_ports {rph_g2_sda rph_g3_scl}]
##   UART
# Use same methodology as UART 0
set_input_delay -clock vclk_sys -max 0 [get_ports rph_rxd0]
set_input_delay -clock vclk_sys -min 0 [get_ports rph_rxd0]
set_output_delay -clock vclk_sys -max 0 [get_ports rph_txd0]
set_output_delay -clock vclk_sys -min 0 [get_ports rph_txd0]
##   Other GPIO
# Use same methodology as PMOD 0
set rph_gpio_names {rph_g4 rph_g5 rph_g6 rph_g12 rph_g13 rph_g22 rph_g23 rph_g24 rph_g25 rph_g26 rph_g27}
set_input_delay -clock vclk_sys -max 0 [get_ports $rph_gpio_names]
set_input_delay -clock vclk_sys -min 0 [get_ports $rph_gpio_names]
set_output_delay -clock vclk_sys -max 0 [get_ports $rph_gpio_names]
set_output_delay -clock vclk_sys -min 0 [get_ports $rph_gpio_names]
##   ID_SC/SD - I2C bus for HAT ID EEPROM
# Use same methodology as the QWIIC I2C
set_input_delay -clock vclk_sys -max 0 [get_ports {rph_g1 rph_g0}] ;# rph_g1 = ID_SC
set_input_delay -clock vclk_sys -min 0 [get_ports {rph_g1 rph_g0}] ;# rph_g0 = ID_SD
set_output_delay -clock vclk_sys -max 0 [get_ports {rph_g1 rph_g0}]
set_output_delay -clock vclk_sys -min 0 [get_ports {rph_g1 rph_g0}]

## RGB LED
# One-wire interface of the WS2813B.
# Drives gate of a low-side transistor connected to device data input and
# pull-up resistor. Data is output from the first device to the second.
# A 'zero' is encoded by 220 ns ~  380 ns high then 580 ns ~ 1000 ns low wave.
# A 'one'  is encoded by 580 ns ~ 1000 ns high then 580 ns ~ 1000 ns low wave.
# So we have something resembling a 200 ns (580-380) timing variation limit.
# Use a negative setup output delay to capture this requirement,
# with some margin and taking the clock period into account.
set_output_delay -clock vclk_sys -max [expr {(-200 * 0.8) + $clk_sys_ns}] [get_ports rgbled0]
set_output_delay -clock vclk_sys -min 0                                   [get_ports rgbled0]

## SPI Flash
# Winbond W25Q256JV flash memory timing requirements:
# - Max clock frequency: 50 MHz (Read Data instructions)
# - Data outputs spec: 6 ns max-dly /  1.5 ns Hold (falling clk edge)
# - Data inputs req:   2 ns Setup   /  3   ns Hold (rising clk edge)
# - Chip select req:   5 ns Setup   / 10   ns Hold (rising clk edge)
#
# Sonata SPI host can only go as fast as half the system clock,
# so constrain the other SPI signals relative to the output SPI clock
# and create multicycle path constraints in the timing exceptions section.
#
# SPI data is launched and sampled on opposite clock edges
# (i.e. sample on rising edge, launch on falling edge).
# For the purposes of these timing constraints we will assume that
# *we* launch and capture data on the SPI *rising* edge and the
# *peripheral* launches and captures on the *falling* edge, to give us the
# required half-cycle within the everything-on-rising-edge framework.
# In other words, we can model the half-cycle relationship between
# launch and capture by adding a half-cycle of I/O delay on data signals.
#
# Distance to app Flash chip = ~10-20 mm  x2(clk there + data back)
set appspi_trce_dly_max [expr {20 * 2 * 0.010}]
set appspi_trce_dly_min [expr {10 * 2 * 0.004}]
# Expect input from 6 ns (+margin) + max-trace-delay after falling (launch)
# edge of SPI clk to 1.5 ns (+margin) + min-trace-delay after the following
# falling (launch) clk edge.
set_input_delay -clock clk_appspi -max [expr {$sclk_ns/2.0 + $appspi_trce_dly_max + (6   * 1.1)}] [get_ports appspi_d1] ;# CIPO
set_input_delay -clock clk_appspi -min [expr {$sclk_ns/2.0 + $appspi_trce_dly_min + (1.5 * 1.1)}] [get_ports appspi_d1]
# Require clock output as soon as reasonably possible after rising
# SPI clk edge. Do so by allocating nearly all the cycle to external delay.
set_output_delay -clock clk_sys -max [expr {$clk_sys_ns - 12}] [get_ports appspi_clk] ;# SCLK
set_output_delay -clock clk_sys -min 0                         [get_ports appspi_clk]
# Require most outputs from 2 ns (+margin) before falling (pseudo-capture)
# edge to 3 ns (+margin) after falling (pseudo-capture) edge.
set_output_delay -clock clk_appspi -max [expr {$sclk_ns/2.0 + (2 * 1.1)}] [get_ports appspi_d0] ;# COPI
set_output_delay -clock clk_appspi -min [expr {$sclk_ns/2.0 - (3 * 1.1)}] [get_ports appspi_d0]
set_output_delay -clock clk_appspi -max [expr {$sclk_ns/2.0 + (2 * 1.1)}] [get_ports appspi_d2] ;# WP_N
set_output_delay -clock clk_appspi -min [expr {$sclk_ns/2.0 - (3 * 1.1)}] [get_ports appspi_d2]
set_output_delay -clock clk_appspi -max [expr {$sclk_ns/2.0 + (2 * 1.1)}] [get_ports appspi_d3] ;# HOLD_N
set_output_delay -clock clk_appspi -min [expr {$sclk_ns/2.0 - (3 * 1.1)}] [get_ports appspi_d3]
# Require chip select from 5 ns (+margin) before falling (pseudo-capture)
# edge to 10 ns (+margin) after falling (pseudo-capture) edge.
set_output_delay -clock clk_appspi -max [expr {$sclk_ns/2.0 + ( 5 * 1.1)}] [get_ports appspi_cs] ;# CS_N
set_output_delay -clock clk_appspi -min [expr {$sclk_ns/2.0 - (10 * 1.1)}] [get_ports appspi_cs]

## Ethernet MAC
# KSZ8851SNLI datasheet:
# - 40MHz max SPI clk frequency
# - Serial output spec: 9 ns max-dly (falling clk edge)
# - Serial input req: 3 ns Setup / 3 ns Hold (rising clk edge)
# - Chip select req: 8 ns Setup (rise clk edge) / 8 ns Hold (fall clk edge)
#
# Uses the same methodology as the SPI Flash interface.
#
# Distance to Ethernet controller = ~25-35 mm  x2(clk there + data back)
set ethmac_trce_dly_max [expr {35 * 2 * 0.010}]
set ethmac_trce_dly_min [expr {25 * 2 * 0.004}]
# Expect data input from 9 ns (+margin) + max-trace-delay after falling
# (launch) edge to min-trace-delay after the following falling edge.
set_input_delay -clock clk_ethmac -max [expr {$sclk_ns/2.0 + $ethmac_trce_dly_max + (9   * 1.1)}] [get_ports ethmac_cipo]
set_input_delay -clock clk_ethmac -min [expr {$sclk_ns/2.0 + $ethmac_trce_dly_min              }] [get_ports ethmac_cipo]
# Require clock output as soon as reasonably possible after rising
# SPI clk edge. Do so by allocating nearly all the cycle to external delay.
set_output_delay -clock clk_sys -max [expr {$clk_sys_ns - 12}] [get_ports ethmac_sclk]
set_output_delay -clock clk_sys -min 0                         [get_ports ethmac_sclk]
# Require data output from 3 ns (+margin) before falling (pseudo-capture)
# clk edge to 3 ns (+margin) after falling (pseudo-capture) clk edge.
set_output_delay -clock clk_ethmac -max [expr {$sclk_ns/2.0 + (3 * 1.1)}] [get_ports ethmac_copi]
set_output_delay -clock clk_ethmac -min [expr {$sclk_ns/2.0 - (3 * 1.1)}] [get_ports ethmac_copi]
# Require chip select output from 8 ns (+margin) before falling
# (pseudo-capture) clk edge to 8 ns (+margin) after the same edge.
set_output_delay -clock clk_ethmac -max [expr {$sclk_ns/2.0 + (8 * 1.1)}] [get_ports ethmac_cs]
set_output_delay -clock clk_ethmac -min [expr {$sclk_ns/2.0 - (8 * 1.1)}] [get_ports ethmac_cs]

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
# JTAG tck is completely asynchronous to FPGA mainClk and derivatives
set_clock_groups -asynchronous -group tck -group $mainClk_and_generated


#### Timing Exceptions Section ####

### False Paths ###

## Reset port - asynchronous on-board button or external source.
set_false_path -from [get_ports nrst]

## General purpose LEDs - human-scale asynchronous
set_false_path -to [get_ports {usrLed[*]}]
## CHERI error LEDs - human-scale asynchronous
set_false_path -to [get_ports {cheriErr[*]}]
## Status LEDs - human-scale asynchronous
set_false_path -to [get_ports {led_legacy led_cheri led_halted led_bootok}]
## Switch and button input - human-scale asynchronous
set_false_path -from [get_ports {usrSw[*]}]
set_false_path -from [get_ports {navSw[*]}]
set_false_path -from [get_ports {selSw[*]}]

## USRUSB interface - asynchronous vbus detection
set_false_path -from [get_ports usrusb_vbusdetect]

## LCD display
# Reset must be held for 9+ us.
# Backlight signal drives gate of discrete transistor that powers backlights.
set_false_path -to [get_ports {lcd_rst lcd_backlight}] ;# vbus detection - async

## Ethernet MAC - asynchronous reset
set_false_path -to [get_ports ethmac_rst]

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

## prim_flop_2sync
# Explicit false_path not needed so long as ASYNC_REG property is correctly
# set on the underlying flops earlier in the flow.

## prim_fifo_async and prim_fifo_async_simple
# Set false_path timing exceptions on asynchronous fifo outputs.
# Target the outputs because the storage elements are clocked by the source
# clock domain (but made safe to read from the destination clock domain
# thanks to the gray-coded read/write pointers and surrounding logic).
#
# Reliant on the hierarchical pin names used here remaining unchanged during
# synthesis by setting DONT_TOUCH or KEEP_HIERARCHY earlier in the flow.
set async_fifo_cells [get_cells -hier -regexp -filter {ORIG_REF_NAME =~ {prim_fifo_async(_simple)?}}]
set async_fifo_pins [get_pins -filter {REF_PIN_NAME =~ rdata_o*} -of $async_fifo_cells]
set async_fifo_startpoints [all_fanin -startpoints_only -flat $async_fifo_pins]
# Specify `-through` as well as `-from` to avoid including non-rdata_o paths,
# such as paths from the read pointers that stay internal or exit via rvalid_o.
set_false_path -from $async_fifo_startpoints -through $async_fifo_pins

### Max Delay / Min Delay ###

## Reset async-assert sync-deassert CDC - async path to synchroniser reset pins
# Use max_delay rather than false_path to avoid big skew between clock domains.
#
# Reliant on the hierarchical pin names used here remaining unchanged during
# synthesis by setting DONT_TOUCH or KEEP_HIERARCHY earlier in the flow.
set rst_sync_cells [get_cells u_rst_sync/* -filter {ORIG_REF_NAME == prim_flop_2sync}]
set rst_sync_pins [get_pins -filter {REF_PIN_NAME =~ rst_ni} -of $rst_sync_cells]
# Filter out any that do not have a real timing path (fail to find leaf cell).
set rst_sync_endpoints [filter [all_fanout -endpoints_only -flat $rst_sync_pins] IS_LEAF]
set_max_delay $clk_sys_ns -to $rst_sync_endpoints

## Ethernet MAC
# Asynchronous(?) interrupt.
# Use max_delay rather than false_path to keep the delay within reason.
set_max_delay 30 -datapath_only -from [get_ports ethmac_intr]

### Multicycle Paths ###

## USRUSB interface - pseudo-static config
# USB config signals are in no hurry, changing either hundreds of cycles
# ahead of any traffic or not changing at all.
# Allow them to be output as late as several USB bit-periods after the
# launching clock edge (setup), or as early as right after the launching
# clock edge (have to relax hold back following setup change).
set usb_conf_extusb_mulcycs 4
set_multicycle_path        $usb_conf_extusb_mulcycs       -setup -end -to [get_ports $usb_conf_names]
set_multicycle_path [expr {$usb_conf_extusb_mulcycs - 1}] -hold  -end -to [get_ports $usb_conf_names]

## SPI interfaces
## LCD display
# Data is clocked by SPI host no faster than half the rate of the system clk.
set lcd_fast_mulcycs 2
set_multicycle_path        $lcd_fast_mulcycs       -setup -start -to [get_ports lcd_copi]
set_multicycle_path [expr {$lcd_fast_mulcycs - 1}] -hold  -start -to [get_ports lcd_copi]
# Chip select and data/command select are output by a GPIO block
# once every transaction, so give them a couple of (destination clk) cycles
# to account for the delay between writing to GPIO and starting SPI output.
set lcd_slow_mulcycs 2
set_multicycle_path        $lcd_slow_mulcycs       -setup -end -to [get_ports {lcd_cs lcd_dc}]
set_multicycle_path [expr {$lcd_slow_mulcycs - 1}] -hold  -end -to [get_ports {lcd_cs lcd_dc}]
## App Flash
# Data in and out driven/captured by SPI host block.
set appspi_mulcycs 2
set_multicycle_path        $appspi_mulcycs       -setup -start -to [get_ports {appspi_d0 appspi_d2 appspi_d3 appspi_cs}] ;# out
set_multicycle_path [expr {$appspi_mulcycs - 1}] -hold  -start -to [get_ports {appspi_d0 appspi_d2 appspi_d3 appspi_cs}] ;# out
set_multicycle_path        $appspi_mulcycs       -setup -end -from [get_ports appspi_d1] ;# in
set_multicycle_path [expr {$appspi_mulcycs - 1}] -hold  -end -from [get_ports appspi_d1] ;# in
## Ethernet MAC
# Data in and out driven/captured by SPI host block.
set ethmac_mulcycs 2
set_multicycle_path        $ethmac_mulcycs       -setup -start -to [get_ports {ethmac_copi ethmac_cs}] ;# out
set_multicycle_path [expr {$ethmac_mulcycs - 1}] -hold  -start -to [get_ports {ethmac_copi ethmac_cs}] ;# out
set_multicycle_path        $ethmac_mulcycs       -setup -end -from [get_ports ethmac_cipo] ;# in
set_multicycle_path [expr {$ethmac_mulcycs - 1}] -hold  -end -from [get_ports ethmac_cipo] ;# in
## Arduino Shield SPI
# Data is clocked by SPI host no faster than half the rate of the system clk.
set ah_spi_mulcycs 2
set_multicycle_path        $ah_spi_mulcycs       -setup -start -to [get_ports {ah_tmpio11 ah_tmpio10}] ;# out
set_multicycle_path [expr {$ah_spi_mulcycs - 1}] -hold  -start -to [get_ports {ah_tmpio11 ah_tmpio10}] ;# out
set_multicycle_path        $ah_spi_mulcycs       -setup -end -from [get_ports ah_tmpio12] ;# in
set_multicycle_path [expr {$ah_spi_mulcycs - 1}] -hold  -end -from [get_ports ah_tmpio12] ;# in
## mikroBUS SPI - use same methodology as Arduino SPI
set mb_spi_mulcycs $ah_spi_mulcycs
set_multicycle_path        $mb_spi_mulcycs       -setup -start -to [get_ports {mb1 mb4}]
set_multicycle_path [expr {$mb_spi_mulcycs - 1}] -hold  -start -to [get_ports {mb1 mb4}]
set_multicycle_path        $mb_spi_mulcycs       -setup -end -from [get_ports mb3]
set_multicycle_path [expr {$mb_spi_mulcycs - 1}] -hold  -end -from [get_ports mb3]
## R-Pi SPI0 - use same methodology as Arduino SPI
set rph_spi0_mulcycs $ah_spi_mulcycs
set_multicycle_path        $rph_spi0_mulcycs       -setup -start -to [get_ports {rph_g10_copi rph_g8_ce0 rph_g7_ce1}]
set_multicycle_path [expr {$rph_spi0_mulcycs - 1}] -hold  -start -to [get_ports {rph_g10_copi rph_g8_ce0 rph_g7_ce1}]
set_multicycle_path        $rph_spi0_mulcycs       -setup -end -from [get_ports rph_g9_cipo]
set_multicycle_path [expr {$rph_spi0_mulcycs - 1}] -hold  -end -from [get_ports rph_g9_cipo]
## R-Pi SPI1 - use same methodology as Arduino SPI
set rph_spi1_mulcycs $ah_spi_mulcycs
set_multicycle_path        $rph_spi1_mulcycs       -setup -start -to [get_ports {rph_g20_copi rph_g18 rph_g17 rph_g16_ce2}]
set_multicycle_path [expr {$rph_spi1_mulcycs - 1}] -hold  -start -to [get_ports {rph_g20_copi rph_g18 rph_g17 rph_g16_ce2}]
set_multicycle_path        $rph_spi1_mulcycs       -setup -end -from [get_ports rph_g19_cipo]
set_multicycle_path [expr {$rph_spi1_mulcycs - 1}] -hold  -end -from [get_ports rph_g19_cipo]

## UART interfaces
## UART 0 - essentially asynchronous, x16 oversampled
# UART RX and TX are run 16x faster than the baud rate.
# Allow them to be captured or output over the course of half the a bit-period.
# (Could maybe allow full bit-period, but prefer to keep timing less variable)
set ser0_mulcycs 8
set_multicycle_path        $ser0_mulcycs       -setup -from [get_ports ser0_rx]
set_multicycle_path [expr {$ser0_mulcycs - 1}] -hold  -from [get_ports ser0_rx]
set_multicycle_path        $ser0_mulcycs       -setup -to [get_ports ser0_tx]
set_multicycle_path [expr {$ser0_mulcycs - 1}] -hold  -to [get_ports ser0_tx]
## UART 1 - use same methodology as UART 0
set ser1_mulcycs $ser0_mulcycs
set_multicycle_path        $ser1_mulcycs       -setup -from [get_ports ser1_rx]
set_multicycle_path [expr {$ser1_mulcycs - 1}] -hold  -from [get_ports ser1_rx]
set_multicycle_path        $ser1_mulcycs       -setup -to [get_ports ser1_tx]
set_multicycle_path [expr {$ser1_mulcycs - 1}] -hold  -to [get_ports ser1_tx]
## UART RS232 - use same methodology as UART 0
set rs232_mulcycs $ser0_mulcycs
set_multicycle_path        $rs232_mulcycs       -setup -from [get_ports rs232_rx]
set_multicycle_path [expr {$rs232_mulcycs - 1}] -hold  -from [get_ports rs232_rx]
set_multicycle_path        $rs232_mulcycs       -setup -to [get_ports rs232_tx]
set_multicycle_path [expr {$rs232_mulcycs - 1}] -hold  -to [get_ports rs232_tx]
## mikroBUS UART - use same methodology as UART 0
set mb_ser_mulcycs $ser0_mulcycs
set_multicycle_path        $mb_ser_mulcycs       -setup -from [get_ports mb8]
set_multicycle_path [expr {$mb_ser_mulcycs - 1}] -hold  -from [get_ports mb8]
set_multicycle_path        $mb_ser_mulcycs       -setup -to [get_ports mb7]
set_multicycle_path [expr {$mb_ser_mulcycs - 1}] -hold  -to [get_ports mb7]
## R-Pi Header UART - use same methodology as UART 0
set rph_ser_mulcycs $ser0_mulcycs
set_multicycle_path        $rph_ser_mulcycs       -setup -from [get_ports rph_rxd0]
set_multicycle_path [expr {$rph_ser_mulcycs - 1}] -hold  -from [get_ports rph_rxd0]
set_multicycle_path        $rph_ser_mulcycs       -setup -to [get_ports rph_txd0]
set_multicycle_path [expr {$rph_ser_mulcycs - 1}] -hold  -to [get_ports rph_txd0]

## I2C interfaces
## QWIIC and Arduino Shield I2C
# I2C Fast-mode Plus has a maximum speed of 1 Mbps using a 1 MHz clock
# with an abnormal waveform (narrow high pulses).
# Some timings of clock and data pulses are run-time programmable
# in units of system clock cycles.
# Balance the desire to provide repeatable user-programable timing
# against the desire to relax physical timing to help wider design QoR.
set qwiic_ah_i2c_mulcycs 2
set_multicycle_path        $qwiic_ah_i2c_mulcycs       -setup -from [get_ports {sda0 scl0}]
set_multicycle_path [expr {$qwiic_ah_i2c_mulcycs - 1}] -hold  -from [get_ports {sda0 scl0}]
set_multicycle_path        $qwiic_ah_i2c_mulcycs       -setup -to [get_ports {sda0 scl0}]
set_multicycle_path [expr {$qwiic_ah_i2c_mulcycs - 1}] -hold  -to [get_ports {sda0 scl0}]
## QWIIC-only I2C - Use same methodology as the other QWIIC I2C
set qwiic_only_i2c_mulcycs $qwiic_ah_i2c_mulcycs
set_multicycle_path        $qwiic_only_i2c_mulcycs       -setup -from [get_ports {sda1 scl1}]
set_multicycle_path [expr {$qwiic_only_i2c_mulcycs - 1}] -hold  -from [get_ports {sda1 scl1}]
set_multicycle_path        $qwiic_only_i2c_mulcycs       -setup -to [get_ports {sda1 scl1}]
set_multicycle_path [expr {$qwiic_only_i2c_mulcycs - 1}] -hold  -to [get_ports {sda1 scl1}]
## mikroBUS I2C - use same methodology as the QWIIC I2C
set mb_i2c_mulcycs $qwiic_ah_i2c_mulcycs
set_multicycle_path        $mb_i2c_mulcycs       -setup -from [get_ports {mb5 mb6}]
set_multicycle_path [expr {$mb_i2c_mulcycs - 1}] -hold  -from [get_ports {mb5 mb6}]
set_multicycle_path        $mb_i2c_mulcycs       -setup -to [get_ports {mb5 mb6}]
set_multicycle_path [expr {$mb_i2c_mulcycs - 1}] -hold  -to [get_ports {mb5 mb6}]
## R-Pi Header I2C/GPIO - use same methodology as the QWIIC I2C
set rph_i2c_mulcycs $qwiic_ah_i2c_mulcycs
set_multicycle_path        $rph_i2c_mulcycs       -setup -from [get_ports {rph_g2_sda rph_g3_scl}]
set_multicycle_path [expr {$rph_i2c_mulcycs - 1}] -hold  -from [get_ports {rph_g2_sda rph_g3_scl}]
set_multicycle_path        $rph_i2c_mulcycs       -setup -to [get_ports {rph_g2_sda rph_g3_scl}]
set_multicycle_path [expr {$rph_i2c_mulcycs - 1}] -hold  -to [get_ports {rph_g2_sda rph_g3_scl}]
## R-Pi Hat ID EEPROM I2C - use same methodology as the QWIIC I2C
set rph_id_mulcycs $qwiic_ah_i2c_mulcycs
set_multicycle_path        $rph_id_mulcycs       -setup -from [get_ports {rph_g1 rph_g0}]
set_multicycle_path [expr {$rph_id_mulcycs - 1}] -hold  -from [get_ports {rph_g1 rph_g0}]
set_multicycle_path        $rph_id_mulcycs       -setup -to [get_ports {rph_g1 rph_g0}]
set_multicycle_path [expr {$rph_id_mulcycs - 1}] -hold  -to [get_ports {rph_g1 rph_g0}]

## PMOD 0 - asynchronous GPIO
# Could be human-scale or processor-scale (bit-bashing) time periods.
# May be possible for some TLUL host to read or write at full system clock
# speed, but unlikely to be reliable.
# Allow two cycles so that known high-speed ports get place & route priority.
set pmod0_mulcycs 2
set_multicycle_path        $pmod0_mulcycs       -setup -from [get_ports {pmod0[*]}]
set_multicycle_path [expr {$pmod0_mulcycs - 1}] -hold  -from [get_ports {pmod0[*]}]
set_multicycle_path        $pmod0_mulcycs       -setup -to [get_ports {pmod0[*]}]
set_multicycle_path [expr {$pmod0_mulcycs - 1}] -hold  -to [get_ports {pmod0[*]}]
## PMOD 1 - use same methodology as PMOD 0
set pmod1_mulcycs $pmod0_mulcycs
set_multicycle_path        $pmod1_mulcycs       -setup -from [get_ports {pmod1[*]}]
set_multicycle_path [expr {$pmod1_mulcycs - 1}] -hold  -from [get_ports {pmod1[*]}]
set_multicycle_path        $pmod1_mulcycs       -setup -to [get_ports {pmod1[*]}]
set_multicycle_path [expr {$pmod1_mulcycs - 1}] -hold  -to [get_ports {pmod1[*]}]
## mikroBUS GPIO - use same methodology as PMOD 0
set mb_gpio_mulcycs $pmod0_mulcycs
set_multicycle_path        $mb_gpio_mulcycs       -setup -from [get_ports mb9] ;# Interrupt input
set_multicycle_path [expr {$mb_gpio_mulcycs - 1}] -hold  -from [get_ports mb9]
set_multicycle_path        $mb_gpio_mulcycs       -setup -to [get_ports mb0] ;# Reset output
set_multicycle_path [expr {$mb_gpio_mulcycs - 1}] -hold  -to [get_ports mb0]
## R-Pi Header GPIO - use same methodology as PMOD 0
set rph_gpio_mulcycs $pmod0_mulcycs
set_multicycle_path        $rph_gpio_mulcycs       -setup -from [get_ports $rph_gpio_names]
set_multicycle_path [expr {$rph_gpio_mulcycs - 1}] -hold  -from [get_ports $rph_gpio_names]
set_multicycle_path        $rph_gpio_mulcycs       -setup -to [get_ports $rph_gpio_names]
set_multicycle_path [expr {$rph_gpio_mulcycs - 1}] -hold  -to [get_ports $rph_gpio_names]
## Arduino Shield GPIO - use same methodology as PMOD 0
set ah_gpio_mulcycs $pmod0_mulcycs
set_multicycle_path        $ah_gpio_mulcycs       -setup -from [get_ports $ah_gpio_names]
set_multicycle_path [expr {$ah_gpio_mulcycs - 1}] -hold  -from [get_ports $ah_gpio_names]
set_multicycle_path        $ah_gpio_mulcycs       -setup -to [get_ports $ah_gpio_names]
set_multicycle_path [expr {$ah_gpio_mulcycs - 1}] -hold  -to [get_ports $ah_gpio_names]

### Case Analysis ###
# none at present

### Disable Timing ###
# none at present
