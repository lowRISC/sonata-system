// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <cassert>
#include <fstream>
#include <iostream>

#include "Vtop_verilator__Syms.h"
#include "ibex_pcounts.h"
#include "sonata_system.hh"
#include "verilated_toplevel.h"
#include "verilator_memutil.h"
#include "verilator_sim_ctrl.h"

SonataSystem::SonataSystem(const char *ram_hier_path, int ram_size_words,
  const char *hyperram_hier_path, int hyperram_size_words)
    : _ram(ram_hier_path, ram_size_words, 4),
      _hyperram(hyperram_hier_path, hyperram_size_words, 4) {}

int SonataSystem::Main(int argc, char **argv) {
  bool exit_app;
  int ret_code = Setup(argc, argv, exit_app);

  if (exit_app) {
    return ret_code;
  }

  Run();

  if (!Finish()) {
    return 1;
  }

  return 0;
}

int SonataSystem::Setup(int argc, char **argv, bool &exit_app) {
  VerilatorSimCtrl &simctrl = VerilatorSimCtrl::GetInstance();

  // Just a single reset signal for all clock domains.
  simctrl.SetTop(&_top, &_top.rst_ni,
                 VerilatorSimCtrlFlags::ResetPolarityNegative);

  _memutil.RegisterMemoryArea("ram", 0x100000, &_ram);
  _memutil.RegisterMemoryArea("hyperram", 0x40000000, &_hyperram);
  simctrl.RegisterExtension(&_memutil);

  // Create our clocks with their default properties.
  //
  // 40MHz System Clock, no jitter.
  // 48MHz USB Clock, no jitter.
  // 100MHz HyperRAM clocks, no jitter.
  // 300MHz HyperRAM clock, no jitter.
  //
  // All times are in picoseconds for greater accuracy.
  const uint32_t nano = 1000u;  // in ps.
  const uint32_t micro = 1000u * nano;
  const uint32_t milli = 1000u * micro;
  const uint32_t sys_hperiod = micro / 80u;  // 40MHz cycle

  // Main system clock must be added first.
  VerilatorSimClock clk_sys(&_top.clk_i, sys_hperiod, sys_hperiod);
  simctrl.AddClock(clk_sys);

  // The design may run on a single clock for faster simulations.
#ifdef USE_SEPARATED_CLOCKS
  uint32_t usb_hperiod  = micro / 96u;   // 48MHz cycle
  // Note: calculate the period of the higher frequency clock first because
  // the period of the 'hr' reference clock must be exactly 3 times longer
  // to maintain the phase relationship.
  uint32_t hr3x_hperiod = (micro + 599u) / 600u;  // 300MHz cycle
  uint32_t hr_hperiod = 3 * hr3x_hperiod;  // 100MHz cycle

  // The HyperRAM requires a clock that is phase-shifted by 90 degress.
  uint32_t hr90p_offset = hr_hperiod / 2;

  // Supplementary clocks.
  VerilatorSimClock clk_usb(&_top.clk_usb_i, usb_hperiod, usb_hperiod);
  VerilatorSimClock clk_hr(&_top.clk_hr_i, hr_hperiod, hr_hperiod);
  VerilatorSimClock clk_hr90p(&_top.clk_hr90p_i, hr_hperiod, hr_hperiod, hr90p_offset);
  VerilatorSimClock clk_hr3x(&_top.clk_hr3x_i, hr3x_hperiod, hr3x_hperiod);

  simctrl.AddClock(clk_usb);
  simctrl.AddClock(clk_hr);
  simctrl.AddClock(clk_hr90p);
  simctrl.AddClock(clk_hr3x);
#endif

  exit_app = false;
  return simctrl.ParseCommandArgs(argc, argv, exit_app);
}

void SonataSystem::Run() {
  VerilatorSimCtrl &simctrl = VerilatorSimCtrl::GetInstance();

  std::cout << "Simulation of Sonata System" << std::endl
            << "===========================" << std::endl
            << std::endl;

  simctrl.RunSimulation();
}

bool SonataSystem::Finish() {
  VerilatorSimCtrl &simctrl = VerilatorSimCtrl::GetInstance();

  if (!simctrl.WasSimulationSuccessful()) {
    return false;
  }

  // Set the scope to the root scope, the ibex_pcount_string function otherwise
  // doesn't know the scope itself. Could be moved to ibex_pcount_string, but
  // would require a way to set the scope name from here, similar to MemUtil.
  svSetScope(svGetScopeFromName("TOP.top_verilator"));

  std::cout << "\nPerformance Counters" << std::endl
            << "====================" << std::endl;
  std::cout << ibex_pcount_string(false);

  std::ofstream pcount_csv("sonata_system_pcount.csv");
  pcount_csv << ibex_pcount_string(true);

  return true;
}
