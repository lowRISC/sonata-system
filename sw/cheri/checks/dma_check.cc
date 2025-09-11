/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include <stdint.h>

#include <cheri.hh>
// clang-format off
#include "../../common/defs.h"
// clang-format on
#include "../common/console.hh"
#include "../common/timer-utils.hh"

using namespace CHERI;

struct SonataDma {
  /**
   * Interrupt State Register.
   */
  uint32_t interruptState;
  /**
   * Interrupt Enable Register.
   */
  uint32_t interruptEnable;
  /**
   * Interrupt Test Register.
   */
  uint32_t interruptTest;

  // TODO: Under construction.
  uint32_t control;
  uint32_t status;

  uint32_t src_config;
  // TODO: Will need aligning.
  uint32_t src_cap_lo;
  uint32_t src_cap_hi;
  uint32_t src_stride;
  uint32_t src_row_len;
  uint32_t src_rows;

  uint32_t dst_config;
  // TODO: Will need aligning.
  uint32_t dst_cap_lo;
  uint32_t dst_cap_hi;
  uint32_t dst_stride;
  uint32_t dst_row_len;
  uint32_t dst_rows;
};

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart0 = root.cast<volatile OpenTitanUart>();
  uart0.address()                          = UART_ADDRESS;
  uart0.bounds()                           = UART_BOUNDS;

  uart0->init(BAUD_RATE);
  WriteUart uart{uart0};
  Log log(uart);

  // Create a bounded capability to the UART
  Capability<volatile SonataDma> dma = root.cast<volatile SonataDma>();
  dma.address()                      = DMA_ADDRESS;
  dma.bounds()                       = DMA_BOUNDS;

  while (true) {
    const bool logging = false;

    // Set up a simple copy from SRAM to HyperRAM
    // 8 rows x 0x400 words/row = 8192 words to be transferred.
    dma->src_config = 0x11u;
    dma->dst_config = 0x11u;

    dma->src_cap_lo  = SRAM_ADDRESS;
    dma->src_cap_hi  = 0u;
    dma->src_stride  = 0x404;
    dma->src_rows    = 0x7;
    dma->src_row_len = 0x3ff;

    dma->dst_cap_lo  = HYPERRAM_ADDRESS;
    dma->dst_cap_hi  = 0u;
    dma->dst_stride  = 0x804;
    dma->dst_rows    = 0x3;
    dma->dst_row_len = 0x7ff;

    log.println("Starting transfer");
    uint32_t start_time = get_mcycle();
    dma->control        = 1;

    int cnt = 0;
    if (logging) {
      log.println("Started, state {}", dma->status & 0xfu);
    }
    while (dma->status & 0xfu) {
      asm("");
      if (++cnt < 10)
        if (logging) {
          log.println(" - status {:#x}", dma->status);
        }
    }

    log.println("Took {} cycles", get_mcycle() - start_time);
  }
}
