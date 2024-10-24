// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef SONATA_SYSTEM_H_
#define SONATA_SYSTEM_H_

#include <stddef.h>
#include <stdint.h>

#include "gpio.h"
#include "sonata_system_regs.h"
#include "uart.h"

// System Clock Frequency (Hz)
#define SYSCLK_FREQ (40 * 1000 * 1000)

#define UART_IRQ_NUM 16
#define UART_IRQ (1 << UART_IRQ_NUM)
#define DEFAULT_UART UART_FROM_BASE_ADDR(UART0_BASE)

#define GPIO_OUT GPIO_FROM_BASE_ADDR(GPIO_BASE + GPIO_OUT_REG)
#define GPIO_IN GPIO_FROM_BASE_ADDR(GPIO_BASE + GPIO_IN_REG)
#define GPIO_IN_DBNC GPIO_FROM_BASE_ADDR(GPIO_BASE + GPIO_IN_DBNC_REG)
#define GPIO_OUT_SHIFT GPIO_FROM_BASE_ADDR(GPIO_BASE + GPIO_OUT_SHIFT_REG)

#define TIMER_IRQ (1 << 7)

#define NUM_PWM_MODULES 12

#define DEFAULT_I2C I2C_FROM_BASE_ADDR(I2C0_BASE)
#define FLASH_SPI SPI_FROM_BASE_ADDR(SPI0_BASE)
#define LCD_SPI SPI_FROM_BASE_ADDR(SPI1_BASE)
#define ETH_SPI SPI_FROM_BASE_ADDR(SPI2_BASE)
#define RPI_SPI0 SPI_FROM_BASE_ADDR(SPI3_BASE)
#define RPI_SPI1 SPI_FROM_BASE_ADDR(SPI4_BASE)
#define ARDUINO_SPI SPI_FROM_BASE_ADDR(SPI5_BASE)
#define MIKRO_BUS_SPI SPI_FROM_BASE_ADDR(SPI6_BASE)
#define DEFAULT_USBDEV USBDEV0_BASE

/**
 * Writes character to default UART. Signature matches c stdlib function
 * of the same name.
 *
 * @param c Character to output
 * @returns Character output (never fails so no EOF ever returned)
 */
int putchar(int c);

/**
 * Reads character from default UART. Signature matches c stdlib function
 * of the same name.
 *
 * @returns Character from the uart rx fifo
 */
int getchar(void);

/**
 * Immediately halts the simulation
 */
void sim_halt();

/**
 * Writes string and newline to default UART. Signature matches c stdlib
 * function of the same name.
 *
 * @param str String to output
 * @returns 0 always (never fails so no error)
 */
int puts(const char *str);

/**
 * Writes string to default UART.
 *
 * @param str String to output
 * @returns 0 always (never fails so no error)
 */
int putstr(const char *str);

/**
 * Writes ASCII hex representation of number to default UART.
 *
 * @param h Number to output in hex
 */
void puthex(uint32_t h);

/**
 * Writes shortened ASCII hex representation of a number to default UART.
 *
 * @param h Number to output in hex
 * @param n Number of digits (nibbles) to output
 */
void puthexn(uint32_t h, unsigned n);

/**
 * Writes shortened ASCII hex representation of a number to the supplied
 * buffer.
 *
 * @param buf Buffer to receive ASCII hex string
 * @param sz  Size of buffer in bytes
 * @param h   Number to output in hex
 * @param n   Number of digits (nibbles) to output
 * @return    Number of digits emitted
 */
unsigned snputhexn(char *buf, size_t sz, uint32_t h, unsigned n);

/**
 * Write ASCII decimal representation of unsigned number to default UART.
 *
 * @param d Number to output in decimal
 */
void putdec(uint32_t d);

/**
 * Install an exception handler by writing a `j` instruction to the handler in
 * at the appropriate address given the `vector_num`.
 *
 * @param vector_num Which IRQ the handler is for, must be less than 32. All
 * non-interrupt exceptions are handled at vector 0.
 *
 * @param handle_fn Function pointer to the handler function. The function is
 * responsible for interrupt prolog and epilog, such as saving and restoring
 * register to the stack and executing `mret` at the end.
 *
 * @return 0 on success, 1 if `vector_num` out of range, 2 if the address of
 * `handler_fn` is too far from the exception handler base to use with a `j`
 * instruction.
 */
int install_exception_handler(uint32_t vector_num, void (*handler_fn)(void));

/**
 * Set per-interrupt enables (`mie` CSR)
 *
 * @param enable_mask Any set bit is set in `mie`, enabling the interrupt. Bits
 * not set in `enable_mask` aren't changed.
 */
void enable_interrupts(uint32_t enable_mask);

/**
 * Clear per-interrupt enables (`mie` CSR)
 *
 * @param enable_mask Any set bit is cleared in `mie`, disabling the interrupt.
 * Bits not set in `enable_mask` aren't changed.
 */
void disable_interrupts(uint32_t disable_mask);

#define MSTATUS_MIE (1 << 3)

/**
 * Save and clear global interrupt enable (the `mie` field of `mstatus`).
 *
 * @return The saved state that can be used to restore the global interrupt
 *         enable state with `arch_local_irq_restore`.
 */
static inline uint32_t arch_local_irq_save(void) {
  uint32_t mstatus;
  asm volatile("csrrc %0, mstatus, %1" : "=r"(mstatus) : "rK"(MSTATUS_MIE));
  return mstatus;
}

/**
 * Restore global interrupt enable (the `mie` field of `mstatus`).
 *
 * @param mstatus The saved state from `arch_local_irq_save`.
 */
static inline void arch_local_irq_restore(uint32_t mstatus) {
  // Set the MIE bit using the mstatus.
  // Does nothing if MIE is not set in mstatus (indicating reentering a local_irq_save region).
  asm volatile("csrs mstatus, %0" : : "rK"(mstatus & MSTATUS_MIE));
}

/**
 * Set the global interrupt enable (the `mie` field of `mstatus`).
 */
static inline void arch_local_irq_enable(void) { asm volatile("csrs mstatus, %0" : : "rK"(MSTATUS_MIE)); }

/**
 * Clear the global interrupt enable (the `mie` field of `mstatus`).
 */
static inline void arch_local_irq_disable(void) { asm volatile("csrc mstatus, %0" : : "rK"(MSTATUS_MIE)); }

unsigned int get_mepc();
unsigned int get_mcause();
unsigned int get_mtval();
uint32_t get_mcycle(void);
void reset_mcycle(void);

#endif
