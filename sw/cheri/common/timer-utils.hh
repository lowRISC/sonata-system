#pragma once
#include <stdint.h>

static inline uint32_t get_mcycle(void)
{
	uint32_t result;
	asm volatile("csrr %0, mcycle;" : "=r"(result));
	return result;
}

static inline void reset_mcycle(void)
{
	asm volatile("csrw mcycle, x0");
}

static inline void wait_mcycle(uint32_t value)
{
	reset_mcycle();
	while (get_mcycle() < value) {}
}
