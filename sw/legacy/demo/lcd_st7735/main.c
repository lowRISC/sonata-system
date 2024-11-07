// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "core/lucida_console_10pt.h"
#include "core/m3x6_16pt.h"
#include "fbcon.h"
#include "fractal.h"
#include "gpio.h"
#include "pwm.h"
#include "lcd.h"
#include "lowrisc_logo.h"
#include "sonata_system.h"
#include "spi.h"
#include "st7735/lcd_st7735.h"
#include "timer.h"

#define SIMULATION 0

// Constants.
enum {
  // Pin out mapping using Spi CS lines
  LcdCsLine = 0,
  LcdDcLine,
  LcdRstLine,
  // Other SPI pins
  LcdMosiPin,
  LcdSclkPin,
  // Spi clock rate.
  SpiSpeedHz = 5 * 100 * 1000,
};

// Buttons
// The direction is relative to the screen in landscape orientation.
typedef enum {
  BTN_DOWN  = 0b00001 << 8,
  BTN_LEFT  = 0b00010 << 8,
  BTN_CLICK = 0b00100 << 8,
  BTN_RIGHT = 0b01000 << 8,
  BTN_UP    = 0b10000 << 8,
} Buttons_t;

// Local functions declaration.
static uint32_t spi_write(void *handle, uint8_t *data, size_t len);
static uint32_t gpio_write(void *handle, bool cs, bool dc);
static void timer_delay(uint32_t ms);
static void fractal_test(St7735Context *lcd);
static Buttons_t scan_buttons(uint32_t timeout);

int main(void) {
  timer_init();

  // Init spi driver.
  spi_t spi;
  spi_init(&spi, LCD_SPI, SpiSpeedHz);

  // Turn on LCD backlight via PWM
  pwm_t lcd_bl = PWM_FROM_ADDR_AND_INDEX(PWM_BASE, PWM_LCD);
  set_pwm(lcd_bl, 1, 255);

  // Set the initial state of the LCD control pins.
  spi_set_cs(&spi, LcdDcLine, 0x0);
  spi_set_cs(&spi, LcdCsLine, 0x0);

  // Reset LCD.
  spi_set_cs(&spi, LcdRstLine, 0x0);
  timer_delay(150);
  spi_set_cs(&spi, LcdRstLine, 0x1);

  // Init LCD driver and set the SPI driver.
  St7735Context lcd;
  LCD_Interface interface = {
      .handle      = &spi,         // SPI handle.
      .spi_write   = spi_write,    // SPI write callback.
      .gpio_write  = gpio_write,   // GPIO write callback.
      .timer_delay = timer_delay,  // Timer delay callback.
  };
  lcd_st7735_init(&lcd, &interface);

  // Set the LCD orientation.
  lcd_st7735_set_orientation(&lcd, LCD_Rotate180);

  // Setup text font bitmaps to be used and the colors.
  lcd_st7735_set_font(&lcd, &lucidaConsole_10ptFont);
  lcd_st7735_set_font_colors(&lcd, BGRColorWhite, BGRColorBlack);

  // Clean display with a white rectangle.
  lcd_st7735_clean(&lcd);

  // Draw the splash screen with a RGB 565 bitmap and text in the bottom.
  lcd_st7735_draw_rgb565(&lcd, (LCD_rectangle){.origin = {.x = (160 - 105) / 2, .y = 5}, .width = 105, .height = 80},
                         (uint8_t *)lowrisc_logo_105x80);

  lcd_println(&lcd, "Booting...", alined_center, (LCD_Point){.x = 0, .y = 100});
  timer_delay(1000);

  // Show the main menu.
  const char *items[] = {
      "0. Fractal",
      "1. CoreMark",
  };
  Menu_t main_menu = {
      .title          = "Main menu",
      .color          = BGRColorBlue,
      .selected_color = BGRColorRed,
      .background     = BGRColorWhite,
      .items_count    = sizeof(items) / sizeof(items[0]),
      .items          = items,
  };

  bool repaint    = true;
  size_t selected = 0;
  char line_buffer[21];

  // Boot countdown when no button is pressed. Value 0 indicates the countdown is dismissed.
  int boot_countdown_sec = 3;

menu:
  while (1) {
    if (repaint) {
      repaint = false;
      lcd_st7735_clean(&lcd);
      lcd_show_menu(&lcd, &main_menu, selected);

      if (boot_countdown_sec != 0) {
        lcd_st7735_puts(&lcd, (LCD_Point){.x = 8, .y = 102}, "Defaulting to item");
        strcpy(line_buffer, "0 after 0 seconds");
        line_buffer[strlen("0 after ")] += boot_countdown_sec;
        lcd_st7735_puts(&lcd, (LCD_Point){.x = 12, .y = 115}, line_buffer);
      }
    }

    switch (scan_buttons(1000)) {
      case BTN_UP:
        if (selected > 0) {
          selected--;
        } else {
          selected = main_menu.items_count - 1;
        }
        repaint            = true;
        boot_countdown_sec = 0;
        break;
      case BTN_DOWN:
        if (selected < main_menu.items_count - 1) {
          selected++;
        } else {
          selected = 0;
        }
        repaint            = true;
        boot_countdown_sec = 0;
        break;
      // Left/right buttons currently don't do anything.
      case BTN_LEFT:
      case BTN_RIGHT:
        continue;

      case BTN_CLICK:
        goto boot;

      default:
        if (boot_countdown_sec == 0) {
          continue;
        }

        if (--boot_countdown_sec == 0) {
          goto boot;
        }

        repaint = true;
        break;
    }
  };

boot:
  switch (selected) {
    case 0:
      fractal_test(&lcd);
      break;

    case 1:
      // Switch to a smaller font for the coremark.
      lcd_st7735_set_font(&lcd, &m3x6_16ptFont);

      fbcon_init(&lcd);

      // Trick to make the coremark appear at bottom.
      fbcon_putstr("\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

      // Clean the screen and draw "CoreMark" as title bar.
      lcd_st7735_clean(&lcd);
      lcd_st7735_fill_rectangle(
          &lcd,
          (LCD_rectangle){.origin = {.x = 0, .y = 0}, .width = lcd.parent.width, .height = lcd.parent.font->height + 2},
          BGRColorBlue);
      lcd_st7735_set_font_colors(&lcd, BGRColorBlue, BGRColorWhite);
      lcd_println(&lcd, "CoreMark", alined_center, (LCD_Point){.x = 0, .y = 1});
      lcd_st7735_set_font_colors(&lcd, BGRColorWhite, BGRColorBlue);

      int coremark_main();
      coremark_main();
      break;
  }

  // Wait until navigation button is clicked.
  while (scan_buttons(1000) != BTN_CLICK);

  // Return to the main menu.
  repaint = true;
  goto menu;

  return 0;
}

static Buttons_t scan_buttons(uint32_t timeout) {
  while (true) {
    // Sample navigation buttons (debounced).
    uint32_t in_val = read_gpio(GPIO_IN_DBNC) & (0x1f << 8);
    if (in_val == 0) {
      // No button pressed, so delay for 20ms and then try again, unless the timeout is reached.
      const uint32_t poll_delay = 20;
      timer_delay(poll_delay);
      if (timeout < poll_delay) {
        // Timeout reached, return 0.
        return 0;
      } else {
        // Timeout not reached yet, decrease it and try again.
        timeout -= poll_delay;
      }
      continue;
    }

    // Some button pressed.
    // Find the most significant bit set.
    in_val |= in_val >> 1;
    in_val |= in_val >> 2;
    in_val |= in_val >> 4;
    in_val = ((in_val >> 1) & (0x1f << 8)) + 1;

    // Wait until the button is released to avoid an event being triggered multiple times.
    while (read_gpio(GPIO_IN_DBNC) & in_val);

    return in_val;
  }
}

static void fractal_test(St7735Context *lcd) {
  fractal_mandelbrot_float(lcd);
  timer_delay(5000);
  fractal_mandelbrot_fixed(lcd);
  timer_delay(5000);
}

static uint32_t spi_write(void *handle, uint8_t *data, size_t len) {
  spi_tx((spi_t *)handle, data, len);
  spi_wait_idle((spi_t *)handle);
  return len;
}

static uint32_t gpio_write(void *handle, bool cs, bool dc) {
  spi_set_cs((spi_t *)handle, LcdDcLine, dc);
  spi_set_cs((spi_t *)handle, LcdCsLine, cs);
  return 0;
}

static void timer_delay(uint32_t ms) {
#if !SIMULATION
  // Configure timer to trigger every 1 ms
  timer_enable(SYSCLK_FREQ / 1000);
  uint32_t timeout = get_elapsed_time() + ms;
  while (get_elapsed_time() < timeout) {
    asm volatile("wfi");
  }
  timer_disable();
#endif
}
