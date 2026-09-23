#pragma once

#include <Arduino.h>

// =====================================================================
// Board configuration - Blue Pill STM32F103C8T6
//
// Everything about "what is wired where" and "what is tuned to what"
// lives here, and nowhere else. Rewiring the panel means editing this
// file only.
// =====================================================================

namespace cfg {

// ---------------------------------------------------------------------
// Display: SH1106 128x64 OLED on I2C1, mounted upside down (U8G2_R2)
// ---------------------------------------------------------------------

constexpr uint32_t PIN_OLED_SDA = PB7;
constexpr uint32_t PIN_OLED_SCL = PB6;

// 400 kHz is the hard ceiling of the F103 I2C peripheral. Asking for
// more is silently accepted in release builds and misconfigures CCR.
// The legacy firmware ran at the 100 kHz default: a full 1024-byte
// frame took ~100 ms, versus ~25 ms here.
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// Upper bound on redraws. The panel is static most of the time and the
// dirty flag usually skips the redraw entirely; this only caps the rate
// when something really is changing.
constexpr uint16_t SCREEN_MIN_PERIOD_MS = 50;

// Below this raw ADC reading (~5% of 4095) the screen is turned off.
constexpr uint16_t SCREEN_OFF_BELOW = 205;

// ---------------------------------------------------------------------
// 8 two-position switches (active low, internal pull-ups)
// ---------------------------------------------------------------------

constexpr uint8_t SWITCH_COUNT = 8;

constexpr uint32_t PIN_SWITCH[SWITCH_COUNT] = {
  PC13, PC14, PC15, PA0, PA1, PA2, PA3, PA4
};

// ---------------------------------------------------------------------
// 3-position mode switch (active low)
// ---------------------------------------------------------------------

constexpr uint32_t PIN_MODE_OFF     = PB0;
constexpr uint32_t PIN_MODE_STANDBY = PB1;
constexpr uint32_t PIN_MODE_ON      = PB10;

// ---------------------------------------------------------------------
// 6 LEDs
//
// Software PWM, not hardware PWM. On the F103C8 these six pins are
// TIM1 CH1/CH2/CH3 (PA8/PA9/PA10) and their complementary outputs
// CH1N/CH2N/CH3N (PB13/PB14/PB15). A channel and its complement share
// one compare register, so hardware PWM could only ever produce three
// independent duty cycles here - with the PB pins forced to the inverse
// of the PA pins. No other timer reaches PB13/PB14/PB15 on this package.
// ---------------------------------------------------------------------

constexpr uint8_t LED_COUNT = 6;

constexpr uint32_t PIN_LED[LED_COUNT] = {
  PB13, PB14, PB15, PA8, PA9, PA10
};

// ---------------------------------------------------------------------
// Potentiometers (12-bit ADC)
// ---------------------------------------------------------------------

constexpr uint32_t PIN_POT_BRIGHTNESS = PA5;  // drives BOTH screen and LEDs
constexpr uint32_t PIN_POT_AUX        = PA6;  // read into state, no role yet

constexpr uint16_t ADC_MAX = 4095;

// ---------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------

constexpr uint16_t INPUT_PERIOD_MS = 2;
constexpr uint8_t  DEBOUNCE_TICKS  = 4;   // 4 x 2 ms = 8 ms of stability

// Smoothing strength for the pots: value += (raw - value) >> SHIFT.
constexpr uint8_t  POT_EMA_SHIFT = 3;

// A smoothed pot must move by more than this many ADC counts before the
// new value is published. Without it, +/-5 LSB of ADC noise would make
// the LEDs shimmer and redraw the screen forever.
constexpr uint16_t POT_DEADBAND = 24;

// ---------------------------------------------------------------------
// Software PWM
// ---------------------------------------------------------------------

// The public brightness API is in percent; internally the duty cycle
// has 256 steps, because the ISR counter is a uint8_t that wraps on its
// own - no compare, no reset branch.
constexpr uint8_t BRIGHTNESS_MAX = 100;

// 32000 / 256 = 125 Hz refresh. The legacy firmware ran 100 levels at
// 10 kHz (100 Hz); 256 steps are what makes the gamma table usable,
// since gamma spends most of its resolution at the bottom of the range.
// At 32 kHz there are 2250 CPU cycles between ticks and the ISR needs
// well under a hundred - which is exactly why it writes BSRR instead of
// calling digitalWrite six times (that alone would be ~250 cycles).
constexpr uint32_t PWM_TICK_HZ = 32000;

// The core defaults timer interrupts to priority 14, which is below the
// I2C interrupt (priority 2). That lets an OLED refresh preempt the PWM
// ISR and makes the LEDs flicker. 1 puts us above I2C while staying
// below SysTick (0), so millis() keeps working.
constexpr uint32_t PWM_IRQ_PRIORITY    = 1;
constexpr uint32_t PWM_IRQ_SUBPRIORITY = 0;

}  // namespace cfg
