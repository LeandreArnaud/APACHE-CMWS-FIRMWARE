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

// Rotary OFF / ON / TEST, one contact per position (mapped 2026-09-29).
constexpr uint32_t PIN_MODE_OFF  = PB0;
constexpr uint32_t PIN_MODE_ON   = PB1;
constexpr uint32_t PIN_MODE_TEST = PB10;

// ---------------------------------------------------------------------
// Physical levers
//
// Each 2-position lever has one contact per position, wired to two of
// the inputs above. A lever is described by the indexes (into
// PIN_SWITCH) of its two contacts, first position first. The USB report
// exposes one button per contact, in this order: lever 1 position A,
// lever 1 position B, lever 2 position A, ... then the rotary.
// Mapped on the panel 2026-09-29 by toggling each lever in turn.
// ---------------------------------------------------------------------

constexpr uint8_t LEVER_COUNT = 4;

struct Lever {
  uint8_t contactA;   // index into PIN_SWITCH closed in position A
  uint8_t contactB;   // index into PIN_SWITCH closed in position B
};

constexpr Lever LEVERS[LEVER_COUNT] = {
  { 7, 6 },   // 1: ARM      (PA4) / SAFE (PA3)
  { 5, 4 },   // 2: CMWS     (PA2) / NAV  (PA1)
  { 3, 2 },   // 3: BYPASS   (PA0) / AUTO (PC15)
  { 1, 0 },   // 4: JETTISON ON (PC14) / OFF (PC13)
};

// Buttons as the host numbers them (1-based): 2 per lever, then the
// rotary OFF / ON / TEST. 11 in total; the report pads to 16 bits.
constexpr uint8_t USB_BUTTON_COUNT = 2 * LEVER_COUNT + 3;

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

constexpr uint32_t PIN_POT_BRIGHTNESS = PA5;  // "LAMP": drives BOTH screen and LEDs, USB axis X
constexpr uint32_t PIN_POT_AUX        = PA6;  // "AUDIO": USB axis Y, no local role

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

// Brightness is expressed in percent everywhere, and the duty cycle
// has the same 100 steps - one counter tick is one percent.
//
// 256 steps were measured and dropped: they need a 32 kHz tick, which
// costs ~8.4 % of the CPU against 2.6 % here, and buy only 88 visually
// distinct settings instead of 73. Not worth 3x the interrupt rate.
constexpr uint8_t BRIGHTNESS_MAX = 100;
constexpr uint8_t PWM_LEVELS     = 100;

// 10000 / 100 = 100 Hz refresh, same as the legacy firmware. The ISR
// costs about 190 cycles out of the 7200 available between two ticks,
// because it writes BSRR once per port instead of calling digitalWrite
// six times (that alone would be ~250 cycles).
constexpr uint32_t PWM_TICK_HZ = 10000;

// The core defaults timer interrupts to priority 14, which is below the
// I2C interrupt (priority 2). That lets an OLED refresh preempt the PWM
// ISR and makes the LEDs flicker. 1 puts us above I2C while staying
// below SysTick (0), so millis() keeps working.
constexpr uint32_t PWM_IRQ_PRIORITY    = 1;
constexpr uint32_t PWM_IRQ_SUBPRIORITY = 0;

// ---------------------------------------------------------------------
// USB HID joystick
//
// VID 0x1209 is pid.codes, free for open-source hardware. PIDs 0x0001
// to 0x000F are their test range: fine for personal panels, and each
// panel must get its own so Windows never mixes two of them up. The
// next panel is 0x0002. An official PID can be requested later; only
// this constant would change.
//
// Do not rename these USB_PRODUCT_STRING / USB_MANUFACTURER_STRING:
// those are macros the core consumes for its own (unused) descriptors.
// ---------------------------------------------------------------------

constexpr uint16_t USB_VID = 0x1209;
constexpr uint16_t USB_PID = 0x0001;

constexpr char USB_MANUFACTURER_NAME[] = "SimPeet";
constexpr char USB_PRODUCT_NAME[]      = "SimPeet CMWS";

// A report goes out whenever the panel state changes. This is the
// upper bound between two reports when nothing changes, so a host that
// missed one (re-enumeration, dropped packet) catches up quickly.
constexpr uint16_t USB_HEARTBEAT_MS = 250;

// Reset-storm guard. Some F103 clones raise the USB RESET interrupt
// without pause when no host is attached; unguarded, that interrupt
// eats 100% of the CPU before setup() even reaches the screen. A real
// host sends two or three resets during enumeration, tens of
// milliseconds apart, so this threshold cannot trip on a healthy bus.
// After tripping, USB is shut down and retried every USB_RETRY_MS.
constexpr uint8_t  USB_STORM_RESETS    = 50;
constexpr uint16_t USB_STORM_WINDOW_MS = 50;
constexpr uint16_t USB_RETRY_MS        = 2000;

// Interrupt priority of the bare-metal USB backend (menu USB = None).
// Below the LED PWM (1) so a packet never jitters the LEDs, below
// SysTick (0) so millis() keeps running. The HAL backend takes the same
// value through build_opt.h (-DUSBD_IRQ_PRIO=2).
constexpr uint32_t USB_IRQ_PRIORITY = 2;

}  // namespace cfg
