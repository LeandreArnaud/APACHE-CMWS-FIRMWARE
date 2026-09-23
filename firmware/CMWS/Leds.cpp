#include "Leds.h"

namespace {

// -------------------------------------------------------------------
// Perceptual correction.
//
// The eye responds logarithmically: a linear 0..100 ramp looks like
// everything happens between 0 and 20 %. This table (gamma 2.2) maps
// a percentage to a 0..255 duty cycle so that turning the pot feels
// even. Any non-zero request maps to at least 1, so "barely on" never
// silently becomes "off".
// -------------------------------------------------------------------
const uint8_t kGamma[cfg::BRIGHTNESS_MAX + 1] = {
    0,   1,   1,   1,   1,   1,   1,   1,   1,   1,
    2,   2,   2,   3,   3,   4,   5,   5,   6,   7,
    7,   8,   9,  10,  11,  12,  13,  14,  15,  17,
   18,  19,  21,  22,  24,  25,  27,  29,  30,  32,
   34,  36,  38,  40,  42,  44,  46,  48,  51,  53,
   55,  58,  60,  63,  66,  68,  71,  74,  77,  80,
   83,  86,  89,  92,  96,  99, 102, 106, 109, 113,
  116, 120, 124, 128, 131, 135, 139, 143, 148, 152,
  156, 160, 165, 169, 174, 178, 183, 188, 192, 197,
  202, 207, 212, 217, 223, 228, 233, 238, 244, 249,
  255
};

// -------------------------------------------------------------------
// Pin table, resolved once in begin().
//
// The ISR must not call digitalWrite(): that is a non-inlined call
// plus three table lookups per pin. Instead every LED is reduced to a
// port pointer and a bit mask, and the ISR writes BSRR once per port.
//
// BSRR: the low 16 bits set pins, the high 16 bits clear them, in a
// single atomic store. Note STM_GPIO_PIN() and not STM_LL_GPIO_PIN() -
// on the F1 the LL_GPIO_PIN_x values are not 1 << n, they also encode
// the pin's position inside CRL/CRH.
// -------------------------------------------------------------------
struct LedPin {
  uint32_t mask      = 0;  // 1 << pin, ready for the low half of BSRR
  uint8_t  portIndex = 0;  // index into s_port, resolved in begin()
};

// Upper bound: one distinct port per LED.
constexpr uint8_t kMaxPorts = cfg::LED_COUNT;

LedPin        s_pin[cfg::LED_COUNT];
GPIO_TypeDef* s_port[kMaxPorts];  // distinct ports used (PA and PB today)
uint8_t       s_portCount = 0;

// Requested brightness per LED, and the global scale, both in percent.
uint8_t s_percent[cfg::LED_COUNT] = { 0 };
uint8_t s_master = cfg::BRIGHTNESS_MAX;

// Last mask handed to setMask(), so a per-iteration call is free.
// set() invalidates it, so a per-LED level is never silently kept by a
// setMask() that thinks it has nothing to do.
uint8_t s_lastMask  = 0;
bool    s_maskKnown = false;

// Gamma-corrected duty actually used by the ISR, 0..255.
// Written from loop() context, read from the ISR: a byte store is
// atomic on Cortex-M3, so no critical section is needed.
volatile uint8_t s_duty[cfg::LED_COUNT] = { 0 };

HardwareTimer* s_timer = nullptr;

uint8_t clampPercent(uint8_t percent) {
  return percent > cfg::BRIGHTNESS_MAX ? cfg::BRIGHTNESS_MAX : percent;
}

void recomputeDuty(uint8_t index) {
  const uint16_t scaled =
      (static_cast<uint16_t>(s_percent[index]) * s_master) / cfg::BRIGHTNESS_MAX;
  s_duty[index] = kGamma[scaled];
}

void recomputeAll() {
  for (uint8_t i = 0; i < cfg::LED_COUNT; i++) {
    recomputeDuty(i);
  }
}

// -------------------------------------------------------------------
// PWM interrupt.
//
// One uint8_t counter wrapping on its own gives 256 duty steps.
// Two register stores drive all six LEDs.
// -------------------------------------------------------------------
void onPwmTick() {
  static uint8_t counter = 0;
  counter++;

  uint32_t bsrr[kMaxPorts] = { 0 };

  for (uint8_t i = 0; i < cfg::LED_COUNT; i++) {
    const bool on = (counter < s_duty[i]);
    // Set = low half of BSRR, clear = high half.
    bsrr[s_pin[i].portIndex] |= on ? s_pin[i].mask : (s_pin[i].mask << 16);
  }

  for (uint8_t p = 0; p < s_portCount; p++) {
    s_port[p]->BSRR = bsrr[p];
  }
}

}  // namespace

namespace Leds {

void begin() {
  s_portCount = 0;

  for (uint8_t i = 0; i < cfg::LED_COUNT; i++) {
    const uint32_t pin = cfg::PIN_LED[i];

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    const PinName       name = digitalPinToPinName(pin);
    GPIO_TypeDef* const port = get_GPIO_Port(STM_PORT(name));

    s_pin[i].mask = STM_GPIO_PIN(name);

    // Resolve the port to an index now, so the ISR never searches.
    uint8_t index = 0;
    while (index < s_portCount && s_port[index] != port) {
      index++;
    }
    if (index == s_portCount) {
      s_port[s_portCount++] = port;
    }
    s_pin[i].portIndex = index;

    s_percent[i] = 0;
  }

  recomputeAll();

  s_timer = new HardwareTimer(TIM2);
  s_timer->setOverflow(cfg::PWM_TICK_HZ, HERTZ_FORMAT);
  s_timer->attachInterrupt(onPwmTick);

  // The core defaults timer interrupts to priority 14, below I2C's 2.
  // Left alone, every OLED refresh preempts this ISR and the LEDs
  // visibly flicker. Stay above 0 so SysTick - and therefore millis() -
  // still runs.
  s_timer->setInterruptPriority(cfg::PWM_IRQ_PRIORITY, cfg::PWM_IRQ_SUBPRIORITY);

  s_timer->resume();
}

void set(uint8_t index, uint8_t percent) {
  if (index >= cfg::LED_COUNT) {
    return;
  }
  s_percent[index] = clampPercent(percent);
  recomputeDuty(index);

  // A per-LED level no longer matches any mask: invalidate the cache so
  // the next setMask() really reapplies instead of short-circuiting.
  s_maskKnown = false;
}

void setMask(uint8_t mask) {
  // Called on every loop iteration, so skip the work when nothing moved.
  if (s_maskKnown && mask == s_lastMask) {
    return;
  }
  s_lastMask  = mask;
  s_maskKnown = true;

  for (uint8_t i = 0; i < cfg::LED_COUNT; i++) {
    s_percent[i] = (mask & (1u << i)) ? cfg::BRIGHTNESS_MAX : 0;
    recomputeDuty(i);
  }
}

void setMaster(uint8_t percent) {
  const uint8_t clamped = clampPercent(percent);
  if (clamped == s_master) {
    return;
  }
  s_master = clamped;
  recomputeAll();
}

}  // namespace Leds
