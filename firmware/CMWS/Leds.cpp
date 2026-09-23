#include "Leds.h"

namespace {

// -------------------------------------------------------------------
// Perceptual correction.
//
// The eye responds logarithmically: a linear 0..100 ramp looks like
// everything happens between 0 and 20 %. This table (gamma 2.2) maps
// a percentage to a 0..100 duty cycle so that turning the pot feels
// even. Any non-zero request maps to at least 1, so "barely on" never
// silently becomes "off". 73 of the 101 settings end up visually
// distinct; the rest collapse at the bottom, where gamma is steepest.
// -------------------------------------------------------------------
const uint8_t kGamma[cfg::BRIGHTNESS_MAX + 1] = {
    0,   1,   1,   1,   1,   1,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   2,   2,   2,   2,   3,
    3,   3,   4,   4,   4,   5,   5,   6,   6,   7,
    7,   8,   8,   9,   9,  10,  11,  11,  12,  13,
   13,  14,  15,  16,  16,  17,  18,  19,  20,  21,
   22,  23,  24,  25,  26,  27,  28,  29,  30,  31,
   33,  34,  35,  36,  37,  39,  40,  41,  43,  44,
   46,  47,  49,  50,  52,  53,  55,  56,  58,  60,
   61,  63,  65,  66,  68,  70,  72,  74,  75,  77,
   79,  81,  83,  85,  87,  89,  91,  94,  96,  98,
  100
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
// LEDs are stored grouped by port, in one flat pair of arrays, and each
// group points at a contiguous slice of them. A per-group 2D table would
// have to be sized LED_COUNT x LED_COUNT and would waste most of it:
// there are only ever LED_COUNT entries in total, however they split.
uint32_t s_bit[cfg::LED_COUNT]    = { 0 };  // BSRR bit of that LED
uint8_t  s_dutyOf[cfg::LED_COUNT] = { 0 };  // its index in s_duty

struct PortGroup {
  GPIO_TypeDef* port    = nullptr;
  uint32_t      allMask = 0;  // every LED bit on this port
  uint8_t       first   = 0;  // slice start in s_bit / s_dutyOf
  uint8_t       count   = 0;
};

// Upper bound: one distinct port per LED. Today it resolves to 2.
constexpr uint8_t kMaxPorts = cfg::LED_COUNT;

PortGroup s_group[kMaxPorts];
uint8_t   s_groupCount = 0;

// Requested brightness per LED, and the global scale, both in percent.
uint8_t s_percent[cfg::LED_COUNT] = { 0 };
uint8_t s_master = cfg::BRIGHTNESS_MAX;

// Last mask handed to setMask(), so a per-iteration call is free.
// set() invalidates it, so a per-LED level is never silently kept by a
// setMask() that thinks it has nothing to do.
uint8_t s_lastMask  = 0;
bool    s_maskKnown = false;

// Gamma-corrected duty actually used by the ISR, 0..PWM_LEVELS.
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
// The counter walks 0..PWM_LEVELS-1, so one tick is one percent of
// duty cycle. Two register stores drive all six LEDs.
// -------------------------------------------------------------------
void onPwmTick() {
  static uint8_t counter = 0;
  if (++counter >= cfg::PWM_LEVELS) {
    counter = 0;
  }

  for (uint8_t g = 0; g < s_groupCount; g++) {
    const PortGroup& grp = s_group[g];
    const uint8_t    end = grp.first + grp.count;

    // Collect the LEDs of this port that should be lit right now...
    uint32_t on = 0;
    for (uint8_t j = grp.first; j < end; j++) {
      if (counter < s_duty[s_dutyOf[j]]) {
        on |= s_bit[j];
      }
    }

    // ...and write the whole port in one atomic store: low half sets,
    // high half clears. Everything of ours that is not "on" is "off",
    // so the clear mask costs nothing to derive.
    grp.port->BSRR = on | ((grp.allMask & ~on) << 16);
  }
}

}  // namespace

namespace Leds {

// Resolves an LED index to its GPIO port and its BSRR bit.
void resolvePin(uint8_t index, GPIO_TypeDef*& port, uint32_t& bit) {
  const PinName name = digitalPinToPinName(cfg::PIN_LED[index]);
  port = get_GPIO_Port(STM_PORT(name));
  bit  = STM_GPIO_PIN(name);
}

void begin() {
  s_groupCount = 0;

  // Pass 1: configure the pins and collect the distinct ports.
  for (uint8_t i = 0; i < cfg::LED_COUNT; i++) {
    pinMode(cfg::PIN_LED[i], OUTPUT);
    digitalWrite(cfg::PIN_LED[i], LOW);
    s_percent[i] = 0;

    GPIO_TypeDef* port = nullptr;
    uint32_t      bit  = 0;
    resolvePin(i, port, bit);

    uint8_t g = 0;
    while (g < s_groupCount && s_group[g].port != port) {
      g++;
    }
    if (g == s_groupCount) {
      s_group[s_groupCount++].port = port;
    }
  }

  // Pass 2: lay the LEDs out grouped by port, so the ISR walks a
  // contiguous slice and never searches.
  uint8_t next = 0;
  for (uint8_t g = 0; g < s_groupCount; g++) {
    PortGroup& grp = s_group[g];
    grp.first   = next;
    grp.count   = 0;
    grp.allMask = 0;

    for (uint8_t i = 0; i < cfg::LED_COUNT; i++) {
      GPIO_TypeDef* port = nullptr;
      uint32_t      bit  = 0;
      resolvePin(i, port, bit);

      if (port != grp.port) {
        continue;
      }

      s_bit[next]    = bit;
      s_dutyOf[next] = i;
      next++;

      grp.count++;
      grp.allMask |= bit;
    }
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
