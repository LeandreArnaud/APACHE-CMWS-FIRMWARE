// =====================================================================
// CMWS - countermeasures panel firmware
//
// Target: Blue Pill STM32F103C8T6
// FQBN:   STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8
//
// Hardware: SH1106 128x64 OLED (I2C1, mounted upside down), 8 two-
// position switches, one 3-position mode switch, 2 potentiometers,
// 6 LEDs on software PWM.
//
// This file is wiring only: no pin numbers, no register access, no
// hardware knowledge. Those live in Config.h and in the modules.
//
// Note: Servo and tone() must never be used here. On this variant they
// claim TIM2 and TIM3, and TIM2 drives the LED PWM.
// =====================================================================

#include "Config.h"
#include "PanelState.h"
#include "Inputs.h"
#include "Leds.h"
#include "Screen.h"

// What the panel shows. For now these are the constants the v0.1 sketch
// drew inline; later the game will fill this in over the link.
PanelOutputs outputs;

// The only place in the firmware that holds application rules.
static void applyPanelPolicy(const PanelInputs& in) {
  if (in.mode == ModeSwitch::Off) {
    Leds::setMaster(0);
    Screen::setPowerSave(true);
    return;
  }

  // Standby, On and Unknown all behave as "powered" for now. They are
  // decoded into PanelInputs, but no behaviour is invented for them
  // until the game link defines one.
  Screen::setPowerSave(false);

  Leds::setMask(outputs.ledMask);
  Leds::setMaster(
      (in.potBrightness * cfg::BRIGHTNESS_MAX) / cfg::ADC_MAX);

  Screen::setBrightness(in.potBrightness);
}

void setup() {
  Inputs::begin();
  Leds::begin();
  Screen::begin();
}

void loop() {
  const uint32_t now = millis();

  Inputs::poll(now);
  const PanelInputs& in = Inputs::state();

  applyPanelPolicy(in);
  Screen::render(outputs, now);
}
