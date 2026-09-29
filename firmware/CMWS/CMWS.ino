// =====================================================================
// CMWS - countermeasures panel firmware
//
// Target: Blue Pill STM32F103C8T6
// FQBN:   STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8[,usb=HID]
//
// The panel enumerates as a USB joystick: 11 buttons, 2 axes. Two USB
// backends exist behind the same API, chosen by Tools > USB support:
//  - "HID (keyboard and mouse)": the joystick class rides on the core's
//    HAL USB stack (UsbHidJoystick.cpp). Use this on a genuine STM32.
//  - "None": a bare-metal stack driving the peripheral directly
//    (UsbBareMetal.cpp). Written for Blue Pills fitted with a Hangshun
//    HK32F103 (non-A), whose USB block the core's stack cannot drive.
//
// Hardware: SH1106 128x64 OLED (I2C1, mounted upside down), 8 two-
// position switches, one 3-position mode switch, 2 potentiometers,
// 6 LEDs on software PWM, USB on PA11/PA12.
//
// This file is wiring only: no pin numbers, no register access, no
// hardware knowledge. Those live in Config.h and in the modules.
//
// Never use here:
//  - Servo or tone(): on this variant they claim TIM2 and TIM3, and
//    TIM2 drives the LED PWM.
//  - <Keyboard.h> or <Mouse.h>: their begin() re-initialises the single
//    USB peripheral and would silently replace the joystick.
// =====================================================================

#include "Config.h"
#include "PanelState.h"
#include "Inputs.h"
#include "Leds.h"
#include "Screen.h"
#include "UsbGamepad.h"

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

  // On, Test and Unknown all behave as "powered" for now. They are
  // decoded into PanelInputs, but no behaviour is invented for them
  // until the game link defines one.
  Screen::setPowerSave(false);

  Leds::setMask(outputs.ledMask);
  Leds::setMaster(
      (in.potBrightness * cfg::BRIGHTNESS_MAX) / cfg::ADC_MAX);

  Screen::setBrightness(in.potBrightness);
}

void setup() {
  // USB first: enumeration then overlaps the OLED initialisation
  // instead of waiting behind it.
  UsbGamepad::begin();

  Inputs::begin();
  Leds::begin();
  Screen::begin();
}

void loop() {
  const uint32_t now = millis();

  Inputs::poll(now);
  const PanelInputs& in = Inputs::state();

  // The host gets the panel state in every mode, Off included: the
  // game must know the lever is on OFF.
  UsbGamepad::update(in, now);

  applyPanelPolicy(in);
  Screen::render(outputs, now);
}
