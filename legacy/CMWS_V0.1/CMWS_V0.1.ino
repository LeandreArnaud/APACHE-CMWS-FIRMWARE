#include <Wire.h>
#include <U8g2lib.h>

// =====================================================
// ECRAN SH1106 128x64 I2C
// SDA = PB7
// SCL = PB6
// =====================================================

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2);

// =====================================================
// 8 SWITCHS 2 POSITIONS
// =====================================================

const uint8_t nbButtons = 8;

const uint8_t buttonPins[8] = {
  PC13,
  PC14,
  PC15,
  PA0,
  PA1,
  PA2,
  PA3,
  PA4
};

// =====================================================
// POTENTIOMETRES
// =====================================================

const uint8_t brightnessPin = PA5;
const uint8_t pot2Pin = PA6;

// =====================================================
// SWITCH 3 POSITIONS
// =====================================================

const uint8_t switch3Pins[3] = {
  PB0,
  PB1,
  PB10
};

// =====================================================
// 6 LED
// =====================================================

const uint8_t nbLeds = 6;

const uint8_t leds[6] = {
  PB13,
  PB14,
  PB15,
  PA8,
  PA9,
  PA10
};

// =====================================================
// PWM
// =====================================================

const uint8_t PWM_LEVELS = 100;

volatile uint8_t pwmCounter = 0;
volatile uint8_t ledBrightness = 100;

HardwareTimer *pwmTimer = nullptr;

// =====================================================
// ETAT GENERAL
// =====================================================

volatile bool systemOff = false;

// =====================================================
// INTERRUPTION PWM
// =====================================================

void pwmInterrupt() {

  pwmCounter++;

  if (pwmCounter >= PWM_LEVELS) {
    pwmCounter = 0;
  }

  // Si PB0 demande l'extinction :
  // toutes les LED restent éteintes
  if (systemOff) {

    for (uint8_t i = 0; i < nbLeds; i++) {
      digitalWrite(leds[i], LOW);
    }

    return;
  }

  bool state = (pwmCounter < ledBrightness);

  for (uint8_t i = 0; i < nbLeds; i++) {
    digitalWrite(leds[i], state ? HIGH : LOW);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  // ---------------------------------------------------
  // ADC
  // ---------------------------------------------------

  analogReadResolution(12);

  // ---------------------------------------------------
  // ECRAN
  // ---------------------------------------------------

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  u8g2.begin();

  // ---------------------------------------------------
  // BOUTONS
  // ---------------------------------------------------

  for (uint8_t i = 0; i < nbButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // ---------------------------------------------------
  // SWITCH 3 POSITIONS
  // ---------------------------------------------------

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(switch3Pins[i], INPUT_PULLUP);
  }

  // ---------------------------------------------------
  // LEDS
  // ---------------------------------------------------

  for (uint8_t i = 0; i < nbLeds; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  // ---------------------------------------------------
  // TIMER PWM
  // ---------------------------------------------------

  pwmTimer = new HardwareTimer(TIM2);

  pwmTimer->setOverflow(10000, HERTZ_FORMAT);
  pwmTimer->attachInterrupt(pwmInterrupt);
  pwmTimer->resume();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // ===================================================
  // POSITION DU SWITCH 3 POSITIONS
  // ===================================================

  bool switchPB0 = !digitalRead(PB0);
  bool switchPB1 = !digitalRead(PB1);
  bool switchPB10 = !digitalRead(PB10);

  // ---------------------------------------------------
  // PB0 = TOUT ETEINDRE
  // ---------------------------------------------------

  if (switchPB0) {

    systemOff = true;

    // Eteindre immédiatement les LED
    for (uint8_t i = 0; i < nbLeds; i++) {
      digitalWrite(leds[i], LOW);
    }

    // Eteindre complètement l'écran
    u8g2.setPowerSave(1);

  } else {

    systemOff = false;

    // =================================================
    // PA5 = LUMINOSITE
    // =================================================

    uint16_t potBrightness = analogRead(brightnessPin);

    // LED : 0 à 100 %
    ledBrightness = map(
      potBrightness,
      0,
      4095,
      0,
      100
    );

    // =================================================
    // ECRAN
    // =================================================

    // 0 à 5 % = écran éteint
    if (potBrightness < 205) {

      u8g2.setPowerSave(1);

    } else {

      u8g2.setPowerSave(0);

      // 5 à 100 % = contraste 0 à 255
      uint8_t screenContrast = map(
        potBrightness,
        205,
        4095,
        0,
        255
      );

      u8g2.setContrast(screenContrast);
    }

    // =================================================
    // ECRAN
    // =================================================

    if (potBrightness >= 205) {

      u8g2.firstPage();

      do {

        u8g2.setFont(u8g2_font_helvB18_tf);

        u8g2.drawStr(50, 27, "F");
        u8g2.drawStr(50, 59, "C");

        u8g2.drawStr(90, 27, "60");
        u8g2.drawStr(90, 59, "30");

      } while (u8g2.nextPage());
    }
  }

  // ===================================================
  // PA6 = POTENTIOMETRE 2
  // ===================================================

  uint16_t pot2Value = analogRead(pot2Pin);

  // ===================================================
  // 8 BOUTONS
  // ===================================================

  for (uint8_t i = 0; i < nbButtons; i++) {

    bool buttonState = !digitalRead(buttonPins[i]);

    // Fonctions à ajouter plus tard
  }
}