# CMWS panel firmware

Firmware for a replica of the AH-64 CMWS (countermeasures) control panel.
The panel is a USB joystick: its levers, rotary and pots reach the flight
sim as buttons and axes.

Board: **Blue Pill STM32F103C8T6** (or a Hangshun HK32F103CBT6, see below).

```
firmware/CMWS/      the firmware (Arduino sketch)
legacy/CMWS_V0.1/   the original single-file sketch, kept as a reference. Never modify it.
```

Every pin and every tuning constant lives in `Config.h`, including the
table that says which lever contact is wired to which input.

## Building

Arduino IDE 2.x with the STM32duino core 3.0.0 (`STMicroelectronics:stm32`)
and the U8g2 library.

- Board: **Generic STM32F1 series** → **BluePill F103C8**
- Upload method: **STM32CubeProgrammer (SWD)**
- USB support: see the next section. This setting is not stored in the
  repo; the sketch will not build if the wrong one is chosen.

Same thing from the command line, using the CLI bundled with the IDE:

```sh
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" --config-file ~/.arduinoIDE/arduino-cli.yaml \
       compile --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8 \
       --warnings all firmware/CMWS
```

Add `,usb=HID` to the FQBN for the HAL backend. Both builds must stay
warning-free.

`build_opt.h` in the sketch folder holds extra compiler flags (the core
picks it up automatically). Do not put `#define`s in it.

## Two USB backends, one API

`UsbHidJoystick.h` is the API (`begin`, `service`, `configured`, `send`).
Two files implement it; the IDE's **USB support** menu chooses:

| USB support menu | File | When |
|---|---|---|
| HID (keyboard and mouse) | `UsbHidJoystick.cpp` | genuine STM32F103. Our joystick class on top of the core's HAL USB stack. |
| None | `UsbBareMetal.cpp` | Blue Pills fitted with a **Hangshun HK32F103** (non-A). A small bare-metal stack driving the peripheral directly. |

Never include `<Keyboard.h>` or `<Mouse.h>`: they would re-initialise
the single USB peripheral. Never use `Servo` or `tone()`: they claim TIM2
and TIM3, and TIM2 drives the LED PWM.

### About the HK32F103 clone

Many cheap Blue Pills carry an `HK32F103CBT6` instead of an STM32. Its
core and peripherals are compatible, its USB block is not quite:

1. local packet-memory addresses 0..63 are registers (the buffer
   descriptor table), not RAM: packet buffers must live at 0x40 or above;
2. after every SETUP, EP0's TX side must be forced to NAK before the
   answer is loaded, or the first GET_DESCRIPTOR fails;
3. DADDR.EF must be set before USB interrupts are unmasked, or RESET is
   asserted forever.

The bare-metal backend does all three. The HAL backend does 1 and 3 and
cannot do 2, so on that chip it stalls after SET_ADDRESS: use the
"None" menu there. Both backends also carry a guard that shuts USB down
and retries every 2 s if a reset storm ever starves the CPU, so a bad
USB never freezes the screen.
