#pragma once

#include <Arduino.h>

#include "PanelState.h"

// =====================================================================
// Panel -> host: turns PanelInputs into the joystick report.
//
// One button per physical contact, pressed while that contact is
// closed, numbered as the host sees them (1-based):
//
//    1 ARM        2 SAFE        (lever 1)
//    3 CMWS       4 NAV         (lever 2)
//    5 BYPASS     6 AUTO        (lever 3)
//    7 JETTISON   8 JETT. OFF   (lever 4)
//    9 OFF       10 ON     11 TEST   (rotary)
//
// Exactly one button of each lever is pressed at rest; none while the
// lever travels between contacts. The lever table lives in Config.h.
//
// Axes: X = LAMP pot (PA5), Y = AUDIO pot (PA6), 0..4095.
// =====================================================================

namespace UsbGamepad {

void begin();

// Builds the report from the current inputs and sends it when it
// differs from the last one sent, or when the heartbeat interval has
// elapsed. Cheap enough to call on every loop iteration.
void update(const PanelInputs& in, uint32_t now);

}  // namespace UsbGamepad
