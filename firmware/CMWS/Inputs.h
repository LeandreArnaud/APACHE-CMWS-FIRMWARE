#pragma once

#include <Arduino.h>

#include "PanelState.h"

// =====================================================================
// Input acquisition
//
// Reads the 8 switches, the 3-position mode switch and the 2 pots, and
// publishes them as a PanelInputs. Debouncing and ADC smoothing happen
// here so that no other module ever sees a raw, bouncing reading.
// =====================================================================

namespace Inputs {

// Configures the pins and primes the filters with a first reading, so
// the pots do not ramp up from zero after power-on.
void begin();

// Samples the inputs, at most once every cfg::INPUT_PERIOD_MS.
// Call it on every loop iteration; it decides for itself whether the
// period has elapsed. When it does not sample, the "changed" flags of
// the published state are cleared, so an edge is reported exactly once.
void poll(uint32_t now);

const PanelInputs& state();

}  // namespace Inputs
