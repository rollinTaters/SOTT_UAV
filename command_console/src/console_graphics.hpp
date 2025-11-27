/*
        MIT License

        Copyright (c) 2025 rollinTaters

        Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
        furnished to do so, subject to the following conditions:

        The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.

        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
        FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
   THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
        LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

/*
    This is where we display data from the vehicle and issue control commands

    - DONE - vehicle attitude indicator

    - TODO vehicle control mode (manual, auto-drive, fully-auto)

    - TODO camera feeds

    - TODO turret attitude
    - TODO weapon control (target aquisition status, big red fire button)

    - MAYBE LATER - motor loading
    - TODO component temperatures
    - DONE - motor temperatures
    - DONE - land speed
    - DONE - direction of travel

    - TODO - overhead map
    - TODO - waypoints display

    - WILL BE MIGRATED FROM NGC GUI - immediate obstacle display


    */

#pragma once

#include "adi.hpp"
#include "gauge.hpp"
#include "raylib.h"
#include "user_input.hpp"
#include <cassert>

namespace cg {

extern Gauge gauge_temp;
extern Gauge gauge_amp;
extern Gauge gauge_amp2;
extern Gauge gauge_temp2;
extern Gauge gauge_speed;
extern Gauge gauge_compass;
extern Gauge gauge_tachometer;
extern Gauge gauge_battery;
extern Gauge gauge_signal;
extern Adi gauge_adi;
extern UserInput input;
extern Gauge steering_wheel;

void InitWindowSafe(int width, int height, const char *title);

void createPanel(Vector2 pos, Vector2 size, Color color, const char *title);

bool isInitialized();

void renderGauges();

inline void EnsureWindow() {
  assert(isInitialized() &&
         "Can't call cg::InitWindowSafe() before creating objects!");
}
// Debug test
void DEBUG_gauge_test();
} // namespace cg
