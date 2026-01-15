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

#include "gauge.hpp"
#include "flight_instruments.hpp"
#include "raylib.h"
#include "user_input.hpp"
#include <cassert>

namespace cg {

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


struct FlightInstrumentsPanel
{
    Vector2 pos{930,50};
    Vector2 size{460,780};

    ADI horizon = ADI({230, 180}, 150);
    HeadingIndicator compass = HeadingIndicator({230, 560}, 165);
    Altimeter altimeter = Altimeter({380, 30, 80, 300});
    VerticalSpeedIndicator vsi = VerticalSpeedIndicator({100,210}, 100);
    AirspeedIndicator airspeed = AirspeedIndicator({0, 30, 80, 300});

    void render()
    {
        createPanel( pos, size, DARKGRAY, "Flight Instruments" );
        horizon.render( pos );
        compass.render( pos );
        altimeter.render( pos );
        airspeed.render( pos );
        vsi.render( pos );
    }
};

struct EngineeringPanel
{
    Vector2 pos{1420,50};
    Vector2 size{400,600};

    Gauge temp1 = Gauge( Gauge::type_temperature, { 15, 55}, 170 );
    Gauge temp2 = Gauge( Gauge::type_temperature, { 15,230}, 170 );
    Gauge amp1 = Gauge( Gauge::type_amp,          {225, 55}, 170 );
    Gauge amp2 = Gauge( Gauge::type_amp,          {225,230}, 170 );
    Gauge battery = Gauge(Gauge::type_battery,    { 30,400}, 150 );
    Gauge signal  = Gauge(Gauge::type_signal,     {200,400}, 150 );

    void render()
    {
        createPanel( pos, size, DARKGRAY, "Engineering Panel" );
        temp1.render( pos );
        temp2.render( pos );
        amp1.render( pos );
        amp2.render( pos );
        battery.render( pos );
        signal.render( pos );
    }
};

//extern Gauge gauge_compass;
extern Gauge gauge_tachometer;
extern Gauge gauge_speed;

extern FlightInstrumentsPanel FI_panel;
extern EngineeringPanel E_panel;

extern UserInput input;

} // namespace cg
