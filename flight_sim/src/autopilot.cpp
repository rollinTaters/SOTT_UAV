#include "aircraft.hpp"
#include "raymath.h"

namespace AP{

    enum AP_mode{
        OFF,    // off, meaning so as to be no longer continuing, operating or functioning; (she switched off the radio)
        ATTITUDE_HOLD,    // Stability Assist System, keeps you pointing where you pointing
        wip // TODO Work in Progress
    };

int mode = 0;

float theta_target = 0.05f;
float phi_target = 0.0f;

void run( State& state, ControlSignal& cs )
{
    if( mode == ATTITUDE_HOLD )
    {
        // PI control
        cs.elevator -= -0.5 * (state.theta - theta_target) - 0.2 * state.q;  // Pitch hold
        cs.aileron += -0.3 * (state.phi - phi_target) - 0.1 * state.p;  // Roll hold
        cs.rudder -= -0.2 * state.r;  // Yaw damper

        // clamp excessive auto pilot
        cs.elevator = Clamp( cs.elevator, -1.0f, 1.0f );
        cs.aileron = Clamp( cs.aileron, -1.0f, 1.0f );
        cs.rudder = Clamp( cs.rudder, -1.0f, 1.0f );
    } else {
        // nice day eh?
    }
}

}   // namespace AP
