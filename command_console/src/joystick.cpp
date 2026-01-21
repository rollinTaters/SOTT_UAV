#include "joystick.hpp"
#include "raylib.h" // DEBUG
#include <iostream> // DEBUG
#include <cmath>

struct Vec2 {
    float x;
    float y;
};

Vec2 ProcessStickPolar(
    float rawX,
    float rawY,
    float deadzone,     // 0.0 – 1.0
    float curve,        // 1.0 = linear, >1 = softer center
    float smoothing,    // 0.0 = none, ~0.1–0.3 typical
    Vec2 &prev          // previous smoothed output
)
{
    // 1) Compute magnitude
    float mag = std::sqrt(rawX * rawX + rawY * rawY);

    // Inside deadzone → zero output
    if (mag < deadzone) {
        prev.x = 0.0f;
        prev.y = 0.0f;
        return {0.0f, 0.0f};
    }

    // 2) Normalize direction
    float nx = rawX / mag;
    float ny = rawY / mag;

    // 3) Rescale magnitude to [0..1] after deadzone
    float scaledMag = (mag - deadzone) / (1.0f - deadzone);
    if (scaledMag > 1.0f) scaledMag = 1.0f;

    // 4) Apply curve (exponential response)
    if (curve != 1.0f) {
        scaledMag = std::pow(scaledMag, curve);
    }

    // 5) Reconstruct vector
    Vec2 out;
    out.x = nx * scaledMag;
    out.y = ny * scaledMag;

    // 6) Smooth (LERP)
    out.x = (1.0f - smoothing) * out.x + smoothing * prev.x;
    out.y = (1.0f - smoothing) * out.y + smoothing * prev.y;

    prev = out;
    return out;
}


// Takes raw axis input in [-1, 1]
float ProcessAxis(
    float raw,
    float deadzone = 0.1f,   // Deadzone radius (0–1)
    float curve = 1.0f,      // Exponent: 1 = linear, >1 = more aggressive
    float smoothing = 0.5f,  // Lerp factor [0: no smoothing, 1: max smoothing]
    float &prevValue = *(new float(0.0f)) // Keep previous for smoothing
)
{
    // 1) Deadzone
    float absRaw = std::fabs(raw);
    if (absRaw < deadzone) {
        raw = 0.0f;
    } else {
        // Rescale so movement starts at 0 at the edge of deadzone
        float sign = (raw > 0.0f ? 1.0f : -1.0f);
        raw = sign * ((absRaw - deadzone) / (1.0f - deadzone));
    }

    // 2) Curve – exponent for sensitivity shaping
    // curve > 1 makes it more sensitive around center
    if (curve != 1.0f) {
        raw = (raw >= 0.0f)
              ? std::pow(raw, curve)
              : -std::pow(-raw, curve);
    }

    // 3) Smoothing – linear interpolation
    float result = (1.0f - smoothing) * raw + smoothing * prevValue;
    prevValue = result;

    return result;
}


Joystick::Joystick( int jn ): joy_number(jn)
{
    sdl_state = init( jn );
}

Joystick::~Joystick()
{
    // maybe?
    // if( sdl_state == 0 )
    SDL_JoystickClose(joy);
    SDL_Quit();
}

int Joystick::init( int jn )
{
    // --- Init SDL2 only for joystick ---
    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        std::cout << "SDL2 Init Error: " << SDL_GetError() << "\n";
        return 1;
    }

    // Check and open joystick 0
    if (SDL_NumJoysticks() < 1) {
        std::cout << "No joystick found!\n";
        return 2;
    }

    joy = SDL_JoystickOpen( jn );

    if (!joy) {
        std::cout << "Could not open joystick: " << SDL_GetError() << "\n";
        return 3;
    }
    SDL_JoystickUpdate();
    axes = SDL_JoystickNumAxes(joy);
    btnCount = SDL_JoystickNumButtons(joy);
    return 0;
}

void Joystick::update()
{
    // Poll SDL events
    SDL_JoystickUpdate();

    // Read axes (normalized -32768 to 32767)
    for (int i = 0; i < axes; i++) {
        axisValuesRaw[i] = SDL_JoystickGetAxis(joy, i);
        axisValues[i] = axisValuesRaw[i] / 32768.0d;
    }

    // Read buttons
    for (int i = 0; i < btnCount; i++) {
        btnPrevState[i] = btnPressed[i];
        btnPressed[i] = SDL_JoystickGetButton(joy, i);
    }
}

void Joystick::getControlInputs( ControlSignal& cs ) const
{
    if( sdl_state != 0 ) return;

    static Vec2 prevStick = {0.f, 0.f};
    static float prevRudder = 0.f;
    static float prevThrottle = 0.f;

    Vec2 stick = ProcessStickPolar(
            axisValues[0],  // aileron ( stick x axis )
            -axisValues[1], // elevator( stick y axis )
            0.08f,  // deadzone (%)
            1.8f,   // curve (soft center, strong edge)
            0.2f,   // smoothing
            prevStick );

    float rudd = ProcessAxis( -axisValues[2], 0.08f, 1.8f, 0.2f, prevRudder );
    float thro = ProcessAxis( -axisValues[3], 0.08f, 1.8f, 0.2f, prevThrottle );

    cs.throttle = thro;
    cs.elevator = stick.y;
    cs.aileron  = stick.x;
    cs.rudder   = rudd;
}

bool Joystick::buttonReleased( int button ) const
{
    if( btnPrevState[button] == true && btnPressed[button] == false )
        return true;
    return false;
}

void Joystick::draw_debug()
{
        // --- Draw with raylib ---
        // Show axes
        for (int i = 0; i < axes; i++) {
            DrawText(
                TextFormat("Axis %d: %.2f, %.3f", i, axisValues[i], axisValuesRaw[i]),
                300, 20 + i * 25, 20, BLACK
            );
        }

        // Show buttons
        for (int i = 0; i < btnCount; i++) {
            if (btnPressed[i]) {
                DrawText(
                    TextFormat("Button %d DOWN", i),
                    600, 20 + i * 25, 20, RED
                );
            }
        }

        DrawText("Use your joystick!", 300, 500, 20, BLUE);
}

