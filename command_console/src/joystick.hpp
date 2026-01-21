#pragma once
#include <SDL2/SDL.h>
#include "aircraft.hpp" // ControlSignal

struct Joystick
{
    int joy_number = 0;
    int sdl_state = -1;

    SDL_Joystick* joy = nullptr;
    int axes = -1;
    int btnCount = -1;
    
    float axisValues[16] = {0};
    double axisValuesRaw[16] = {0};
    bool btnPressed[32] = {false};
    bool btnPrevState[32] = {false};



    int init( int jn );

    Joystick( int jn = 0 );

    ~Joystick();

    void update();

    void getControlInputs( ControlSignal& cs ) const;

    bool buttonReleased( int button ) const;

    void draw_debug();
};
