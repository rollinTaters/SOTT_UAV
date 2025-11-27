/*
    MIT License

    Copyright (c) 2025 rollinTaters, guvenchemy

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once
#include "raylib.h"
#include <iostream>


class Adi {
public:
    Adi(Vector2 pos, float dia);
    void init();    // loads textures, call when opengl context is ready
    void updateRollVal_prop(const float);   // 0-1
    void updatePitchVal_prop(const float);  // 0-1
    void updateRollVal(const float);    // degree
    void updatePitchVal(const float);   // degree 
    void render();

private:
    // adi things
    Texture2D roll_markings;
    Texture2D pitch_scale;
    Texture2D horizon;
    Texture2D horizon_outline;

    Vector2 horizon_outline_pos;
    Vector2 roll_markings_pos;
    Vector2 pitch_scale_pos;
    Vector2 horizon_pos;

    Color background_color;

    float m_dia;        // diameter in pixels
    
    float m_max_roll_value;
    float m_min_roll_value;
    float m_roll_value;

    float m_max_pitch_value;
    float m_min_pitch_value;
    float m_pitch_value;
    const float m_pitch_scale = 0.5f; //FIXME this needs tuning

    Vector2 m_size;    // in pixels
    Vector2 m_pos;     // of top left corner    
};
