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
#include "raymath.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <string>

class Gauge {
public:
    enum gauge_type {
        type_temperature,
        type_amp,
        type_adi,
        type_speedometer,
        type_tachometer,
        type_battery,
        type_signal,
    };

    Gauge(gauge_type type, Vector2 pos, float dia);
    ~Gauge() { }  // No need to unload font as it's static

    void updateVal(const float value); // input is value to be displayed
    void updateProportionalVal(const float value); // input is from 0.f to 1.f
    void render();
    void render(Vector2 offset);

private:
    static Font m_font;  // Static font shared by all gauges
    static bool m_font_loaded;  // Flag to track if font is loaded

    gauge_type m_type; // to store gauge type

    Vector2 m_pos; // Top left corner
    float m_dia;   // Diameter in pixels

    std::vector<Vector2> m_ticks; // Tick positions
    std::vector<std::string> m_numbers; // Numbers in gauge

    float m_max_value;
    float m_min_value;
    float m_value;

    float m_needle_min_degree; // Angle at which needle rests when min
    float m_needle_max_degree;

    float m_red_start_value;
    float m_red_end_value;
    float m_red_start_ang;
    float m_red_end_ang;

    float m_yellow_start_value;
    float m_yellow_end_value;
    float m_yellow_start_ang;
    float m_yellow_end_ang;

    float m_green_start_value;
    float m_green_end_value;
    float m_green_start_ang;
    float m_green_end_ang;

    Vector2 m_center;   // Center of the gauge
    Vector2 m_needle;   // Needle end position
    Color m_needle_color;
    std::string m_label_text; // Label text (temperature/ampere/etc.)
    Vector2 m_label_pos; // Label position
    float m_label_font_size;
    Color m_label_color;
};

