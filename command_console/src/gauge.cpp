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


#include "gauge.hpp"
#include <raylib.h>

// Initialize static members
Font Gauge::m_font = {0};
bool Gauge::m_font_loaded = false;

Gauge::Gauge(gauge_type gt, Vector2 pos, float dia)
    : m_type(gt), m_pos(pos), m_dia(dia)
{
    // Load font once in constructor
    m_font = LoadFontEx("./assets/fonts/arial.ttf", 20, NULL, 0);
    if (m_font.texture.id == 0) {
        std::cerr << "Failed to load font!" << std::endl;
    }

    // Center of gauge for drawing
    m_center = { m_pos.x + m_dia/2, m_pos.y + m_dia/2 };

    switch (gt) {
        case type_temperature:
            m_label_text = "temp (C)";
            m_max_value = 130; m_min_value = -5; m_value = 0;
            m_needle_min_degree = -150; m_needle_max_degree = 150;
            m_red_start_value = 110; m_red_end_value = 130;
            m_yellow_start_value = 70; m_yellow_end_value = 110;
            m_green_start_value = 10; m_green_end_value = 35;
            break;
        case type_amp:
            m_label_text = "current (A)";
            m_max_value = 40; m_min_value = 0; m_value = 0;
            m_needle_min_degree = -150; m_needle_max_degree = 150;
            m_red_start_value = 35; m_red_end_value = 40;
            m_yellow_start_value = 30; m_yellow_end_value = 35;
            m_green_start_value = 0; m_green_end_value = 10;
            break;
        case type_speedometer:
            m_label_text = "km/h";
            m_max_value = 150; m_min_value = 0; m_value = 0;
            m_needle_min_degree = -220; m_needle_max_degree = 70;
            m_red_start_value = 0; m_red_end_value = 25;
            m_yellow_start_value = 50; m_yellow_end_value = 150;
            m_green_start_value = 35; m_green_end_value = 50;
            break;
        case type_tachometer:
            m_label_text = "RPM";
            m_max_value = 40; m_min_value = 0; m_value = 0;
            m_needle_min_degree = -150; m_needle_max_degree = 150;
            m_red_start_value = 110; m_red_end_value = 130;
            m_yellow_start_value = 70; m_yellow_end_value = 110;
            m_green_start_value = 10; m_green_end_value = 35;
            break;
        case type_battery:
            m_label_text = "Battery (V)";
            m_max_value = 100; m_min_value = 0; m_value = 100;
            m_needle_min_degree = -150; m_needle_max_degree = 150;
            m_red_start_value = 110; m_red_end_value = 130;
            m_yellow_start_value = 70; m_yellow_end_value = 110;
            m_green_start_value = 10; m_green_end_value = 35;
            break;
        case type_signal:
            m_label_text = "Signal Power";
            m_max_value = 100; m_min_value = 0; m_value = 100;
            m_needle_min_degree = -150; m_needle_max_degree = 150;
            m_red_start_value = 110; m_red_end_value = 130;
            m_yellow_start_value = 70; m_yellow_end_value = 110;
            m_green_start_value = 10; m_green_end_value = 35;
            break;
        default: 
            m_label_text = "value";
            m_max_value = 120; m_min_value = -30; m_value = 0;
            m_needle_min_degree = -150; m_needle_max_degree = 150;
            m_red_start_value = 100; m_red_end_value = 120;
            m_yellow_start_value = 70; m_yellow_end_value = 100;
            m_green_start_value = 40; m_green_end_value = 60;
            break;
    }

    // compute angles of bg status colors
    auto getang = [&](float a){
    return m_needle_min_degree +
        (a - m_min_value) *
        (m_needle_max_degree - m_needle_min_degree) / (m_max_value - m_min_value);
    };
    m_red_start_ang = getang( m_red_start_value );
    m_red_end_ang   = getang( m_red_end_value );
    m_yellow_start_ang = getang( m_yellow_start_value );
    m_yellow_end_ang   = getang( m_yellow_end_value );
    m_green_start_ang = getang( m_green_start_value );
    m_green_end_ang   = getang( m_green_end_value );


    // Precompute ticks and labels
    for (int i = 0; i <= 20; ++i) {
        float angleDeg = m_needle_min_degree + i * (m_needle_max_degree - m_needle_min_degree) / 20.0f;
        float angleRad = angleDeg * PI / 180.0f;
        float outerR = m_dia/2;
        float innerR = (i % 2 == 0) ? outerR - 20 : outerR - 10;
        // Tick endpoints
        Vector2 outer = { m_center.x +outerR * cosf(angleRad), m_center.y +outerR * sinf(angleRad) };
        Vector2 inner = { m_center.x +innerR * cosf(angleRad), m_center.y +innerR * sinf(angleRad) };
        m_ticks.push_back(outer);
        m_ticks.push_back(inner);
        // Labels every other tick
        if (i % 2 == 0) {
            std::string lbl;
            float v = i * (m_max_value - m_min_value) / 20.0f + m_min_value;
            lbl = TextFormat("%g", v);
            m_numbers.push_back(lbl);
        }
    }

    // Needle color
    m_needle_color = RED;

    // Label position and style
    m_label_pos       = {m_center.x -40, m_center.y +10};
    m_label_font_size = 15;
    m_label_color     = BLACK;
}

void Gauge::updateVal(const float value) {
    m_value = value;
}

void Gauge::updateProportionalVal(const float value) {
    m_value = value * (m_max_value - m_min_value) + m_min_value;
}


void Gauge::render() { render({0,0}); }

void Gauge::render( Vector2 offset ) {
    Vector2 center{ m_center.x + offset.x, m_center.y + offset.y };

    // Draw gauge frame
    DrawCircle((int)center.x, (int)center.y, m_dia/2 - 2, WHITE);
    DrawCircleLines((int)center.x, (int)center.y, m_dia/2, BLACK);

    // Draw colored status bar
    // center, innerR, outerR, startAng, endAng, segments, color
    DrawRing(center, m_dia/2 -15, m_dia/2 -7,
            m_red_start_ang, m_red_end_ang, 0 ,Fade(RED,0.5f));
    DrawRing(center, m_dia/2 -15, m_dia/2 -7,
            m_yellow_end_ang, m_yellow_start_ang, 0 ,Fade(YELLOW,0.5f));
    DrawRing(center, m_dia/2 -15, m_dia/2 -7,
            m_green_end_ang, m_green_start_ang, 0 ,Fade(GREEN,0.5f));

    // Draw ticks
    for (size_t i = 0; i < m_ticks.size(); i += 2) {
        DrawLineV(m_ticks[i]+offset, m_ticks[i+1]+offset, BLACK);
    }

    // Draw label
    DrawTextEx(
            m_font,
            m_label_text.c_str(),
            {m_label_pos.x+offset.x, m_label_pos.y+offset.y},
            m_label_font_size,
            1,
            m_label_color );

    // Draw numbers
    for (int idx = 0; idx < (int)m_numbers.size(); ++idx) {
        int i = idx * 2;
        float angleDeg = m_needle_min_degree + i * (m_needle_max_degree - m_needle_min_degree) / 20.0f;
        float angleRad = angleDeg * PI / 180.0f;
        float textR = (m_dia/2) - 30;
        Vector2 basePos = { center.x + textR * cosf(angleRad), center.y + textR * sinf(angleRad) };
        // Center text
        Vector2 size = MeasureTextEx(m_font, m_numbers[idx].c_str(), m_label_font_size, 1);
        Vector2 txtPos = { basePos.x - size.x/2, basePos.y - size.y/2 };
        //Color col = (i <= 6) ? BLACK : (i <= 10) ? YELLOW : RED;
        Color col = BLACK;

        DrawTextEx(m_font, m_numbers[idx].c_str(), txtPos, m_label_font_size, 1, col);
    }

    // Draw needle
    float needleAngle = m_needle_min_degree +
        (m_value - m_min_value) * (m_needle_max_degree - m_needle_min_degree) /
        (m_max_value - m_min_value);
    Rectangle rec = { center.x, center.y - 1.5f, m_dia/2 -20.f, 3 };
    DrawRectanglePro(rec, { 0, 1.5f }, needleAngle, m_needle_color);

    // Draw center dot
    DrawCircle((int)center.x, (int)center.y, 5, BLACK);
}

///// Gauge end /////
