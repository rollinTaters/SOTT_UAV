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
    m_font = LoadFontEx("./assets/fonts/arial.ttf", 20, 0, NULL);
    if (m_font.texture.id == 0) {
        std::cerr << "Failed to load font!" << std::endl;
    }

    switch (gt) {
        case type_compass:
            m_max_value = 360; m_min_value = 0; m_value = 0;
            // Center positions
            compass_bg_pos = { m_pos.x + m_dia/2, m_pos.y + m_dia/2 };
            compass_ticks_pos = compass_bg_pos;
            return; 

        case type_temperature:
            m_label_text = "temp C: ";
            m_max_value = 120; m_min_value = -5; m_value = -5;
            break;

        case type_amp:
            m_label_text = "current A: ";
            m_max_value = 40; m_min_value = 0; m_value = 0;
            break;
        case type_speedometer:
            m_label_text = "km/h: ";
            m_max_value = 40; m_min_value = 0; m_value = 0;
            break;
        case type_tachometer:
            m_label_text = "RPM: ";
            m_max_value = 40; m_min_value = 0; m_value = 0;
            break;
        case type_battery:
            m_label_text = "Battery %: ";
            m_max_value = 100; m_min_value = 0; m_value = 100;
            break;
        case type_signal:
            m_label_text = "Signal Power: ";
            m_max_value = 100; m_min_value = 0; m_value = 100;
            break;
        case type_steering_wheel:
            m_label_text = "Steering: ";
            m_max_value = 90; m_min_value = -90; m_value = 0;  // -540 to +540 degrees
            break;
        default: 
            m_label_text = "value: ";
            m_max_value = 120; m_min_value = -10; m_value = 0;
            break;
    }

    // Needle angle range
    m_needle_min_degree = -150;
    m_needle_max_degree = 150;
    // Precompute ticks and labels
    for (int i = 0; i <= 20; ++i) {
        float angleDeg = m_needle_min_degree + i * (m_needle_max_degree - m_needle_min_degree) / 20.0f;
        float angleRad = angleDeg * PI / 180.0f;
        Vector2 center = { m_pos.x + m_dia/2, m_pos.y + m_dia/2 };
        float outerR = m_dia/2;
        float innerR = (i % 2 == 0) ? outerR - 20 : outerR - 10;
        // Tick endpoints
        Vector2 outer = { center.x + outerR * cosf(angleRad), center.y + outerR * sinf(angleRad) };
        Vector2 inner = { center.x + innerR * cosf(angleRad), center.y + innerR * sinf(angleRad) };
        m_ticks.push_back(outer);
        m_ticks.push_back(inner);
        // Labels every other tick
        if (i % 2 == 0) {
            std::string lbl;
            if (gt == type_temperature) {
                float v = i * (m_max_value - m_min_value) / 20.0f + m_min_value;
                lbl = TextFormat("%g", v);
            } else if (gt == type_amp || gt == type_tachometer) {
                lbl = TextFormat("%d", i * 2);
            } else if (gt == type_speedometer){
                lbl = TextFormat("%d", i * 2 );
            } else {
                lbl = TextFormat("%d", i * 10);
            }
            m_numbers.push_back(lbl);
        }
    }

    // Setup frame and needle
        // Needle color
    m_needle_color = RED;
        // Center of gauge for drawing
    m_center = { m_pos.x + m_dia/2, m_pos.y + m_dia/2 };

    // Label position and style
    m_label_pos       = { (m_pos.x + m_dia/2 ) - 40 , m_pos.y + m_dia + 10 };
    m_label_font_size = 15;
    m_label_color     = BLACK;
}

void Gauge::init()
{
    if( m_type == type_compass )
    {
        // Load compass textures
        compass_bg = LoadTexture("assets/compass/compass_bg_200.png");
        compass_ticks_numbers = LoadTexture("assets/compass/compass_ticks_numbers_200.png");
    }
    else if( m_type == type_steering_wheel){
        steering_wheel = LoadTexture("./assets/steering_wheel.png");
    }
}

void Gauge::updateVal(const float value) {
    m_value = value;
}

void Gauge::updateProportionalVal(const float value) {
    m_value = value * (m_max_value - m_min_value) + m_min_value;
}


void Gauge::render() {
    // Load font only once when first needed
    if (!m_font_loaded) {
        m_font = LoadFontEx("./assets/fonts/arial.ttf", 20, 0, NULL);
        m_font_loaded = true;
    }

    if (m_type == type_compass) {
        // Draw compass background and rotating layer
        Rectangle src = { 0, 0, (float)compass_bg.width, (float)compass_bg.height };
        Rectangle dst = { m_pos.x, m_pos.y, m_dia, m_dia };
        Vector2 origin = { 0, 0 };
        // Use Pro to handle rotation around center
        DrawTexturePro(compass_bg, src, dst, { m_dia/2, m_dia/2 }, 0.0f, WHITE);
        DrawTexturePro(compass_ticks_numbers, src, dst, { m_dia/2, m_dia/2 }, m_value, WHITE);
        return;
    }
    else if(m_type == type_steering_wheel){
        Rectangle src = {0,0, (float)steering_wheel.width,(float)steering_wheel.height};
        Rectangle dst = {m_pos.x, m_pos.y, m_dia, m_dia };
        Vector2 origin = {m_dia/2,m_dia/2};
        char buf[32];
        // Show absolute value with direction indicator (R for right turn, L for left turn, nothing for center)
        const char* direction = m_value > 0 ? "L" : (m_value < 0 ? "R" : "");
        snprintf(buf, sizeof(buf), "%s%s %.1f", m_label_text.c_str(), direction, std::abs(m_value));
        Vector2 lblSize = MeasureTextEx(m_font, buf, m_label_font_size, 1);
        Vector2 lblPos = { m_pos.x- m_pos.x/20, m_pos.y + m_dia/2 };
        DrawTextEx(m_font, buf, lblPos, m_label_font_size, 1, m_label_color);
        // Negative value for clockwise rotation (right turn), positive for counter-clockwise (left turn)
        DrawTexturePro(steering_wheel, src, dst, origin, -m_value, WHITE);
    }
    else if(m_type == type_battery) {
        const int margin = 5;
        int maxInnerW = m_dia - margin*2;
        float pct = m_value / 100.0f;
        int currentPercent = int(pct * maxInnerW);

        // drawnings
        DrawRectangle(m_pos.x,           m_pos.y,         m_dia,       m_dia/2,       BLACK);
        DrawRectangle(m_pos.x+margin,    m_pos.y+margin,  currentPercent, m_dia/2 - margin*2, WHITE);
        // draw battery value
        char buf[32];
        snprintf(buf, sizeof(buf), "%s%.2f", m_label_text.c_str(), m_value);
        Vector2 lblSize = MeasureTextEx(m_font, buf, m_label_font_size, 1);
        Vector2 lblPos = { m_pos.x/2, m_label_pos.y - m_dia/2 };
        DrawTextEx(m_font, buf, lblPos, m_label_font_size, 1, m_label_color);
    }
    else if (m_type == type_signal) {
        int levels = 4; // Number of bars
        int barWidth = m_dia / 10; 
        int spacing = m_dia / 20;  
        int maxBarHeight = m_dia / 2; 
    
        // Drawing bars
        for (int i = 0; i < levels; i++) {
            int barHeight = (m_dia / 8) + i * (maxBarHeight / levels); 
            int xOffset = m_pos.x + i * (barWidth + spacing); 
            int yOffset = m_pos.y - barHeight;
    
            // Determine the fill percentage for each bar
            float fillRatio = (m_value - (i * 25)) / 25.0f; // Ratio for the current bar
            fillRatio = (fillRatio > 1.0f) ? 1.0f : ((fillRatio < 0.0f) ? 0.0f : fillRatio); // Clamp between 0 and 1
    
            // Calculate filled and empty bar sections
            int filledHeight = barHeight * fillRatio;
            int emptyHeight = barHeight - filledHeight;
    
            // Draw filled and empty sections of the bar
            DrawRectangle(xOffset, yOffset + emptyHeight, barWidth, filledHeight, GREEN);  // Filled part
            DrawRectangle(xOffset, yOffset, barWidth, emptyHeight, LIGHTGRAY);            // Empty part
        }
    
        // Display "Signal Power" text below the bars

        DrawTextEx(m_font,TextFormat("Signal Power: %.0f%%", m_value), {m_pos.x, m_pos.y + maxBarHeight / 2 - 15}, m_label_font_size, 1, BLACK);
    }
    
    else{
    // Draw gauge frame
    DrawCircleLines((int)m_center.x, (int)m_center.y, m_dia/2, BLACK);
    DrawCircle((int)m_center.x, (int)m_center.y, m_dia/2 - 5, WHITE);

    // Draw ticks
    for (size_t i = 0; i < m_ticks.size(); i += 2) {
        DrawLineV(m_ticks[i], m_ticks[i+1], BLACK);
    }

    // Draw numbers
    for (int idx = 0; idx < (int)m_numbers.size(); ++idx) {
        int i = idx * 2;
        float angleDeg = m_needle_min_degree + i * (m_needle_max_degree - m_needle_min_degree) / 20.0f;
        float angleRad = angleDeg * PI / 180.0f;
        Vector2 center = { m_pos.x + m_dia/2, m_pos.y + m_dia/2 };
        float textR = (m_dia/2) - 30;
        Vector2 basePos = { center.x + textR * cosf(angleRad), center.y + textR * sinf(angleRad) };
        // Center text
        Vector2 size = MeasureTextEx(m_font, m_numbers[idx].c_str(), m_label_font_size, 1);
        Vector2 txtPos = { basePos.x - size.x/2, basePos.y - size.y/2 };
        //Color col = (i <= 6) ? BLACK : (i <= 10) ? YELLOW : RED;
        Color col = BLACK;

        DrawRing(m_center, m_dia/2 - 55, 40, 30, -30, 0 ,Fade(YELLOW,0.5f));
        DrawRing(m_center, m_dia/2 - 55, 40, 30, 150, 0 ,Fade(RED,0.5f));
        DrawTextEx(m_font, m_numbers[idx].c_str(), txtPos, m_label_font_size, 1, col);
    }

    // Draw needle
    float needleAngle = m_needle_min_degree +
        (m_value - m_min_value) * (m_needle_max_degree - m_needle_min_degree) /
        (m_max_value - m_min_value);
    Rectangle rec = { m_center.x, m_center.y - 1.5f, m_dia/2 -20.f, 3 };
    DrawRectanglePro(rec, { 0, 1.5f }, needleAngle, m_needle_color);

    // Draw center dot
    DrawCircle((int)m_center.x, (int)m_center.y, 5, BLACK);

    // Draw label
    char buf[32];
    snprintf(buf, sizeof(buf), "%s%.2f", m_label_text.c_str(), m_value);
    Vector2 lblSize = MeasureTextEx(m_font, buf, m_label_font_size, 1);
    Vector2 lblPos = { m_pos.x + m_dia/2 - lblSize.x/2, m_label_pos.y };
    DrawTextEx(m_font, buf, m_label_pos, m_label_font_size, 1, m_label_color);
    }
}

///// Gauge end /////
