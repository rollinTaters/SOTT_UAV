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

/// Attitude Director Indicator /// 

#include "adi.hpp"

Adi::Adi(Vector2 pos, float dia) {
    m_dia = dia;
    m_pos = pos;

    m_roll_value = 0;
    m_max_roll_value = 180;
    m_min_roll_value = -180;

    m_pitch_value = 0;
    m_max_pitch_value = 100;
    m_min_pitch_value = -100;
}

void Adi::init()
{
    // Texture 
    roll_markings = LoadTexture("./assets/attitude_director_indicator/rm.png");
    if (roll_markings.id == 0) {
        std::cerr << "Error: adi class could not load Roll markings texture!" << std::endl;
    }

    pitch_scale = LoadTexture("./assets/attitude_director_indicator/pitch_scale_.png");
    if (pitch_scale.id == 0) {
        std::cerr << "Error: adi class could not load Pitch scale texture!" << std::endl;
    }

    horizon = LoadTexture("./assets/attitude_director_indicator/horizon.png");
    if (horizon.id == 0) {
        std::cerr << "Error: adi class could not load Horizon texture!" << std::endl;
    }
    
    horizon_outline = LoadTexture("./assets/attitude_director_indicator/horizon_outline.png");
    if (horizon_outline.id == 0) {
        std::cerr << "Error: adi class could not load Horizon Outline texture!" << std::endl;
    }
}

void Adi::updateRollVal_prop(const float prop) {
    m_roll_value = (prop * (m_max_roll_value - m_min_roll_value)) + m_min_roll_value;
}

void Adi::updatePitchVal_prop(const float prop) {
    m_pitch_value = (prop * (m_max_pitch_value - m_min_pitch_value)) + m_min_pitch_value;
}

void Adi::updateRollVal(const float val) {
    m_roll_value = val;
    if (m_roll_value < m_min_roll_value)
        m_roll_value = m_min_roll_value;
    if (m_roll_value > m_max_roll_value)
        m_roll_value = m_max_roll_value;
}

void Adi::updatePitchVal(const float val) {
    m_pitch_value = val;
    if (m_pitch_value < m_min_pitch_value)
        m_pitch_value = m_min_pitch_value;
    if (m_pitch_value > m_max_pitch_value)
        m_pitch_value = m_max_pitch_value;
}

void Adi::render() {

    Vector2 originalPosition = {m_pos.x, m_pos.y + m_dia / 2.f};
    float newX = originalPosition.x;
    float newY = originalPosition.y - m_pitch_value * m_pitch_scale;


    DrawTexturePro(horizon_outline,
                   {0, 0, (float)horizon_outline.width, (float)horizon_outline.height},
                   {m_pos.x, m_pos.y + m_dia / 2.f, m_dia, m_dia},
                   {m_dia / 2, m_dia / 2}, 0, WHITE);
    DrawTexturePro(horizon,
                   {0, 0, (float)horizon.width, (float)horizon.height},
                   {newX, newY, m_dia, m_dia},
                   {m_dia / 2, m_dia / 2}, m_roll_value, WHITE);
    DrawTexturePro(pitch_scale,
                   {0, 0, (float)pitch_scale.width, (float)pitch_scale.height},
                   {newX, newY, m_dia, m_dia},
                   {m_dia / 2, m_dia / 2}, m_roll_value, WHITE);

    int outerRadius = (m_dia / 2) + 300;
    int innerRadius = (m_dia / 2) - 26;
    // color rgb, opacity
    const Color color1 = {180,180,180,255};
    DrawRing({m_pos.x, m_pos.y + m_dia / 2.f}, innerRadius, outerRadius, 0, 360, 64, color1);
                   

    DrawTexturePro(roll_markings,
                   {0, 0, (float)roll_markings.width, (float)roll_markings.height},
                   {m_pos.x, m_pos.y + m_dia / 2.f, m_dia, m_dia},
                   {m_dia / 2, m_dia / 2}, 0, WHITE);

}
