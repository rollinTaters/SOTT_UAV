#pragma once

struct SensorData
{
    float airspeed = 0.f;   // m/s
    float qw{1.f},qx{0.f},qy{0.f},qz{0.f};
    // angular rate?
    // magnetic north?
    // linear acceleration? (g-forces / wing loading)
    float temperature = 0.f;    // C
};

