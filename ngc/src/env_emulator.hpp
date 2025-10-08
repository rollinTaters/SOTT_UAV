/*
	MIT License

	Copyright (c) 2025 rollinTaters

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


/*
   environment emulator
   - course map (world map)
   - holds real vehicle pos, rotation
   - updates real vehicle pos from motion actuator emulator

   */

// stuff goes here i guess

#pragma once
#include <thread>
#include "raylib.h"
#include "vehicle.hpp"
//#include "sensor_emulator.hpp"  // getSensorData needs to know what a sensor is

class Sensor_Emulator;  // getSensorData needs to know what a sensor is
class NGC;  // we are gonna get the commanded speed and rate values from it

class Env_Emulator
{
  public:
    Env_Emulator( const Vehicle& );
    ~Env_Emulator();
    Env_Emulator( const Env_Emulator& )  = delete;
    Env_Emulator( const Env_Emulator&& ) = delete;

    // vehicle simulation
    bool startPhysSim();
    bool stopPhysSim();
    
    void setNGC( NGC * );
    Vehicle getRealVehicle() const;

    // returns time elapsed since emulator start
    // SWITCH TO CHRONO sf::Time getTime() const;

    // this populates the sensor data object with emulated sensor readings from the "real" vehicle
    bool getSensorData( Sensor_Emulator*, Sensor_Data& ) const; // returns false on read fail

    void drawHMap();
    bool setupModel();
    bool unloadModel();

  private:
    // "real" map/course
    Texture2D m_hm_texture;
    Mesh m_hm_mesh;
    Model m_hm_model;
    bool m_model_initialized = false;
    const float m_metre_per_pixel = 0.050;  // 50mm per pixel

    //float shittyPixelMarch( BB3D, v3f ) const;
    float useRaycast( v3f, v3f ) const;

    Matrix getWorldTransform( Vector3 offset ) const;   // for objects in world

    // physics simulation thread
    std::thread* m_phys_thread = nullptr;
    bool m_run_phys_thread = false;
    void physThreadFunc();

    // "real" vehicle, is copied from given vehicle on constructor method
    Vehicle m_real_vehicle;

    NGC * m_ngc = nullptr;

    // simulation clock
    std::chrono::steady_clock m_clock;

};

// we have a global environment emulator object, declared at main.cpp
extern Env_Emulator env_emulator;
