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


#include <iostream> // cerr
#include "sensor_emulator.hpp"
#include "env_emulator.hpp" // getTime(), getSensorData()

Sensor_Emulator::Sensor_Emulator()
{
}

Sensor_Emulator::Sensor_Emulator( Sensor_Type type,
                                  const v3f inp_pos,
                                  const v3f inp_angle )
    :m_type(type)
{
    m_bb3d.setPos( inp_pos );
    m_bb3d.setAng( inp_angle );
    m_bb3d.setSize( 0.010f, 0.010f, 0.010f );
}

Sensor_Emulator::Sensor_Emulator( Sensor_Type type, const BB3D inp_bb3d )
    :m_type(type)
{
    m_bb3d = inp_bb3d;
}


float Sensor_Emulator::read( Sensor_Data& data )
{
    // we are a sensor emulator, there is no hardware to read
    // forward the call to the environment emulator
    return env_emulator.getSensorData(this, data );
}

Sensor_Type Sensor_Emulator::getType() const { return m_type; }
BB3D Sensor_Emulator::getBox() const { return m_bb3d; }
