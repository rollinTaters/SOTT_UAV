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
   sensor emulator:
   - using the environment emulator;
   - models sensor read errors, angular and positional deviations
   - returns would be read value

   */

#pragma once
#include "../../common_code/src/utility.hpp"

#define LIDAR_POINTS 50

enum Sensor_Type{
    E_type_undefined,
    E_type_distance,
    E_type_LIDAR,
    E_type_IMU,
    E_type_temperature,
    E_type_current,
    E_type_turret_encoder
};

struct Sensor_Data
{
    // this is the last sensor type that modified the data
    Sensor_Type type;
    // TODO maybe we should add a "last update time" for each sensor in here
    
    // for distance sensor
    float distance;


    // for lidar sensor
    float lidar[LIDAR_POINTS];  // metre
    float lidar_angle[LIDAR_POINTS];    // radian
    //float lidar_quality[LIDAR_POINTS];

    // for IMU
    v3f acceleration;  // m/s^2
    v3f velocity;      // m/s   (i dont know what imu actually provides)
    v3f angular_rate;  // radian/sec
    v3f magnetic_north;    // unit vector
    float barometric_pressure;      // Pa

    // for LIDAR
    // TODO probably a point cloud

    // for temperature
    float temperature;  // C
    //float temperature_motor1;
    //float temperature_motor2;
    //float temperature_electronics;

    // for current sensor
    float electric_current; // amps
    //float electric_current_motor1;
    //float electric_current_motor2;
    //float electric_current_electronics;
    //float electric_current_turret??;


    // turret encoder sensor populates these two angles
    float turret_pitch = 0.0f;  // Current turret angle (radian)
    float turret_yaw = 0.0f;    // Current turret angle (radian)
};

class Sensor_Emulator
{
  public:
    Sensor_Emulator();  // DO NOT use, here to shut up the compiler

    Sensor_Emulator( Sensor_Type type,
                     const v3f inp_pos,
                     const v3f inp_angle );

    Sensor_Emulator( Sensor_Type type, const BB3D inp_bb3d );

    float read( Sensor_Data& );

    Sensor_Type getType() const;
    BB3D getBox() const;    // returns 3d bounding box of sensor

  private:

    // position info, relative to the vehicle
    BB3D m_bb3d;

    Sensor_Type m_type = E_type_undefined;

};
