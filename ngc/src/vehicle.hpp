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



#pragma once
#include "raylib.h"
#include "../../common_code/src/utility.hpp"
#include "sensor_emulator.hpp"


class Env_Emulator; // we be friends, them and I

class Turret
{
  public:
    Turret();

    void setYaw(float yaw_angle);
    void setPitch(float pitch_angle);

    float getYaw() const;
    float getPitch() const;

    BB3D getYawBox() const;
    BB3D getPitchBox() const;

    BB3D getCameraWideBox() const;
    BB3D getCameraNarrowBox() const;

    BB3D getLaserBox() const;

    bool getLaserStatus() const;    // true for laser active
    void setLaserStatus( bool );    // true for laser active

    static constexpr int camera_wide_resolution_x = 1920;
    static constexpr int camera_wide_resolution_y = 1080;
    static constexpr int camera_narrow_resolution_x = 1920;
    static constexpr int camera_narrow_resolution_y = 1080;

  private:
    // TODO
    // 3d bounding box has position and rotation information
    // these values are relative to the vehicle and follows our coordinate system conventions
    // (see utility.hpp)
    // you could use this to define camera and laser marker position and oriantations
    // BB3D getCameraBB() <-- example, this includes both the relative position and angles (quaternio)
    // BB3D getLaserBB()
    // or:
    // Vector3 getCameraVector() <-- example, returns the unit vector of camera direction
    // Vector3 getLaserVector()
    // start with implementing camera first, then ill ask you to add the laser
    // we should work with the mechanical design team and get the geometric dimensions and
    // axis positions from them.

    BB3D m_yaw_box;  // rotates around yaw axis, is on top of vehicle chasis
    BB3D m_pitch_box;   // rotates around pitch axis, is on top of m_yaw_box
    BB3D m_camera_wide_box;   // BB of general purpose camera, is on top of m_pitch_box
    BB3D m_camera_narrow_box;   // narrow field of view camera, used for fine adjustment of turret aim
    BB3D m_laser_box;

    bool m_laser_active = false;

    float m_yaw = 0.f;      // radians, 0 is dead ahead, positive towards port
    float m_pitch = 0.f;    // radians, 0 is dead ahead, positive towards up
};

// in the final product this class may evolve into "VehicleManager"
class Vehicle
{
  public:
    Vehicle();  // constructor
    Vehicle( const Vehicle& );
    // TODO Vehicle& operator =( const Vehicle& );

    BB3D getBox() const;    // returns 3d bounding box of vehicle
    v3f getPos() const;
    v3f getVel() const;
    v3f getAcc() const;
    v3f getAngVel() const;
    v3f getAngAcc() const;

    // Returns a reference to the turret (modifiable)
    Turret& getTurret();

    // Returns a const reference to the turret (read-only)
    const Turret& getTurret() const;

    Sensor_Data getSensorData() const;

    Sensor_Emulator getSensor( const unsigned short int number ) const;
    float readSensor( const unsigned short int number );

    void overridePos( const v3f ); // set position of vehicle

    friend Env_Emulator;

    // NOTE: this method is only to be called from NGC code
    void setNavigationState( const int time_step_milli ); // does dead reckoning using internal sensor data
    
    static constexpr float wheelbase = 1.3f;
    static constexpr float track = 1.0f;
    static constexpr float wheel_dia = 0.5f;
    static constexpr float wheel_width = 0.25f;
    static constexpr unsigned short int m_num_sensors = 8;

  private:
    // unless otherwise specified, all 3d vectors are:
    // +X:starboard, +Y:bow, +Z:above, (or width, len, height) (metre)
    // +X:pitch up, +Y:roll right, +Z:yaw left      (radian)

    // sensors on vehicle
    Sensor_Emulator m_sensor[ m_num_sensors ];
    Sensor_Data m_sensor_data;  // a packet containing data for all sensors

    // vehicle geometrical properties, (see utility.hpp)
    BB3D m_bb3d;

    // vehicle physical properties
    // NOTE: in reality we can only measure these with sensors, there will be errors
    v3f m_vel;     // velocity vector in local csys, (m/s)
    v3f m_acc;     // acceleration vector in local csys, (m/s^2)
    v3f m_angVel;  // angular velocity in local csys, (rad/s)
    v3f m_angAcc;  // angular acceleration in local csys (rad/s^2)
    float m_turn_radius = 0; // radius of turn circle, positive is towards port, 0 is dead ahead (m)
    float m_mass = 10;          // vehicle mass (kg)
    //v3f m_moment_of_inertia;
    //v3f m_center_of_mass;

    // Turret component attached to the vehicle
    Turret m_turret;

};

