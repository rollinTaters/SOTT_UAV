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


#include <iostream>     // cerr
#include "vehicle.hpp"
#include "env_emulator.hpp" // read sensor

Turret::Turret()
    : m_yaw(0.0f), m_pitch(0.0f)
{
    // Position and size init is up to mechanical design, but here is a basic example:
    m_yaw_box.setPos({0.f, 0.f, 0.5f});  // position relative to vehicle
    m_yaw_box.setSize( 0.200f, 0.150f, 0.220f );
    m_yaw_box.setAng({0.f, 0.f, m_yaw}); 

    m_pitch_box.setPos({0.2f, 0.0f, 0.f});   // relative to yaw box
    m_pitch_box.setSize( 0.200f, 0.050f, 0.020f );
    m_pitch_box.setAng({m_pitch, 0.f, 0.f}); 

    m_camera_wide_box.setPos    (  0.023f,  0.008f,  0.045f );
    m_camera_wide_box.setSize   (  0.010f,  0.020f,  0.010f );
    m_camera_wide_box.setAng    (  0.000f,  0.000f,  0.000f );

    m_camera_narrow_box.setPos  ( -0.050f,  0.008f,  0.070f );
    m_camera_narrow_box.setSize (  0.010f,  0.095f,  0.020f );
    m_camera_narrow_box.setAng  (  0.000f,  0.000f,  0.000f );

    m_laser_box.setPos          ( -0.032f,  0.014f,  0.030f );;
    m_laser_box.setSize         (  0.025f,  0.040f,  0.025f );;
    m_laser_box.setAng          (  0.000f,  0.000f,  0.000f );;
}

Vehicle::Vehicle()
{
    // populate sensors on vehicle
    m_sensor[0] = Sensor_Emulator( E_type_IMU,      v3f( 0, 0, 0), v3f(0, 0, 0) );
    m_sensor[1] = Sensor_Emulator( E_type_LIDAR,    v3f( 0, 0, 0.7), v3f(0, 0, 0) );
    m_sensor[2] = Sensor_Emulator( E_type_distance, v3f(-0.20, 0,0), v3f(0,0,-50) );
    m_sensor[3] = Sensor_Emulator( E_type_distance, v3f(-0.10, 0,0), v3f(0,0,-20) );
    m_sensor[4] = Sensor_Emulator( E_type_distance, v3f( 0.00, 0,0), v3f(0,0,  0) );
    m_sensor[5] = Sensor_Emulator( E_type_distance, v3f( 0.10, 0,0), v3f(0,0, 20) );
    m_sensor[6] = Sensor_Emulator( E_type_distance, v3f( 0.20, 0,0), v3f(0,0, 50) );
    m_sensor[7] = Sensor_Emulator( E_type_turret_encoder, v3f(0,0,0), v3f(0,0,0) );

    m_vel = v3f( 0,0,0 );
    m_acc = v3f( 0,0,0 );

    m_bb3d.setSize(0.8, 1.3, 0.6);
    m_bb3d.setPos(0,0,0);
    m_bb3d.setAng(0,0,0);
}

Vehicle::Vehicle( const Vehicle& other )
{
    for( int i = 0; i < m_num_sensors; i++ )
    {
        m_sensor[i] = other.m_sensor[i];
    }
    m_bb3d = other.m_bb3d;
    m_vel = other.m_vel;
    m_acc = other.m_acc;
    m_turn_radius = other.m_turn_radius;
    m_mass = other.m_mass;
}

/*
Vehicle& Vehicle::operator =(const Vehicle& rhs)
{
    if (this != &rhs) {  // Kendisine atama yapılmadığını kontrol et
        // DONT. this is constexpr. this is hardcoded. m_num_sensors = rhs.m_num_sensors;
        // Diğer üyeleri kopyala (örneğin m_turret)
    }
    return *this;
}
*/
//arranged according to the directive
/* BUT THERE ARE STILL PROBLEMMM
Vehicle& Vehicle::operator=(const Vehicle& rhs)
{
    if (this != &rhs) 
    {
        // copy the sensors 
        for (int i = 0; i < m_num_sensors; i++)
        {
            m_sensor[i] = rhs.m_sensor[i];
        }

        // copy other members
        m_bb3d = rhs.m_bb3d;
        m_vel = rhs.m_vel;
        m_acc = rhs.m_acc;
        m_turn_radius = rhs.m_turn_radius;
        m_mass = rhs.m_mass;
        m_turret = rhs.m_turret; // <<<<<< 
        m_angVel = rhs.m_angVel;
        m_angAcc = rhs.m_angAcc;
        m_sensor_data = rhs.m_sensor_data;
    }
    return *this;
}
*/

BB3D Vehicle::getBox() const { return m_bb3d; }

v3f Vehicle::getPos() const { return m_bb3d.getPos(); }
v3f Vehicle::getVel() const { return m_vel; }
v3f Vehicle::getAcc() const { return m_acc; }
v3f Vehicle::getAngVel() const { return m_angVel; }
v3f Vehicle::getAngAcc() const { return m_angAcc; }

Sensor_Data Vehicle::getSensorData() const { return m_sensor_data; };

Sensor_Emulator Vehicle::getSensor( const unsigned short int number ) const
{
    if( number >= m_num_sensors )
    {
        std::cerr<<"ERROR! Requested non existing sensor, returning default constructed sensor.\n";
        return Sensor_Emulator();
    }
    return m_sensor[ number ];
}
/*
								/\/\
								  \_\  _..._
								  (" )(_..._)
								   ^^  // \\
*/

float Vehicle::readSensor( const unsigned short int number )
{
    if( number >= m_num_sensors )
    {
        std::cerr<<"ERROR! Requested read on non existing sensor, returning 0 as read value.\n";
        return 0.f;
    }
    return m_sensor[number].read( m_sensor_data );
}


void Vehicle::overridePos( const v3f pos ) { m_bb3d.setPos(pos); }

void Vehicle::setNavigationState( const int time_step_milli )
{
    // convert time step from milliseconds to seconds
    double time_step = time_step_milli/1000.f;

    // -- dead reckoning calculations --
    // this part ive yanked from the simulatePhys method.
    // this is using the eulers method for integration, we should switch to RungeKutta 4th order

    // set our acceleration values from imu sensor output
    m_acc = m_sensor_data.acceleration;
    // this line commented out because IMU outputs angular velocity???
    //m_angAcc = m_sensor_data.angular_rate;
    m_angVel = m_sensor_data.angular_rate;

    // translations
    m_bb3d.translateLocal( m_vel * time_step );

    // rotations, we should switch to quaternions...
    m_bb3d.yawLeft( m_angVel.z * time_step );
    m_bb3d.pitchUp( m_angVel.x * time_step );
    m_bb3d.rollRight( m_angVel.y * time_step );

    // first derivatives (using eulers method)
    m_vel += m_acc * (float)time_step;
    // this line commented out because IMU outputs angular velocity???
    //m_angVel += m_angAcc * time_step;

    // DEBUG
    m_vel = m_sensor_data.velocity; // DEBUG, does imu give this to us outright?? idk
    /*
    std::cout<<"dead reckon time step: "<<time_step<<"\n";
    std::cout<<"IMU acce: "<<m_acc.x<<"x "<<m_acc.y<<"y "<<m_acc.z<<"z\n";
    std::cout<<"IMU velo: "<<m_vel.x<<"x "<<m_vel.y<<"y "<<m_vel.z<<"z\n";
    std::cout<<"IMU rate: "<<m_angVel.x<<"x "<<m_angVel.y<<"y "<<m_angVel.z<<"z\n";
    */
}

// Vehicle sınıfının Turret nesnesiyle ilgili fonksiyonlarının implementasyonu

Turret& Vehicle::getTurret() {
    return m_turret;  // m_turret, Vehicle sınıfındaki Turret üyesi
}

const Turret& Vehicle::getTurret() const {
    return m_turret;
}

void Turret::setYaw(float yaw_angle) {
    m_yaw = yaw_angle;
    //m_yaw_box.setAng({0, 0, yaw_angle});
    m_yaw_box.setAng( {yaw_angle, 0, 0} );
}

// Pitch açısını ayarlayan fonksiyon
void Turret::setPitch(float pitch_angle) {
    m_pitch = pitch_angle;
    //m_pitch_box.setAng({pitch_angle, 0, 0});  // pitch is around X
    m_pitch_box.setAng( {0, pitch_angle, 0} );
}

// Yaw açısını döndüren fonksiyon
float Turret::getYaw() const {
    return m_yaw;
}

// Pitch açısını döndüren fonksiyon
float Turret::getPitch() const {
    return m_pitch;
}

BB3D Turret::getYawBox() const {
    return m_yaw_box;
}

BB3D Turret::getPitchBox() const {
    return m_pitch_box.onTop( m_yaw_box );
}

// Final camera bounding box, including both yaw and pitch
BB3D Turret::getCameraWideBox() const {
    return m_camera_wide_box.onTop( m_pitch_box.onTop( m_yaw_box ) );
}

BB3D Turret::getCameraNarrowBox() const {
    return m_camera_narrow_box.onTop( m_pitch_box.onTop( m_yaw_box ) );
}

BB3D Turret::getLaserBox() const {
    return m_laser_box.onTop( m_pitch_box.onTop( m_yaw_box ) );
}

bool Turret::getLaserStatus() const { return m_laser_active; }
void Turret::setLaserStatus( bool inp ) { m_laser_active = inp; }
