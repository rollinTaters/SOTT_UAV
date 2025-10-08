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


#include <chrono>   // thread sleep steady_clock
#include <iostream>     // cerr
#include "env_emulator.hpp"
#include "ngc.hpp"  // we gon get commanded speed and rate values
#include "raylib.h"
#include "raymath.h"    // matrix

Vector3 toVector3(const v3f& v) {
    return Vector3{ v.x, v.y, v.z };
}

namespace GUI{
    extern Vector3 taters2raylib( v3f );
}

Env_Emulator::Env_Emulator( const Vehicle& inp_vehicle ):
    m_real_vehicle(inp_vehicle)
{
}

Env_Emulator::~Env_Emulator()
{
    stopPhysSim();
    unloadModel();
}

bool Env_Emulator::startPhysSim()
{
    // check if we are already runnng the thread
    if( m_phys_thread != nullptr )
    {
        std::cerr<<"Environment Emulator: Warning: Tried to start physics thread when it is already running.\n";
        return false;
    }
    std::cout<<"Environment Emulator: Starting physics thread\n";
    m_run_phys_thread = true;
    m_phys_thread = new std::thread( &Env_Emulator::physThreadFunc, this );
    return true;
}

bool Env_Emulator::stopPhysSim()
{
    // send stop signals and join the threads here

    // check if we have a thread to stop
    if( m_phys_thread == nullptr )
    {
        std::cerr<<"Environment Manager: Warning: Tried to stop physics thread. We dont have a physics thread to stop.\n";
        return false;
    }
    std::cout<<"Environment Manager: Stopping physics thread\n";
    // signal physics thread to stop
    m_run_phys_thread = false;
    m_phys_thread->join();
    delete m_phys_thread;
    m_phys_thread = nullptr;

    // TODO is there a fail condition?? can the thread not join? what then?
    return true;
}

void Env_Emulator::setNGC( NGC * inp ) { m_ngc = inp; }

Vehicle Env_Emulator::getRealVehicle() const { return m_real_vehicle; }

/*
   SWITCH TO CHRONO
sf::Time Env_Emulator::getTime() const
{
    return m_clock.getElapsedTime();
}
*/

void Env_Emulator::physThreadFunc()
{
    unsigned int step_time_milli = 1000/30.f;
    double step_time = step_time_milli/1000.f;
    while( m_run_phys_thread )
    {
        // ah shit, here we go again...
        
        // we calculate new iteration values
        // and set current values to new iterations values
        // we do not want to mix old and new iteration values

        // A very lazy vehicle actuator simulation by hard setting vehicles velocity
        if( m_ngc != nullptr )
        {
            m_real_vehicle.m_vel.y = m_ngc->hey_emulator_speed;
            m_real_vehicle.m_angVel.z = m_ngc->hey_emulator_rate;
        }

        // to avoid mixing
        float fwd_vel = m_real_vehicle.m_vel.y;

        // translations
        m_real_vehicle.m_bb3d.translateLocal( m_real_vehicle.m_vel * step_time );

        // rotations, we should switch to quaternions...
        m_real_vehicle.m_bb3d.yawLeft( m_real_vehicle.m_angVel.z * step_time );
        m_real_vehicle.m_bb3d.pitchUp( m_real_vehicle.m_angVel.x * step_time );
        m_real_vehicle.m_bb3d.rollRight( m_real_vehicle.m_angVel.y * step_time );

        // first derivatives (using eulers method)
        m_real_vehicle.m_vel += m_real_vehicle.m_acc * step_time;
        m_real_vehicle.m_angVel += m_real_vehicle.m_angAcc * step_time;

        // second derivatives
        // these are affected by forces, we dont simulate forces. shit.
        // unless we simulate forces, we must model them
        
        // this is centripetal acceleration to model tire sideways friction
        if( m_real_vehicle.m_turn_radius != 0 )
            m_real_vehicle.m_acc.x = -(fwd_vel*fwd_vel)/m_real_vehicle.m_turn_radius;
        //m_angAcc = /*DONT HAVE FORCE DATA*/;

        // modelling turning
        if( m_real_vehicle.m_turn_radius != 0 )
            m_real_vehicle.m_angVel.z = fwd_vel/m_real_vehicle.m_turn_radius;

        // here be gravity
        /*  TODO we are not ready yet, there is no floor to resist our fall
        Quaternion qc = m_bb3d.getQuat().conjugate();
        v3f gravity( 0, 0, -9.81f );
        qc.rotateVector( gravity ); // gravity on local csys
        m_acc += gravity;
        */

        std::this_thread::sleep_for( std::chrono::milliseconds( step_time_milli ) );
        // TODO we wont sleep for exacly step_time, determine the slept time and use that as the "step time"
    }
}

    /*
float Env_Emulator::shittyPixelMarch( BB3D i_box, v3f i_dir ) const
{
    // pixel marching setup, for direction sensor
    unsigned int max_iteration = 1000;
    unsigned int iteration = 0;
    bool march_successful = false;
    Point start_pos = i_box.getPos();
    start_pos /= m_metre_per_pixel; // convert meters to pixel position
    Point check_pos = start_pos; // position we will iterate upon
    Point direction = i_dir * 0.005;


    while( iteration < max_iteration )
    {
        // check if pixel is marked as "wall"
        // TODO this is stupid, getimagecolor takes the image argument by copy to its function frame. change this to be a straight up "height" array
        Color colour =  GetImageColor( m_image_course, (int)round(check_pos.x),
                                           (int)round(check_pos.y) );
        if( ColorIsEqual( colour, BLACK ) )
        {
            // found wall, return it
            march_successful = true;
            break;
        }
        // TODO what about floors? at least return when point goes below Z0

        // TODO we may skip some pixels if we move by a unit vector, check if this is the case
        // move to next iteration
        check_pos += direction;
        iteration++;
    }

    // returning found value
    if( !march_successful )
    {
        // NOTE: maybe there was no obstacle? this isnt necessarly a failure
        //std::cerr<<"Environment emulator: sensor did not hit obstacle\n";
        return 0.f;
    }
    return ( (check_pos - start_pos)*m_metre_per_pixel ).mag();
}
    */

float Env_Emulator::useRaycast( v3f i_pos, v3f i_dir ) const
{
    // Raycollision -> bool hit, float distance, Vector3 point, Vector3 normal
    // Ray -> Vector3 position, Vector3 direction

    //std::cout<<"useRaycast, direction: "<< i_dir.x <<"x "<<i_dir.y<<"y "<<i_dir.z<<"z\n";

    Vector3 map_offset = GUI::taters2raylib( Point( 0, -1080*m_metre_per_pixel, -0.25 ) );

    Vector3 start_pos = GUI::taters2raylib( i_pos );
    Vector3 ray_dir = GUI::taters2raylib( i_dir );
    Ray rey = { start_pos, ray_dir };

    RayCollision collision = GetRayCollisionMesh(
            rey,
            m_hm_mesh,
            getWorldTransform( map_offset ) );

    if( collision.hit )
        return collision.distance;
    else
        return 0.f;
}

Matrix Env_Emulator::getWorldTransform( Vector3 offset ) const
{
    // get quaternions and positions, convert them to raylib axis conventions
    cQuaternion veh_quat = m_real_vehicle.getBox().getQuaternion();
    Vector3 veh_quat_axis = GUI::taters2raylib( v3f{ veh_quat.x, veh_quat.y, veh_quat.z } );
    Vector3 veh_pos = GUI::taters2raylib( m_real_vehicle.getBox().getPos() );
    veh_pos += offset;

    // convert quaternions and vectors to raylib units
    Quaternion vehicle_rotation = { veh_quat_axis.x, veh_quat_axis.y, veh_quat_axis.z, -veh_quat.w };
    Vector3 vehicle_position = { veh_pos.x, veh_pos.y, veh_pos.z };

    Matrix rotation_matrix = QuaternionToMatrix( vehicle_rotation );

    Matrix translation_matrix = MatrixTranslate( -vehicle_position.x, -vehicle_position.y, -vehicle_position.z );

    return MatrixMultiply( translation_matrix, rotation_matrix );
}

bool Env_Emulator::getSensorData( Sensor_Emulator* sensor, Sensor_Data& data ) const
{
    // nullptr sensor protection
    if (!sensor) return false;

    // determine sensor type
    Sensor_Type sensor_type = sensor->getType();

    // determine sensor position
    //BB3D sensor_box = (sensor->getBox()).onTop( m_real_vehicle.getBox() );
    //sensor_box += m_real_vehicle.getBox();
    BB3D sensor_box = sensor->getBox();
    
    // for lidar case
    float rad_increment = (2*PI)/LIDAR_POINTS;
    v3f sensor_direction = sensor_box.getLocalVecY();
    cQuaternion lidar_quat = cQuaternion::fromAxisAngle( sensor_box.getLocalVecZ(), rad_increment );

    switch( sensor_type )
    {
        default:
        case E_type_undefined:
            std::cerr<<"ERROR: Environment emulator got request to read undefined type sensors data\n";
            return false;
        // ---- Simple distance sensor ----
        case E_type_distance:
            //data.distance = shittyPixelMarch( sensor_box, sensor_box.getLocalVecY() );
            data.distance = useRaycast( sensor_box.getPos(), sensor_box.getLocalVecY() );
            return true;

        // ---- LIDAR ----
        case E_type_LIDAR:
            //std::cout<<"env emulator: reading lidar sensor\n";    // DEBUG
            for( int i = 0; i < LIDAR_POINTS; i++ )
            {
                //data.lidar[i] = shittyPixelMarch( sensor_box, sensor_direction );
                data.lidar[i] = useRaycast( sensor_box.getPos(), sensor_direction );
                data.lidar_angle[i] = rad_increment * i;
                lidar_quat.rotateVector( sensor_direction );
            }
            return true;

        // ---- IMU ----
        case E_type_IMU:
            data.acceleration = m_real_vehicle.getAcc();
            data.velocity = m_real_vehicle.getVel();
            data.angular_rate = m_real_vehicle.getAngVel();
            // TODO data.magnetic_north;
            data.barometric_pressure = 101325;
            return true;

        // ---- Temperature ----
        case E_type_temperature:
            // TODO
            data.temperature = -273.3;  // ankara
            return false;

        // ---- Electric Current ----
        case E_type_current:
            // TODO
            data.electric_current = 0;
            return false;
        
        // ---- Turret Encoders ----
        case E_type_turret_encoder:
            data.turret_pitch = m_real_vehicle.getTurret().getPitch();
            data.turret_yaw = m_real_vehicle.getTurret().getYaw();
            return true;

    }
}

bool Env_Emulator::setupModel()
{
    // ---- height map setup ----
    // load the course image from the disk
    Image im_height_map = LoadImage("./gfx/course1.png");
    if( !IsImageValid( im_height_map ) )
    {
        std::cerr<<"Error. Could not load course image from file!\n";
        m_model_initialized = false;
        return false;
    }
    //im_height_map = GenImageCellular( 2000, 3000, 75 );
    //ImageFlipVertical( &im_height_map );
    //ImageFlipHorizontal( &im_height_map );
    //ImageBlurGaussian( &im_height_map, 12 );
    //ImageColorInvert( &im_height_map );
    //ImageColorGrayscale( &im_height_map );

    // set position of m_real_vehicle in course
    m_real_vehicle.overridePos( Point(340*m_metre_per_pixel, (1080-775)*m_metre_per_pixel, 0.5f) );

    float resize_factor = 0.2f;

    // grab original mesh size
    Vector3 mesh_size = {
        m_metre_per_pixel*im_height_map.width,
        2,
        m_metre_per_pixel*im_height_map.height };

    // resize image down
    ImageResize( &im_height_map,
            im_height_map.width*resize_factor,
            im_height_map.height*resize_factor );

    // generate height map
    m_hm_mesh = GenMeshHeightmap( im_height_map, (mesh_size) );

    // resize image to original, ( here be losses )
    ImageResize( &im_height_map,
            im_height_map.width/resize_factor,
            im_height_map.height/resize_factor );

    // invert color before loading texture, so its lighter color
    ImageColorInvert( &im_height_map );
    Image im_overlay = GenImageCellular( im_height_map.width, im_height_map.height, 20 );
    Rectangle rect = {0,0,im_height_map.width,im_height_map.height};
    ImageDraw( &im_height_map, im_overlay, rect, rect, Fade(WHITE, 0.5f) );

    m_hm_texture = LoadTextureFromImage( im_height_map );
    m_hm_model = LoadModelFromMesh( m_hm_mesh );
    m_hm_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = m_hm_texture;
    UnloadImage( im_height_map );

    m_model_initialized = true;
    return true;
}

bool Env_Emulator::unloadModel()
{
    if( m_model_initialized )
    {
        UnloadTexture( m_hm_texture );
        UnloadModel( m_hm_model );
        m_model_initialized = false;
        return true;
    }
    return false;
}

void Env_Emulator::drawHMap() 
{
    if( !m_model_initialized ) return;

    // map start position offset
    //Vector3 map_offset = GUI::taters2raylib( Point(340*m_metre_per_pixel, (1080-775)*m_metre_per_pixel, 0.5f) );
    Vector3 map_offset = GUI::taters2raylib( Point( 0, -1080*m_metre_per_pixel, -0.25 ) );

    m_hm_model.transform = getWorldTransform( map_offset );

    DrawModel( m_hm_model, (Vector3){0,0,0}, 1.f, DARKGREEN );

}

