/*
    This file should only be included in main.cpp
    if you include it multiple times, you will get multiple definition errors for variables defined here
    ( yes we could fix that issue with a gui.cpp file, but i wanna do it this way. )
*/
#include "raylib.h"
#include "raymath.h"

// a slight nod to external objects
extern NGC ngc_system;
extern Vehicle simulated_vehicle;
extern Env_Emulator env_emulator;


namespace GUI
{

    Vector3 taters2raylib( Vector3 inp )
    { return Vector3RotateByAxisAngle( inp, (Vector3){1,0,0}, 3*PI/2.f ); }

    Vector3 taters2raylib( float x, float y, float z )
    { return Vector3RotateByAxisAngle( (Vector3){x,y,z}, (Vector3){1,0,0}, 3*PI/2.f ); }

    Vector3 taters2raylib( Point p)
    { return Vector3RotateByAxisAngle( (Vector3){p.x,p.y,p.z}, (Vector3){1,0,0}, 3*PI/2.f ); }

    Vector3 raylib2taters( Vector3 inp )
    { return Vector3RotateByAxisAngle( inp, (Vector3){1,0,0}, PI/2.f ); }

    // NOTE: These variables will live in main.cpp
    Camera3D camera;
    Font font;
    Texture2D texture_xp;
    Texture2D texture_xn;
    Texture2D texture_yp;
    Texture2D texture_yn;
    Texture2D texture_zp;
    Texture2D texture_zn;

    RenderTexture2D turretViewRT;
    Camera3D turretCam;
    bool use_wide_camera = true;

    // crosshair variables
    Vector3 crosshair;
    float ch_gap;   // empty distance in the center
    float ch_len;   // line length
    Color ch_color; // color of crosshair lines

    std::vector<Point> ngc_wps;
    double last_wp_update;
    double wp_update_rate;

    bool display_world;

    bool hat_mode_ch;
    float cmd_speed;
    float cmd_rate;
    float cmd_nom_speed;
    float cmd_nom_rate;

    void initGUI()
    {
        // create a window for gui rendering
        InitWindow( 1000,800, "NGC DEBUG GUI" );

        crosshair = {0,0,0};
        ch_gap = 0.05;
        ch_len = 0.15;
        ch_color = LIME;

        display_world = true;
        hat_mode_ch = true;
        cmd_nom_speed = 2.5;
        cmd_nom_rate = 18* (PI/180);

        // camera setup
        camera = {0};
        camera.position = (Vector3){ 0.f, 10.f, 10.f };
        camera.target = (Vector3){ 0.f, 0.f, 0.f };
        camera.up = (Vector3){ 0.f, 1.f, 0.f };
        camera.fovy = 45.f;
        camera.projection = CAMERA_PERSPECTIVE;

        font = LoadFont( "./gfx/PixeloidSans.ttf" );
        texture_xp = LoadTextureFromImage( ImageTextEx( font, "X+", 110, 5, BLACK ) );
        texture_xn = LoadTextureFromImage( ImageTextEx( font, "X-", 110, 5, BLACK ) );
        texture_yp = LoadTextureFromImage( ImageTextEx( font, "Y+", 110, 5, BLACK ) );
        texture_yn = LoadTextureFromImage( ImageTextEx( font, "Y-", 110, 5, BLACK ) );
        texture_zp = LoadTextureFromImage( ImageTextEx( font, "Z+", 110, 5, BLACK ) );
        texture_zn = LoadTextureFromImage( ImageTextEx( font, "Z-", 110, 5, BLACK ) );

        SetTargetFPS(60);

        ngc_wps = ngc_system.getWPs();
        last_wp_update = GetTime();
        wp_update_rate = 1; // seconds

        turretViewRT = LoadRenderTexture(
                            simulated_vehicle.getTurret().camera_wide_resolution_x,
                            simulated_vehicle.getTurret().camera_wide_resolution_y );
        turretCam = camera;  // Aynı ayarları kullanabiliriz

    }

    void deInitGUI()
    {
        UnloadTexture( texture_xp );
        UnloadTexture( texture_xn );
        UnloadTexture( texture_yp );
        UnloadTexture( texture_yn );
        UnloadTexture( texture_zp );
        UnloadTexture( texture_zn );
        UnloadRenderTexture( turretViewRT );
        // de-initialization of window and opengl context
        CloseWindow();
    }

    // DEBUG - draws axes of given box, useful for determining attitude of objects
    void drawAxes( BB3D box )
    {
        float line_length = 0.3f;   // meters

        Vector3 pos = taters2raylib( box.getPos() );
        Vector3 ax_x = taters2raylib( box.getLocalVecX() * line_length );
        Vector3 ax_y = taters2raylib( box.getLocalVecY() * line_length );
        Vector3 ax_z = taters2raylib( box.getLocalVecZ() * line_length );

        Color color_x = RED;
        Color color_y = RED;
        Color color_z = RED;
        
        DrawLine3D( pos, pos+ax_x, color_x );
        DrawLine3D( pos, pos+ax_y, color_y );
        DrawLine3D( pos, pos+ax_z, color_z );
    }

    void checkUserInput()
    {
        // moving camera around
        if( IsKeyDown( KEY_H ) )
            camera.position = Vector3RotateByAxisAngle( camera.position, camera.up, -2*DEG2RAD );
        if( IsKeyDown( KEY_L ) )
            camera.position = Vector3RotateByAxisAngle( camera.position, camera.up,  2*DEG2RAD );
        if( IsKeyDown( KEY_J ) )
            camera.position = Vector3RotateByAxisAngle( camera.position, Vector3CrossProduct(camera.position-camera.target, camera.up), -2*DEG2RAD );
        if( IsKeyDown( KEY_K ) )
            camera.position = Vector3RotateByAxisAngle( camera.position, Vector3CrossProduct(camera.position-camera.target, camera.up),  2*DEG2RAD );
        if( IsKeyDown( KEY_N ) )
            camera.position += Vector3Normalize((camera.position-camera.target)) *  0.2f;
        if( IsKeyDown( KEY_M ) )
            camera.position += Vector3Normalize((camera.position-camera.target)) * -0.2f;

        // hat control mode
        if( IsKeyPressed( KEY_T ) )
            hat_mode_ch = !hat_mode_ch;

        // toggle between rendering world or not
        if( IsKeyPressed( KEY_B ) )
            display_world = !display_world;

        constexpr float ch_speed = 0.1;
        if( hat_mode_ch )
        {
            // moving crosshair
            if( IsKeyDown( KEY_W ) )
                crosshair += { 0, +ch_speed, 0};
            if( IsKeyDown( KEY_S ) )
                crosshair += { 0, -ch_speed, 0};
            if( IsKeyDown( KEY_A ) )
                crosshair += { -ch_speed, 0, 0};
            if( IsKeyDown( KEY_D ) )
                crosshair += { +ch_speed, 0, 0};
            if( IsKeyDown( KEY_R ) )
                crosshair += { 0, 0, +ch_speed};
            if( IsKeyDown( KEY_F ) )
                crosshair += { 0, 0, -ch_speed};
        }else{
            cmd_speed = 0;
            cmd_rate = 0;
            if( IsKeyDown( KEY_W ) )
                cmd_speed = cmd_nom_speed;
            if( IsKeyDown( KEY_S ) )
                cmd_speed = -1*cmd_nom_speed;
            if( IsKeyDown( KEY_A ) )
                cmd_rate = cmd_nom_rate;
            if( IsKeyDown( KEY_D ) )
                cmd_rate = -1*cmd_nom_rate;

            ngc_system.directCommand( cmd_speed, cmd_rate );
        }


        // -------- TURRET --------

        Turret& turret = simulated_vehicle.getTurret();  // vehicle.hpp'de getTurret fonksiyonu olmalı

        BB3D cam_box;

        // update turret cameras variables
        if( use_wide_camera )
        {
            cam_box = turret.getCameraWideBox();
            turretCam.fovy = 60.f;
        }else{
            cam_box = turret.getCameraNarrowBox();
            turretCam.fovy = 10.f;
        }
        v3f cam_pos = cam_box.getPos();
        v3f cam_dir = cam_box.getLocalVecY();
        v3f cam_up  = cam_box.getLocalVecZ();

        turretCam.position = taters2raylib( cam_pos );
        turretCam.target = taters2raylib( cam_pos + cam_dir );
        turretCam.up = taters2raylib( cam_up );


        // turret control (yaw & pitch)

        float deltaYaw = 0.01f;
        float deltaPitch = 0.01f;

        if (IsKeyDown(KEY_LEFT))  // ←
            turret.setYaw(turret.getYaw() + deltaYaw);
        if (IsKeyDown(KEY_RIGHT)) // →
            turret.setYaw(turret.getYaw() - deltaYaw);

        if (IsKeyDown(KEY_UP))    // ↑
            turret.setPitch(turret.getPitch() + deltaPitch);
        if (IsKeyDown(KEY_DOWN))  // ↓
            turret.setPitch(turret.getPitch() - deltaPitch);

        if( IsKeyPressed(KEY_O) )  // o
            use_wide_camera = !use_wide_camera;
        if( IsKeyPressed(KEY_I) )   // this is "I" without the dot on turkish keyboard layout
            turret.setLaserStatus( !turret.getLaserStatus() );


        // -------- NGC COMMANDS --------

        if( IsKeyPressed( KEY_X ) )
            ::ngc_system.executeWPs();
        if( IsKeyPressed( KEY_C ) )
        {
            Point p(crosshair.x, crosshair.y, crosshair.z);
            simulated_vehicle.getBox().getQuaternion().rotateVector( p );
            Point p_veh = simulated_vehicle.getPos();
            ::ngc_system.addWP( p+p_veh );

            std::cout<<"p_veh: "<<p_veh.x<<"x "<<p_veh.y<<"y "<<p_veh.z<<"z\n";    // DEBUG
            std::cout<<"p_cro: "<<p.x<<"x "<<p.y<<"y "<<p.z<<"z\n";    // DEBUG
        }


    }

    void drawAxisBillboards()
    {
        // camera, texture, position, scale, color
        DrawBillboard( camera, texture_xp, taters2raylib((Vector3){ 5, 0, 0}), 0.5f, WHITE );
        DrawBillboard( camera, texture_xn, taters2raylib((Vector3){-5, 0, 0}), 0.5f, WHITE );
        DrawBillboard( camera, texture_yp, taters2raylib((Vector3){ 0, 5, 0}), 0.5f, WHITE );
        DrawBillboard( camera, texture_yn, taters2raylib((Vector3){ 0,-5, 0}), 0.5f, WHITE );
        DrawBillboard( camera, texture_zp, taters2raylib((Vector3){ 0, 0, 5}), 0.5f, WHITE );
        DrawBillboard( camera, texture_zn, taters2raylib((Vector3){ 0, 0,-5}), 0.5f, WHITE );
        // slices, spacing
        DrawGrid(10, 2);

    }

    void drawVehicleBody()
    {
        float wheelbase = simulated_vehicle.wheelbase;
        float track = simulated_vehicle.track;
        float wheel_dia = simulated_vehicle.wheel_dia;
        float wheel_width = simulated_vehicle.wheel_width;
        // front right wheel
        DrawCylinderEx( taters2raylib( track/2 +wheel_width/2, wheelbase/2, 0 ),
                        taters2raylib( track/2 -wheel_width/2, wheelbase/2, 0 ),
                        wheel_dia/2, wheel_dia/2, 20, DARKGRAY );
        // front left wheel
        DrawCylinderEx( taters2raylib( -track/2 +wheel_width/2, wheelbase/2, 0 ),
                        taters2raylib( -track/2 -wheel_width/2, wheelbase/2, 0 ),
                        wheel_dia/2, wheel_dia/2, 20, DARKGRAY );
        // rear right wheel
        DrawCylinderEx( taters2raylib( track/2 +wheel_width/2, -wheelbase/2, 0 ),
                        taters2raylib( track/2 -wheel_width/2, -wheelbase/2, 0 ),
                        wheel_dia/2, wheel_dia/2, 20, DARKGRAY );
        // rear left wheel
        DrawCylinderEx( taters2raylib( -track/2 +wheel_width/2, -wheelbase/2, 0 ),
                        taters2raylib( -track/2 -wheel_width/2, -wheelbase/2, 0 ),
                        wheel_dia/2, wheel_dia/2, 20, DARKGRAY );
        // body
        DrawCubeV( taters2raylib((Vector3){0,0,0.20f}), taters2raylib((Vector3){1.15f, 1.65f, 0.50f}), GREEN ); 

        // some sensors
        for( auto i = 0; i < simulated_vehicle.m_num_sensors; i++ )
        {
            BB3D box = simulated_vehicle.getSensor(i).getBox();
            DrawCubeV(
                    taters2raylib( box.getPos() ),
                    taters2raylib( box.getSize() ),
                    RED );
            DrawLine3D(
                    taters2raylib( box.getPos() ),
                    taters2raylib( box.getPos() + box.getLocalVecY() ),
                    RED );
        }

    }

    void drawTurret()
    {
        Turret turret = simulated_vehicle.getTurret();

        // turret base (yaw)
        BB3D yaw_box = turret.getYawBox();
        DrawCubeV(taters2raylib(yaw_box.getPos()), taters2raylib(yaw_box.getSize()), DARKBLUE);

        // turret pitch platform (pitch)
        BB3D pitch_box = turret.getPitchBox();
        DrawCubeV(taters2raylib(pitch_box.getPos()), taters2raylib(pitch_box.getSize()), BLUE);
        
        // turret general purpose camera
        BB3D w_cam_box = turret.getCameraWideBox();
        DrawCubeV( taters2raylib( w_cam_box.getPos() ), taters2raylib( w_cam_box.getSize() ), SKYBLUE );

        // turret aiming camera
        BB3D n_cam_box = turret.getCameraNarrowBox();
        DrawCubeV( taters2raylib( n_cam_box.getPos() ), taters2raylib( n_cam_box.getSize() ), SKYBLUE );

        BB3D laser_box = turret.getLaserBox();
        DrawCubeV( taters2raylib( laser_box.getPos() ), taters2raylib( laser_box.getSize() ), MAROON );
        if( turret.getLaserStatus() )
            DrawLine3D( taters2raylib( laser_box.getPos() ), taters2raylib( laser_box.getPos()+(laser_box.getLocalVecY())*100.f ), RED );
       
        // camera direction line
        Vector3 cam_pos = taters2raylib( w_cam_box.getPos() );
        Vector3 cam_dir = taters2raylib( w_cam_box.getLocalVecY() );
        DrawLine3D(cam_pos, Vector3Add( cam_pos, Vector3Scale(cam_dir, 1.0f)), YELLOW );

        /*// DEBUG
        drawAxes( yaw_box );
        drawAxes( pitch_box );
        drawAxes( w_cam_box );
        drawAxes( n_cam_box );
        */
    }

    void drawCrosshair()
    {
        // x+
        DrawLine3D( taters2raylib(crosshair+(Vector3){ch_gap,0,0}),
                    taters2raylib(crosshair+(Vector3){ch_gap+ch_len,0,0}),
                    ch_color );
        // x-
        DrawLine3D( taters2raylib(crosshair+(Vector3){-ch_gap,0,0}),
                    taters2raylib(crosshair+(Vector3){-ch_gap-ch_len,0,0}),
                    ch_color );
        // y+
        DrawLine3D( taters2raylib(crosshair+(Vector3){0,ch_gap,0}),
                    taters2raylib(crosshair+(Vector3){0,ch_gap+ch_len,0}),
                    ch_color );
        // y-
        DrawLine3D( taters2raylib(crosshair+(Vector3){0,-ch_gap,0}),
                    taters2raylib(crosshair+(Vector3){0,-ch_gap-ch_len,0}),
                    ch_color );
        // z+
        DrawLine3D( taters2raylib(crosshair+(Vector3){0,0,ch_gap}),
                    taters2raylib(crosshair+(Vector3){0,0,ch_gap+ch_len}),
                    ch_color );
        // z-
        DrawLine3D( taters2raylib(crosshair+(Vector3){0,0,-ch_gap}),
                    taters2raylib(crosshair+(Vector3){0,0,-ch_gap-ch_len}),
                    ch_color );
        
        // drop line
        if( crosshair.z > (ch_gap+ch_len + 0.05) )
            DrawLine3D( taters2raylib(crosshair+(Vector3){0,0,-crosshair.z}),
                        taters2raylib(crosshair+(Vector3){0,0,-ch_gap-ch_len}),
                        ch_color );
        if( crosshair.z < -(ch_gap+ch_len + 0.05) )
            DrawLine3D( taters2raylib(crosshair+(Vector3){0,0,-crosshair.z}),
                        taters2raylib(crosshair+(Vector3){0,0,ch_gap+ch_len}),
                        ch_color );


    }

    void drawOverlay()
    {
        // font, text, position2, font size, spacing, color
        DrawTextEx(
                font,
                "H,J,K,L: camera movement\n"
                "N,M: zoom\n"
                "B: show/hide terrain\n"
                "W,A,S,D,R,F: crosshair move\n"
                "X: execute waypoints\n"
                "C: create waypoint\n"
                "T: toggle hat mode\n"
                "O: change turret camera\n"
                "I: toggle laser\n"
                "Left/Right: turret yaw\n"
                "Up/Down: turret pitch\n",
                (Vector2){10,500}, 20, 2, BLACK ); //cannot see stuff, changed to black
        DrawTextEx(
                font,
                TextFormat("crosshair: %3.2fx %3.2fy %3.2fz \n\theading: %3.2f",
                    crosshair.x, crosshair.y, crosshair.z, (v3f){crosshair.x, crosshair.y, crosshair.z}.heading()),
                (Vector2){10,30}, 20, 2, DARKGRAY );
        DrawTextEx(
                font,
                TextFormat("num WPs: %d", ngc_wps.size()),
                (Vector2){10,70}, 20, 2, DARKGRAY );
        for( int i = 0; i < (int)ngc_wps.size(); i++ )
        {
            Point p = ngc_wps[i];
            DrawTextEx( font, TextFormat("wp%d: %3.1fx %3.1fy %3.1fz", i, p.x, p.y, p.z),
                        (Vector2){160, 70+(16.f*i)}, 16, 1, DARKGRAY );
        }
    
        // position of vehicle in different frames
        v3f real_pos = env_emulator.getRealVehicle().getBox().getPos();
        v3f real_att = env_emulator.getRealVehicle().getBox().getAngEuler();
        v3f internal_pos = simulated_vehicle.getBox().getPos();
        v3f internal_vel = simulated_vehicle.getVel();
        v3f internal_att = simulated_vehicle.getBox().getAngEuler();
        v3f turret_att = simulated_vehicle.getTurret().getCameraNarrowBox().getAngEuler();
        DrawTextEx(
            font,
            TextFormat("Simulation Pos: %3.2fx  %3.2fy  %3.2fz",
                real_pos.x, real_pos.y, real_pos.z),
            (Vector2){350, 10}, 20, 2, BLACK );
        DrawTextEx(
            font,
            TextFormat("  Internal Pos: %3.2fx  %3.2fy  %3.2fz",
                internal_pos.x, internal_pos.y, internal_pos.z),
            (Vector2){350, 30}, 20, 2, BLACK );
        DrawTextEx(
            font,
            TextFormat("  Internal vel: %3.2fx  %3.2fy  %3.2fz",
                internal_vel.x, internal_vel.y, internal_vel.z),
            (Vector2){350, 50}, 20, 2, BLACK );
        DrawTextEx(
            font,
            TextFormat("  Internal att: %3.2fyaw  %3.2fpitch  %3.2froll",
                internal_att.x, internal_att.y, internal_att.z),
            (Vector2){350, 70}, 20, 2, BLACK );
        DrawTextEx(
            font,
            TextFormat("Simulation att: %3.2fyaw  %3.2fpitch  %3.2froll",
                real_att.x, real_att.y, real_att.z),
            (Vector2){350, 90}, 20, 2, BLACK );
        DrawTextEx(
            font,
            TextFormat("Turret att: %3.2fyaw  %3.2fpitch  %3.2froll",
                turret_att.x, turret_att.y, turret_att.z),
            (Vector2){350, 110}, 20, 2, BLACK );

        // ngc system output
        DrawTextEx(
                font,
                TextFormat("ngc target speed: %3.2fm/s   rate: %3.2frad/s",
                    ngc_system.hey_emulator_speed, ngc_system.hey_emulator_rate),
                (Vector2){350, 140}, 20, 2, BLACK );
    }

    void drawWPs()
    {
        if( last_wp_update + wp_update_rate < GetTime() )
            ngc_wps = ngc_system.getWPs();

        Point p_veh = simulated_vehicle.getPos();

        int wp_num = 0;
        for( Point p : ngc_wps )
        {
            //DrawPoint3D( taters2raylib(p), ORANGE );
            Point p_wp = p-p_veh;
            simulated_vehicle.getBox().getQuaternion().conjugate().rotateVector( p_wp );
            Vector2 pos2d = GetWorldToScreen( taters2raylib(p_wp), camera );
            DrawCircle( pos2d.x, pos2d.y, 5.0, ORANGE );
            DrawTextEx( font, TextFormat("%d",wp_num),
                        pos2d, 20, 1, ORANGE );
            wp_num++;
        }
    }

    void drawLIDAR()
    {
        std::vector<Point> pts = ngc_system.getImObPoints();
        //std::cout<<"DEBUG: ImObPoints count: "<< pts.size()<<"\n";  // DEBUG
        for( Point &p : pts )
        {
            DrawSphere( taters2raylib(p), 0.1, RED );
            //std::cout<<"ImObP: "<<p.x<<"x "<<p.y<<"y "<<p.z<<"z\n"; // DEBUG
        }
    }

//#include "ngc.hpp"  // MapPoint struct is here, and this include is out of place. hurray spaghetti code
    void drawMapPoints()
    {
        std::vector<MapPoint> pts = ngc_system.getMapPoints();
        for( MapPoint p : pts )
        {
            p.pos -= simulated_vehicle.getBox().getPos();
            simulated_vehicle.getBox().getQuaternion().conjugate().rotateVector( p.pos );
            DrawSphere(
                    taters2raylib(p.pos),
                    p.radius,
                    ColorAlpha(BLACK, std::max( p.confidence, 0.1f )) );
        }
    }

    void drawPath()
    {
        //v3f veh_pos = simulated_vehicle.getBox().getPos();
        //v3f veh_X = simulated_vehicle.getBox().getLocalVecX();
        //v3f veh_Z = simulated_vehicle.getBox().getLocalVecZ();
        float turn_radius = ngc_system.getTurnRadius();

        v3f center_point = v3f(1,0,0) * turn_radius;

        /// TODO this shit be broken. I cant lay the circle parallel to the ground. 
        // Vector3 center, float radius, Vector3 rotation_axis, float rotation_angle, Color color
        DrawCircle3D(
                GUI::taters2raylib(center_point),
                turn_radius,
                {1,0,0},
                PI/8,
                YELLOW );

    }

    void drawPredictOPs()
    {
        OP *pOPs;
        pOPs = ngc_system.getPredictOPs();
        Point p_veh = simulated_vehicle.getPos();
        Point p_prev;

        //std::cout<<"ops printing:\n";
        for( int i = 0; i < 40; i++ )
        {
            Point p_op = pOPs[i].pos-p_veh;
            if( i == 0 )
                p_prev = {0,0,0}; // p_veh-p_veh
            else
                p_prev = pOPs[i-1].pos-p_veh;

            simulated_vehicle.getBox().getQuaternion().conjugate().rotateVector( p_op );
            simulated_vehicle.getBox().getQuaternion().conjugate().rotateVector( p_prev );
            DrawSphere( taters2raylib( p_op ), 0.1, YELLOW );
            DrawLine3D( taters2raylib( p_op ), taters2raylib( p_prev ), YELLOW );
            //std::cout<<"op: "<<op.pos.x<<"x "<<op.pos.y<<"y "<<op.pos.z<<"z\n";
        }
        //std::cout<<"ops done\n\n";
    }


};
