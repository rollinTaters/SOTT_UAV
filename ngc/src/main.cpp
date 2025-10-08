/*
	MIT License

	Copyright (c) 2025 rollinTaters

*/

#include <iostream>
#include "vehicle.hpp"
#include "env_emulator.hpp"
#include "sensor_emulator.hpp"
#include "ngc.hpp"

#ifdef DEBUG_GUI
#include "gui.hpp"  // raylib joins here
#endif

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << "Vector3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

#ifdef DEBUG_GUI
// DEBUG -- sends env_emulators cam view to command console
#include "../../common_code/src/video_feed.hpp"
void sendCamera2Console() 
{
    static CommsModule undef_comms( CommsModule::udp, CommsModule::random_channel1 );
    static VideoFeed videofeed( 3 );

    std::uint8_t *raw_image_data = nullptr;
    int raw_image_data_size = 123;

    int feed_resolution_x = videofeed.getResolutionWidth();
    int feed_resolution_y = videofeed.getResolutionHeight();


    // RenderTexture2D kamerandan görüntü al
    Image img = LoadImageFromTexture( GUI::turretViewRT.texture ); // BURADA RAM'e çekiyoruz

    ImageResize( &img, feed_resolution_x, feed_resolution_y );
    ImageFormat( &img, PIXELFORMAT_UNCOMPRESSED_R8G8B8 );
    ImageFlipVertical( &img );   // because render textures are flipped, GPU's are weird..

    raw_image_data = ExportImageToMemory( img, ".png", &raw_image_data_size );

    // alternative, but doesnt work
    //raw_image_data_size = (feed_resolution_x*feed_resolution_y*3);
    //std::memcpy( raw_image_data, img.data, raw_image_data_size );

    // Image processing fonksiyonuna gönder
    //ProcessImage(img);
    videofeed.TXFrame( raw_image_data, raw_image_data_size );
    std::vector<CommsPacket> tx_packets = videofeed.getTXPackets();

    /*// DEBUG        ----------------
    std::cout<<"raw image data size: "<<raw_image_data_size<<"\n";
    std::cout<<"raw image data thats being sent:\n";
    for( int i = 0; i < raw_image_data_size; i++ )
    {
        std::cout<< (int)(raw_image_data[i]) <<" ";
    }
    std::cout<<"\n";
    std::cout<<"num of tx packets: "<<tx_packets.size()<<"\n";
    std::cout<<"CommsPackets:\n";
    for( auto p : tx_packets )
    {
        std::cout<<"-->";
        for( auto d : p.data )
            std::cout<< (int)d <<" ";
        std::cout<<"<--\n";
    }
    std::cout<<"\n";
    // DEBUG END    ----------------    */

    // send 'em away
    for( CommsPacket p : tx_packets )
        if( !undef_comms.sendPacket( p, CommsModule::console_channel ) )
            std::cout<<"packet sent failed!! ";

    // RAM'den sildik (memory leak olmasın)
    UnloadImage(img);
    delete raw_image_data;
}
#endif

// a clock for sending camera view, DEBUG
std::chrono::steady_clock m_clock;
std::chrono::milliseconds m_camera_send_interval = std::chrono::milliseconds(50);
std::chrono::time_point<std::chrono::steady_clock> m_last_camera_send_time;
// END OF CAMERA DEBUG

// this is the vehicle we are managing/controlling
Vehicle simulated_vehicle;

// our global environment emulator, if you included env_emulator.hpp, you know about its existance
Env_Emulator env_emulator( simulated_vehicle );

// our navigation guidance and control system
NGC ngc_system( &simulated_vehicle );

int main()
{
    std::cout<<"ULV NGC Emulator v0.2\n";

    m_last_camera_send_time = m_clock.now();    // DEBUG

    // introduce ngc to the env emulator, because I gave up on a better way to do this
    env_emulator.setNGC( &ngc_system );
    env_emulator.startPhysSim();
    ngc_system.start();

#ifdef DEBUG_GUI
    SetTraceLogLevel( LOG_WARNING );    // also LOG_ERROR LOG_FATAL LOG_INFO LOG_NONE

    // this will create a window and initialize gui stuff
    GUI::initGUI();

    // after creating opengl context (initializing window) setup the model in env emulator
    env_emulator.setupModel();

    // main draw loop
    while( !WindowShouldClose() )
    {
        // this will mainly move camera around
        GUI::checkUserInput();

        BeginDrawing();
        BeginTextureMode( GUI::turretViewRT ); // --- texture mode start, Turret Camera texture gets populated here
        ClearBackground(SKYBLUE);

        BeginMode3D( GUI::turretCam );
        env_emulator.drawHMap();
        GUI::drawVehicleBody();
        //GUI::drawTurret();   // camera gets obscured by a part of turret. well... fixme
        EndMode3D();

        EndTextureMode();   // --------------- texture mode end

        ClearBackground( RAYWHITE );

        BeginMode3D( GUI::camera );  //--- mode 3D start, our debug screen get populated here

        if( GUI::display_world )
            env_emulator.drawHMap();
        GUI::drawLIDAR();
        GUI::drawMapPoints();
        GUI::drawAxisBillboards();
        GUI::drawVehicleBody();
        GUI::drawTurret();
        GUI::drawPath();
        GUI::drawPredictOPs();
        GUI::drawCrosshair();

        EndMode3D();  //------------- mode 3D end

        DrawFPS( 10, 10 );
        GUI::drawWPs();

        // display turret camera's video feed on screen
        DrawTexturePro(
                GUI::turretViewRT.texture,  // texture itself
                {0, 0, 1920, -1080},        // source rectangle
                {700, 30, 1920/8, 1080/8},  // destination rectangle
                {0, 0},     // origin
                0.f,        // rotation
                WHITE );    // tint
        DrawText("Turret View:", 600, 10, 10, DARKGRAY);

        GUI::drawOverlay(); // nearly all the text is here

        // send turret cam view to command console, debug
        if( m_clock.now() > m_last_camera_send_time + m_camera_send_interval )
        {
            sendCamera2Console();
            m_last_camera_send_time = m_clock.now();
        }

        EndDrawing();
    }

    env_emulator.stopPhysSim();
    ngc_system.stop();
    env_emulator.unloadModel();
    // this also closes the window
    GUI::deInitGUI();
#else   // HEADLESS MODE
    std::this_thread::sleep_for( std::chrono::seconds(10) );
#endif  // DEBUG_GUI

    std::cout<<"Exiting. Have a nice day\n";
}
