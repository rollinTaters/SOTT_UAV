#include "../../common_code/src/comms_module.hpp"
#include "../../common_code/src/video_feed.hpp"
#include "joystick.hpp"
#include "../../common_code/src/aircraft.hpp"
#include "console_graphics.hpp"
#include "raylib.h"
#include "toast.hpp"
#include "serial_manager.hpp"
#include <chrono> // throttling of packet sends
#include <iostream>
#include <thread> // this_thread::sleep_for

int eatPacket( CommsPacket packet )
{
    // make sense of packet
    if( packet.packet_type == CommsPacket::sim_telemetry )
    {
        cg::FI_panel.compass.update( packet.st_getAS_psi()*180/PI );
        cg::FI_panel.horizon.update( packet.st_getAS_theta()*180/PI, packet.st_getAS_phi()*180/PI );
        cg::FI_panel.airspeed.update( packet.st_getAS_u() );
        cg::FI_panel.altimeter.update( -packet.st_getAS_z() );
        cg::FI_panel.vsi.update( -packet.st_getAS_w() );
        cg::gauge_speed.updateVal( packet.st_getAS_u() );
    } else if( packet.packet_type == CommsPacket::video_packet ){
    /*
        streamer.receivePacket(packet);
        if (streamer.isFrameReady())
        {
          streamer.RXFrame(video_frame_raw, video_frame_size);

          UnloadImage(video_frame);
          video_frame = LoadImageFromMemory(".png", video_frame_raw, video_frame_size);

          UnloadTexture(video_texFrame);
          video_texFrame = LoadTextureFromImage(video_frame);
        }
        */
    } else if( packet.packet_type == CommsPacket::console_telemetry ){
        // an artifact from the Ground Vehicle
        cg::FI_panel.compass.update(packet.ct_getHeading());
        cg::FI_panel.horizon.update(packet.ct_getPitch(), packet.ct_getRoll());
        cg::E_panel.temp1.updateVal(packet.ct_getMotor1Temp());
        cg::E_panel.temp2.updateVal(packet.ct_getMotor2Temp());
        cg::E_panel.amp1.updateVal(packet.ct_getMotor1Amps());
        cg::E_panel.amp2.updateVal(packet.ct_getMotor2Amps());
        cg::gauge_speed.updateVal(packet.ct_getSpeed());
    } else {
        std::cerr << "Error: Received unknown packet type: "
                  << (int)packet.packet_type << std::endl;
    }
      return packet.packet_type;
}

bool sendCommand2Sim( CommsModule& comms_module, const Joystick& joy, int ap_mode )
{
    static CommsPacket tx_packet( CommsPacket::console_sim_command );
    static ControlSignal cs;
    joy.getControlInputs( cs );
    tx_packet.csc_setCS_throttle( cs.throttle );
    tx_packet.csc_setCS_elevator( cs.elevator );
    tx_packet.csc_setCS_aileron ( cs.aileron  );
    tx_packet.csc_setCS_rudder  ( cs.rudder   );
    tx_packet.csc_setCS_APMode( ap_mode );
    tx_packet.csc_setCS_resetSim( joy.btnPressed[1] );

    /*
      // DEBUG
      static uint8_t raw[32];
      tx_packet.getRaw( raw );
      std::cout<<"com2sim: ";
      for(int i=0; i<32; i++)
          std::cout<<(int)raw[i];
      std::cout<<"\n";
      // DEBUG END
      */

    return comms_module.sendPacket( tx_packet, CommsModule::sim_channel );
}

int main() {
  std::cout << "SOTT UAV Command Console v0.2\n";

  std::chrono::steady_clock clock;
  std::chrono::time_point<std::chrono::steady_clock> last_transmission_time;
  std::chrono::milliseconds transmission_interval(200);

  // joystick... because.. you know.. planes and stuff
  Joystick joy;

  // AutoPilot mode
  int ap_mode = 2;

  // ---- Comms Setup ----
  CommsModule comms_module(CommsModule::console_channel);
  VideoFeed streamer(3); // using resolution mode 3
  CommsPacket packet; // this one we use for the data we received
#ifndef TABLET_MODE
  // Serial connection ( to an arduino, it will relay packets thru rf24 )
  SerialManager serial;
#endif
  // ---- Comms Setup ----
  
  Toast toast({700, 0}, {500, 200});

  // graphics initialization
  const int screenWidth = 1900;
  const int screenHeight = 900;

  cg::InitWindowSafe(screenWidth, screenHeight, "Command Console");

  Image video_frame = GenImageColor(streamer.getResolutionWidth(),
                                    streamer.getResolutionHeight(), DARKGRAY);
  ImageDrawText(&video_frame, "No Signal", video_frame.width / 3.f,
                video_frame.height / 2.f, 20, RED);
  video_frame.mipmaps = 1;
  ImageFormat(&video_frame, PIXELFORMAT_UNCOMPRESSED_R8G8B8);

  std::uint8_t *video_frame_raw = new std::uint8_t[1024]{0};
  size_t video_frame_size;
  Texture2D video_texFrame;
  video_texFrame =
      LoadTextureFromImage(video_frame); // default no signal screen



  // main loop
  while (!WindowShouldClose()) {
    // event processing
    if (IsKeyPressed(KEY_ESCAPE)) {
      // ESC to close the application
      break;
    }

#ifndef TABLET_MODE
    // port selection, connection, reading and logging
    serial.update();
#endif

    // clear window for next frame
    BeginDrawing();     // ================ BEGIN DRAWING
    ClearBackground(Color{180, 180, 180, 255});

#ifndef TABLET_MODE
    // draw port selection and serial manager verbose output
    serial.drawMenu();
    // check incoming serial packets
    while( serial.getPacket(packet) )
    {
        eatPacket(packet);
    }
#endif

    // check incoming transmission packets, ALL OF THEM.
    while (comms_module.packetAvailable())
    {
      // read packet
      comms_module.readPacket(packet);

      /*
      // DEBUG
      std::cout<<"hellow!\n";
      static CommsPacket last_packet;
      static uint8_t raw[32];
      static uint8_t last_raw[32];
      packet.getRaw( raw );
      if( memcmp( raw, last_raw, sizeof(CommsPacket) ) == 0 )
          std::cout<<"got same packet\n";
      else {
          std::cout<<"prev: ";
          for(int i=0; i<32; i++)
              std::cout<<(int)last_raw[i];
          std::cout<<"\ncurr: ";
          for(int i=0; i<32; i++)
              std::cout<<(int)raw[i];
          std::cout<<"\n";
      }
      last_packet = packet;
      last_packet.getRaw( last_raw );
      // END OF DEBUG
      */

      //std::cout<<"eatPacket returned: "<<eatPacket(packet)<<"\n";
      eatPacket(packet);
    }

    toast.render();

    // render gauges
    cg::renderGauges();
    DrawTextureEx( video_texFrame, {200,50}, 0.f, 3.f, WHITE);

    // DEBUG
    joy.draw_debug();

    EndDrawing();       // ================ END DRAWING

    // TODO collect all inputs into the input manager
    joy.update();
    if( joy.buttonReleased(2) ) ap_mode = (ap_mode+1)%3;

    // Packet transmission
    if (clock.now() >= last_transmission_time + transmission_interval) {
      sendCommand2Sim( comms_module, joy, ap_mode );
      last_transmission_time = clock.now();
    }

    // some sleep time to stop hogging the cpu, maybe raylibs fps limiter
    // handles this but im adding it anyway
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  UnloadTexture(video_texFrame);
  UnloadImage(video_frame);
  CloseWindow();
  std::cout << "Exiting. Have a nice day\n";
  return 0;
}
