/*
        MIT License

        Copyright (c) 2025 rollinTaters, guvenchemy

*/

#include "../../common_code/src/comms_module.hpp"
#include "../../common_code/src/video_feed.hpp"
#include "console_graphics.hpp"
#include "raylib.h"
#include "toast.hpp"
#include <chrono> // throttling of packet sends
#include <iostream>
#include <thread> // this_thread::sleep_for

int main() {
  std::cout << "SOTT UAV Command Console v0.2\n";

  std::chrono::steady_clock clock;
  std::chrono::time_point<std::chrono::steady_clock> last_transmission_time;
  std::chrono::milliseconds transmission_interval(200);

  // create communications module
  CommsModule comms_module(CommsModule::udp, CommsModule::console_channel);
  VideoFeed streamer(3); // using resolution mode 3
  
  Toast toast({700, 0}, {500, 200});
  

  // declare dummy packets
  CommsPacket packet; // this one we use for the data we received
  CommsPacket packet2send(
      CommsPacket::console_command); // this one we use for sending console
                                     // packets

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


  // DEBUG
  float p{0.f},r{0.f},h{0.f},a{0.f},ias{0.f},vsi{0.f};

  // main loop
  while (!WindowShouldClose()) {
    // event processing
    if (IsKeyPressed(KEY_ESCAPE)) {
      // ESC to close the application
      break;
    }
    // clear window for next frame
    BeginDrawing();
    ClearBackground(Color{180, 180, 180, 255});

    // press t for debug test
    if (IsKeyDown(KEY_T)) {
      cg::DEBUG_gauge_test();
    }
    // DEBUG
    // Mock data updates
    if (IsKeyDown(KEY_W)) p -= 0.5f;
    if (IsKeyDown(KEY_S)) p += 0.5f;
    if (IsKeyDown(KEY_A)) r += 0.5f;
    if (IsKeyDown(KEY_D)) r -= 0.5f;
    if (IsKeyDown(KEY_Q)) h -= 0.5f;
    if (IsKeyDown(KEY_E)) h += 0.5f;
    if (IsKeyDown(KEY_UP)) a += 5.0f;
    if (IsKeyDown(KEY_DOWN)) a -= 5.0f;
    if (IsKeyDown(KEY_R)) ias += 5.0f;
    if (IsKeyDown(KEY_F)) ias -= 5.0f;
    if (IsKeyDown(KEY_T)) vsi += 5.0f;
    if (IsKeyDown(KEY_G)) vsi -= 5.0f;

    cg::FI_panel.horizon.update(p, r);
    cg::FI_panel.compass.update(h);
    cg::FI_panel.altimeter.update(a);
    cg::FI_panel.airspeed.update(ias);
    cg::FI_panel.vsi.update(vsi);
    // DEBUG

    // check incoming transmission packets, ALL OF THEM.
    while (comms_module.packetAvailable())
    {
      // read packet
      comms_module.readPacket(packet);

      // make sense of packet
      switch (packet.packet_type)
      {
      case CommsPacket::console_telemetry:
        cg::FI_panel.compass.update(packet.ct_getHeading());
        cg::FI_panel.horizon.update(packet.ct_getPitch(), packet.ct_getRoll());
        cg::E_panel.temp1.updateVal(packet.ct_getMotor1Temp());
        cg::E_panel.temp2.updateVal(packet.ct_getMotor2Temp());
        cg::E_panel.amp1.updateVal(packet.ct_getMotor1Amps());
        cg::E_panel.amp2.updateVal(packet.ct_getMotor2Amps());
        cg::gauge_speed.updateVal(packet.ct_getSpeed());
        break;

      case CommsPacket::console_command:
        std::cerr << " why are we receiving a console command packet when we "
                     "should be the one sending it?\n";
        break;

      case CommsPacket::video_packet:
        streamer.receivePacket(packet);
        /*// DEBUG        ----------------
        std::cout<<"received video packet. frameID:
        "<<streamer.getCurrentFrameID()<<"\n"; for( auto d : packet.data )
        {
            std::cout<< (int)d <<" ";
        }
        std::cout<<"\n";
        // DEBUG END    ---------------- */

        if (streamer.isFrameReady())
        {
          streamer.RXFrame(video_frame_raw, video_frame_size);
          /*// DEBUG        ----------------
          std::cout<<"Command Console Main.cpp:\nRX frame size: "
              <<video_frame_size<<"\n";
          std::cout<<"raw frames:\n";
          for( int i = 0; i < video_frame_size; i++ )
          {
              std::cout<< (int)( video_frame_raw[i] ) << " ";
          }
          std::cout<<"\n";
          // DEBUG END    ---------------- */

          // ImageFormat( &video_frame, PIXELFORMAT_UNCOMPRESSED_R8G8B8 );  //
          // revert back to 3 channels
          UnloadImage(video_frame);
          video_frame =
              LoadImageFromMemory(".png", video_frame_raw, video_frame_size);

          // because there is a bug in raylib and UpdateTexture function only
          // accepts this format
          // ImageFormat( &video_frame, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 );
          // Color* pixels = LoadImageColors( video_frame );
          // UpdateTexture( video_texFrame, pixels );

          UnloadTexture(video_texFrame);
          video_texFrame = LoadTextureFromImage(video_frame);
        }
        break;

      case CommsPacket::undefined:
        std::cerr << "Warning: Received undefined packet type" << std::endl;
        break;

      default:
        std::cerr << "Error: Received unknown packet type: "
                  << (int)packet.packet_type << std::endl;
        break;
      }
    }


    toast.render();

    // render gauges
    cg::renderGauges();
    DrawTextureEx( video_texFrame, {200,50}, 0.f, 3.f, WHITE);

    EndDrawing();

    // process input
    cg::input.processInput(packet2send);

    if (clock.now() >= last_transmission_time + transmission_interval) {
      // for testing purposes we send it to ngc, normally we wanna send to ccm
      comms_module.sendPacket(packet2send, CommsModule::ngc_channel);
      last_transmission_time = clock.now();

      /*// DEBUG
      std::cout<<"we be sending this data: \n";
      for( auto d: packet2send.data )
      {
          std::cout<<(int)d<<" ";
      }
      std::cout<<"\n";
      // DEBUG END*/
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
