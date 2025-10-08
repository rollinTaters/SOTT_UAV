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

#include "central_command.hpp"
#include <array>
#include <iostream>
#include <thread>   // this_thread::sleep_for

bool drive_auto_mode_enabled = false;
bool turret_auto_mode_enabled = false;

CommsModule ccm_comms( CommsModule::udp, CommsModule::command_channel );

// lets define some parkour parameters
std::array<OperationMode, 4> legs =
{
    OperationMode{ 
            5,
            3,
            1 },
    OperationMode{ 
            5,
            3,
            1 },
    OperationMode{ 
            5,
            3,
            1 },
    OperationMode{ 
            5,
            3,
            1 }
};


int main()
{
    // what now..?

    // raw packet
    PacketBase raw_packet;

    // telemetry packets
    NGC_Telemetry_Packet ngctp;
    Drive_Telemetry_Packet1 dtp1;
    Drive_Telemetry_Packet2 dtp2;
    Turret_Packet ttp;  // turret telemetry packet

    // command packets
    NGC_Command_Packet ngccp;
    Turret_Packet tcp;  // turret control packet

    while( true )
    {
        std::this_thread::sleep_for( std::chrono::seconds(1) );

        // read received packets
        while( ccm_comms.packetAvailable() )
        {
            // TODO receive visual status from image processing module
            // TODO receive turret status
            // TODO receive drive motor status
            // TODO receive "pilot command" from console package
        }


        // TODO send "control mode" to ngc if necessary, otherwise send alive pulse
        // TODO send "turret control package" to turret, otherwise send alive pulse
        // TODO send pilot or "ngc drive command" to drive module

        if(ccm_comms.sendPacket( ngccp, CommsModule::ngc_channel ))
        {
            std::cout<<"CCM: ngc command packet sent succesfully\n";
        }
        // TODO forward received packets
        if( ccm_comms.sendPacket( dtp1, CommsModule::console_channel ))
        {
            std::cout<<"CCM: drive telemetry 1 packet sent succesfully\n";
        }
        // TODO forward received packets
        if( ccm_comms.sendPacket( dtp2, CommsModule::console_channel ))
        {
            std::cout<<"CCM: drive telemetry 2 packet sent succesfully\n";
        }
        // TODO forward the received ngc telemetry packet to the console
        if( ccm_comms.sendPacket( ngctp, CommsModule::console_channel ))
        {
            std::cout<<"CCM: ngc telemetry packet sent succesfully\n";
        }
        std::cout<<"\n";

        dtp1.data1 += 1;
        dtp1.data2 += 3;
        dtp1.data3 += 2;
        dtp2.data1 += 2;
        dtp2.data2 += 1;
        dtp2.data3 += 5;
        ngctp.data1 += 2;
        ngctp.data2 += 5;
        ngctp.data3 += 8;

    }
}

