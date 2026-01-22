/* 
   This is a class that manages the communications between different subsystems
   it manages the different underlying communication protocols for different systems.
   This is intended to be a global class that can be used everywhere in the codebase
*/

#pragma once
#define LOCALHOST

#include "comms_packets.hpp"
#include <iostream> // cerr
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class CommsModule
{
public:
    enum Channel{
        fc_channel = 54001,
        mc_channel = 54002,
        console_channel = 54003,
        sim_channel = 54004,
        random_channel1 = 54005,
        random_channel2 = 54006,
        undefined = 0
    };

    CommsModule( Channel );
    ~CommsModule();

    bool sendPacket( CommsPacket, Channel );
    bool packetAvailable();
    bool readPacket( CommsPacket& );

private:
    int m_socket;
    struct sockaddr_in m_addr;
    struct sockaddr_in m_recv_addr;
    socklen_t m_recv_addr_len;
};

