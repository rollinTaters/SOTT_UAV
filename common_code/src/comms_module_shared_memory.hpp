#pragma once

#include "comms_packets.hpp"

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

    bool sendPacket( const CommsPacket&, Channel );
    bool packetAvailable();
    bool readPacket( CommsPacket& );

private:
    const char* shm_name;
    size_t packet_size;
    size_t buffer_len;
    int shm_fd;
    void* ptr;
    size_t current_index; // Tracks the index for writing
};

