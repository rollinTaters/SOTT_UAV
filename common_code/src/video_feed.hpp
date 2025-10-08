#pragma once
#include "comms_packets.hpp"
#include <vector>
#include <cstdint>


class VideoFeed
{
    public:
        VideoFeed(uint16_t frameWidth, uint16_t frameHight);
        VideoFeed( int resolution_mode );

        static constexpr int resolution_1_x = 1920/2;
        static constexpr int resolution_1_y = 1080/2;
        static constexpr int resolution_2_x = 1920/4;
        static constexpr int resolution_2_y = 1080/4;
        static constexpr int resolution_3_x = 1920/8;
        static constexpr int resolution_3_y = 1080/8;
        static constexpr int resolution_4_x = 1920/32;
        static constexpr int resolution_4_y = 1080/32;
        static constexpr int resolution_5_x = 1920/64;
        static constexpr int resolution_5_y = 1080/64;

        int getResolutionWidth() const;
        int getResolutionHeight() const;
        uint16_t getCurrentFrameID() const;

        //transmit data
        void TXFrame( void* image_data, size_t size );  // populates m_TX_packets
        const std::vector<CommsPacket>& getTXPackets() const;   // packets to be transmitted

        //recieve data
        void receivePacket(const CommsPacket& packet);
        bool isFrameReady() const;
        void RXFrame( std::uint8_t* ptr, size_t &size );

    private:
        // common 
        uint16_t m_width;
        uint16_t m_height;
        uint16_t m_frameID;

        //transmit
        //std::vector<uint8_t> m_outgoingFrameBytes;
        std::vector<CommsPacket> m_TX_packets;

        //recieve
        struct FrameBuffer{
            FrameBuffer( size_t data_size );
            ~FrameBuffer();
            FrameBuffer( FrameBuffer& ) = delete;

            static constexpr size_t chunk_size = 25; // num of bytes in a chunk of transmission (packet size)
            size_t num_of_chunks;   // num of chunks in this frame
            size_t data_size;       // num of bytes in this frames data

            std::uint8_t *data = nullptr;
            bool *chunksReceived = nullptr;
            std::uint16_t frameID = 0;

        };

        FrameBuffer* m_RX_buffer = nullptr;
        FrameBuffer* m_TX_buffer = nullptr;

};
