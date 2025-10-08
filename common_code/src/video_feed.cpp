#include "video_feed.hpp"
#include <iostream> // cerr
#include <cstdlib>
#include <cstring> 

VideoFeed::FrameBuffer::FrameBuffer( size_t frame_size )
{
    data_size = frame_size;
    num_of_chunks = (frame_size + (chunk_size-1)) / chunk_size;
    data = new std::uint8_t[ frame_size ]{0};
    chunksReceived = new bool[ num_of_chunks ]{false};

    /*
    std::cout<<"frame size: "<<frame_size
        <<"\tfs/cs: "<< ((float)frame_size/(float)chunk_size)
        <<"num chunks: "<<num_of_chunks<<"\n";
        */
}

VideoFeed::FrameBuffer::~FrameBuffer()
{
    delete data;
    delete chunksReceived;
}

VideoFeed::VideoFeed(uint16_t frameWidth, uint16_t frameHight){
    m_height = frameHight;
    m_width = frameWidth;
    m_frameID = 0;
}

VideoFeed::VideoFeed( int resolution_mode )
{
    switch( resolution_mode )
    {
        case 1:
            m_height = resolution_1_y;
            m_width = resolution_1_x;
            break;
        case 2:
            m_height = resolution_2_y;
            m_width = resolution_2_x;
            break;
        case 3:
            m_height = resolution_3_y;
            m_width = resolution_3_x;
            break;
        case 4:
            m_height = resolution_4_y;
            m_width = resolution_4_x;
            break;
        case 5:
            m_height = resolution_5_y;
            m_width = resolution_5_x;
            break;
        default:
            std::cerr<<"Error: VideoFeed object got bullshit resolution mode\n";
    }
    m_frameID = 0;
}

int VideoFeed::getResolutionWidth() const { return m_width; }
int VideoFeed::getResolutionHeight() const { return m_height; }
uint16_t VideoFeed::getCurrentFrameID() const { return m_frameID; }


//transmit data

void VideoFeed::TXFrame( void* image_data, size_t size )
{
    m_frameID++;

    if( m_TX_buffer == nullptr ) delete m_TX_buffer;
    m_TX_buffer = new FrameBuffer( size );
    
    m_TX_buffer->frameID = m_frameID;

    std::memcpy( m_TX_buffer->data, image_data, size );

    // split tx buffer into comms packets
    m_TX_packets.clear();
    size_t totalChunks = m_TX_buffer->num_of_chunks;

    for(size_t chunkID = 0; chunkID < totalChunks; chunkID++)
    {
        CommsPacket vdp( CommsPacket::video_packet );
        vdp.setChunkID(chunkID);
        vdp.setFrameID(m_frameID);
        vdp.setFrameSize( m_TX_buffer->data_size );

        std::array<uint8_t, FrameBuffer::chunk_size> payload{};

        size_t offset = chunkID * FrameBuffer::chunk_size;
        size_t copySize = std::min(FrameBuffer::chunk_size, size - offset);

        std::memcpy( payload.data(), &(m_TX_buffer->data)[offset], copySize );
        vdp.setPayload(payload);
        m_TX_packets.push_back(vdp);
    }
}

const std::vector<CommsPacket>& VideoFeed::getTXPackets() const { return m_TX_packets; }



//
//recieve data
//

void VideoFeed::receivePacket(const CommsPacket& vdp)
{
    // check if given packet is indeed a video packet
    if( vdp.packet_type != CommsPacket::video_packet ) return;

    //get identifying data from packet
    uint16_t packetFrameID = vdp.getFrameID();
    uint16_t chunkID = vdp.getChunkID();
    size_t frame_size = vdp.getFrameSize();

    // make sure we have a rx frame buffer
    if( m_RX_buffer == nullptr ) m_RX_buffer = new FrameBuffer( frame_size );

    //if we are receiving a new frame, refresh rx buffer
    if (packetFrameID != m_RX_buffer->frameID)
    {
        if( m_RX_buffer != nullptr ) delete m_RX_buffer;
        m_RX_buffer = new FrameBuffer( frame_size );
        m_RX_buffer->frameID = packetFrameID;
        m_frameID = packetFrameID;
    }
    size_t offset = chunkID * (m_RX_buffer->chunk_size);
    auto payload = vdp.getPayload();

    size_t copySize = std::min<size_t>( m_RX_buffer->chunk_size, frame_size - offset);
    std::memcpy( &((m_RX_buffer->data)[offset]), payload.data(), copySize);

    /*// DEBUG        ------------
    std::cout<<"\n\n----\nreceivePacket:\n";
    std::cout<<"frameID: "<<packetFrameID<<
        " chunks received: "<<chunkID+1<<"/"<<m_RX_buffer->num_of_chunks<<
        " frame_size: "<<frame_size<<"\n"
        <<"status of m_RX_buffer->data:\n";
    for( int i = 0; i < m_RX_buffer->data_size; i++ )
    {
        std::cout<< (int)( (m_RX_buffer->data)[i] ) <<" ";
    }
    std::cout<<"\n----\n";
    // DEBUG END    ------------    */

    m_RX_buffer->chunksReceived[chunkID] = true;
}

bool VideoFeed::isFrameReady() const
{
    if( m_RX_buffer == nullptr ) return false;

    for ( size_t i = 0; i < m_RX_buffer->num_of_chunks; i++ ){
        if ( !(m_RX_buffer->chunksReceived)[i] ){
            return false;
        }
    }
    return true;
    // check chunks 
}

void VideoFeed::RXFrame( std::uint8_t* ptr, size_t &size )
{
    if( m_RX_buffer == nullptr )
    {
        std::cout<<"newFrame method does not have a valid m_RX_buffer to create new frame\n";
        return;
    }
    //if( ptr != nullptr ) delete ptr;  // this causes double free

    // and this probably never triggers, however im too damn tired to give any fucks right now
    if( ptr == nullptr ) ptr = new std::uint8_t[ m_RX_buffer->data_size ];

    std::memcpy( ptr, m_RX_buffer->data, m_RX_buffer->data_size );
    size = m_RX_buffer->data_size;

    /*// DEBUG        ------------
    std::cout<<"\n\n----\nRXFrame:\n"
        <<"status of ptr:\n";
    for( int i = 0; i < m_RX_buffer->data_size; i++ )
    {
        std::cout<< (int)( ptr[i] ) <<" ";
    }
    std::cout<<"\n----\n";
    // DEBUG END    ------------ */
}
