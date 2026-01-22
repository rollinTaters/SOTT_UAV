#include "comms_module.hpp"

CommsModule::CommsModule( Channel channel )
{
    // Create socket for UDP communication
    if ((m_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Comms Module: failed to create socket\n";
        return;
    }

    m_addr.sin_family = AF_INET;
    m_addr.sin_port = htons(channel); // Use channel as port number
    m_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all interfaces

    if (bind(m_socket, (struct sockaddr*)&m_addr, sizeof(m_addr)) < 0) {
        std::cerr << "Comms Module: failed to bind socket to port " << channel << "\n";
        close(m_socket);
    }

    m_recv_addr_len = sizeof(m_recv_addr);
}

CommsModule::~CommsModule()
{
    close(m_socket);
}

bool CommsModule::sendPacket( CommsPacket p, Channel c )
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(c);  // Use channel as port number
    dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");  // Localhost

    ssize_t sent_bytes = sendto(m_socket, &p, sizeof(CommsPacket), 0,
                                (struct sockaddr*)&dest_addr, sizeof(dest_addr));

    if (sent_bytes == sizeof(CommsPacket)) {
        return true;
    }
    return false;
}

bool CommsModule::packetAvailable()
{
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms timeout

    FD_ZERO(&readfds);
    FD_SET(m_socket, &readfds);

    // Use select to wait for data
    int result = select(m_socket + 1, &readfds, nullptr, nullptr, &timeout);
    //return result > 0;  // Returns true if the socket is ready
    
    if (result > 0 && FD_ISSET(m_socket, &readfds)) {
        char buffer[sizeof(CommsPacket)];
        ssize_t peeked_bytes = recv(m_socket, buffer, sizeof(buffer), MSG_PEEK);

        // Check for full packet
        return (peeked_bytes == sizeof(CommsPacket));
    }
    return false;  // No data or timeout
}

bool CommsModule::readPacket( CommsPacket& p )
{
    ssize_t received_bytes = recvfrom(
                m_socket,
                &p,
                sizeof(CommsPacket),
                0,
                (struct sockaddr*)&m_recv_addr,
                &m_recv_addr_len );

    if (received_bytes == sizeof(CommsPacket))
        return true;
    return false;
}


////////////////////////////////////
/*

bool CommsModule::packetAvailable()
{
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms timeout

    FD_ZERO(&readfds);
    FD_SET(m_socket, &readfds);

    // Use select to wait for data
    int result = select(m_socket + 1, &readfds, nullptr, nullptr, &timeout);

    // If there is data, check if we can read it
    if (result > 0 && FD_ISSET(m_socket, &readfds)) {
        char buffer[sizeof(CommsPacket)];
        ssize_t received_bytes = recvfrom(m_socket, buffer, sizeof(buffer), MSG_DONTWAIT, 
                                          (struct sockaddr*)&m_recv_addr, &m_recv_addr_len);

        // If we receive a valid packet, store it
        if (received_bytes == sizeof(CommsPacket)) {
            memcpy(&lastReceivedPacket, buffer, sizeof(CommsPacket)); // Store received packet
            return true; // Indicate that a new packet is available
        }
    }

    return false; // No data available
}

bool CommsModule::readPacket( CommsPacket& p )
{
    // Copy the last received packet to the provided reference
    p = lastReceivedPacket;
    
    // Return true if the last received packet is valid
    return (memcmp(&lastReceivedPacket, &p, sizeof(CommsPacket)) == 0);
}

*/
