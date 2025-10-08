
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

#include "comms_module.hpp"

CommsModule::CommsModule( Type typ, Channel channel ): m_type(typ)
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

bool CommsModule::sendRequest( CommsPacket::PacketType req )
{
    // Implement request sending logic
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

    int result = select(m_socket + 1, &readfds, nullptr, nullptr, &timeout);
    return result > 0;  // Returns true if the socket is ready
}

bool CommsModule::readPacket( CommsPacket& p )
{
    ssize_t received_bytes = recvfrom(m_socket, &p, sizeof(CommsPacket), 0,
                                      (struct sockaddr*)&m_recv_addr, &m_recv_addr_len);

    if (received_bytes == sizeof(CommsPacket)) {
        return true;
    }
    return false;
}

