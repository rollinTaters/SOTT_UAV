#include "comms_module.hpp"
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <vector>

CommsModule::CommsModule(const char* shm_name)
    : shm_name(shm_name), packet_size(32), buffer_len(64), shm_fd(-1), ptr(nullptr)
{
    // Create shared memory
    shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    size_t total_size = packet_size * buffer_len; // Total size for all messages
    ftruncate(shm_fd, total_size);
    ptr = mmap(0, total_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    current_index = 0;
}

CommsModule::~CommsModule() {
    if (ptr) {
        munmap(ptr, packet_size * buffer_len);
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_unlink(shm_name);
    }
}

bool CommsModule::sendPacket(const CommsPacket& packet, Channel )
{
    size_t offset = current_index * packet_size;
    memcpy(static_cast<uint8_t*>(ptr) + offset, packet.getRaw(), packet_size);
    current_index = (current_index + 1) % buffer_len; // Circular buffer
}

bool CommsModule::packetAvailable()
{
}

bool CommsModule::readPacket( CommsPacket& packet )
{
    std::vector<std::string> messages;
    for (size_t i = 0; i < buffer_len; ++i) {
        size_t offset = (current_index + i) % buffer_len;
        messages.push_back(std::string(static_cast<char*>(ptr) + offset * packet_size));
    }
    return messages;
}


