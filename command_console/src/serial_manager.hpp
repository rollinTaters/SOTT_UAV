#pragma once

class SerialPort
{
public:
    int fd;
    SerialPort(const char* portName);
    ~SerialPort();

    int readSerial(char* buffer, int size);
 
    int writeSerial(const char* data);

    bool isValid();
};

#include <vector>
#include <queue>
#include <string>
#include <fstream>
#include "../../common_code/src/comms_packets.hpp"

class SerialManager
{
private:
    // actual goddamn port
    SerialPort* port = nullptr;
    // storage
    std::vector<std::string> availablePorts;
    int hoveredIndex = -1;
    std::string internalBuffer = "";
    std::queue<CommsPacket> packet_q;
    // logging
    std::ofstream logFile;
    int currentHalfHourBlock = -1;
    // visuals
    int menu_x, menu_y, menu_w, menu_h;


    void OpenLogFile();

    std::string GetTimestamp();

    void handleDisconnect();

public:
    bool isConnected = false;
    std::string activePortName = "";
    std::string lastReceivedLine = "";

    SerialManager( int menuX = 10, int menuY = 730 );
    ~SerialManager();

    void refreshPorts();

    std::vector<std::string> getAvailablePorts() const; // does NOT refresh ports automantically

    bool connectTo( std::string inp_port );

    void disconnect();

    // Call this in Update loop, also calls readData()
    void update();

    // Call this in Draw loop
    void drawMenu();

    // sends given bytes, also does logging
    void sendData(const char* data);

    // reads serial buffer and populates internal packet queue, also does logging
    void readData();

    // returns false if internal queue is empty
    bool getPacket( CommsPacket& );
};
