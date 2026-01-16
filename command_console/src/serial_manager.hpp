#pragma once

class SerialPort
{
public:
    int fd;
    SerialPort(const char* portName);
    ~SerialPort();

    int readSerial(char* buffer, int size);
 
    int writeSerial(const char* data);

};

#include <vector>
#include <string>
#include <fstream>

class SerialManager
{
private:
    SerialPort* port = nullptr;
    std::vector<std::string> availablePorts;
    int hoveredIndex = -1;
    std::string internalBuffer = "";
    std::ofstream logFile;
    int currentHalfHourBlock = -1;
    int menu_x, menu_y, menu_w, menu_h;

    void OpenLogFile();

    std::string GetTimestamp();

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

    // Call this in Update loop
    void update();

    // Call this in Draw loop
    void drawMenu();

    void sendData(const char* data);
};
