#include "serial_manager.hpp"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard function definitions

SerialPort::SerialPort(const char* portName)
{
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
    } else {
        struct termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, B115200); // Set baud rate
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB; // No parity
        options.c_cflag &= ~CSTOPB; // 1 stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;     // 8 data bits
        tcsetattr(fd, TCSANOW, &options);
        fcntl(fd, F_SETFL, FNDELAY); // Non-blocking reads
    }
}

SerialPort::~SerialPort() { if (fd != -1) ::close(fd); }

int SerialPort::readSerial(char* buffer, int size) {
    return ::read(fd, buffer, size);
}

int SerialPort::writeSerial(const char* data) {
    return ::write(fd, data, strlen(data));
}





#include "raylib.h"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

void SerialManager::OpenLogFile()
{
    if (logFile.is_open()) {
        logFile.flush();
        logFile.close();
    }

    // 1. Ensure the "logs" directory exists
    std::string folderName = "logs";
    try {
        if (!fs::exists(folderName)) {
            fs::create_directory(folderName);
        }
    } catch (const fs::filesystem_error& e) {
        // If we can't create the folder (e.g. permission issues), 
        // we might want to log this to console
        TraceLog(LOG_ERROR, "SERIAL: Could not create logs directory: %s", e.what());
    }
    
    // 2. Prepare the timestamp
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&in_time_t);

    // 3. Construct path: logs/log_YYYYMMDD_HHMM.txt
    std::ostringstream ss;
    ss << folderName << "/log_" << std::put_time(&bt, "%Y%m%d_%H%M") << ".txt";
    
    // 4. Open the file
    logFile.open(ss.str(), std::ios::app);

    if (logFile.is_open()) {
        logFile << "--- New Log Block Started at " << GetTimestamp() << " ---" << std::endl;
        TraceLog(LOG_INFO, "SERIAL: Logging to %s", ss.str().c_str());
    } else {
        TraceLog(LOG_WARNING, "SERIAL: Failed to open log file for writing!");
    }
}

std::string SerialManager::GetTimestamp()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = system_clock::to_time_t(now);

    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << "[" << std::put_time(&bt, "%Y-%m-%d %H:%M:%S") 
        << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    return oss.str();
}


SerialManager::SerialManager(
        int menuX, int menuY ): menu_x(menuX), menu_y(menuY), menu_w(450), menu_h(180)
{ refreshPorts(); }
SerialManager::~SerialManager() { disconnect(); }

void SerialManager::refreshPorts()
{
    availablePorts.clear();
    for (const auto& entry : fs::directory_iterator("/dev")) {
        std::string path = entry.path().string();
        if (path.find("ttyACM") != std::string::npos || 
            path.find("ttyUSB") != std::string::npos) {
            availablePorts.push_back(path);
        }
    }
}

std::vector<std::string> SerialManager::getAvailablePorts() const
{
    return availablePorts;
}

bool SerialManager::connectTo( std::string inp_port )
{
    port = new SerialPort(inp_port.c_str());
    if (port->fd != -1) {
        isConnected = true;
        activePortName = inp_port;
        return true;
    } else {
        delete port;
        isConnected = false;
        activePortName = "";
    }
    return false;
}

void SerialManager::disconnect()
{
    if (port) {
        delete port;
        port = nullptr;
    }
    isConnected = false;
    activePortName = "";
}

// Call this in Update loop
void SerialManager::update()
{
    if (!isConnected) {
        if (IsKeyPressed(KEY_R)) refreshPorts();
        
        Vector2 mouse = GetMousePosition();
        hoveredIndex = -1;

        for (auto i = 0; i < availablePorts.size(); i++) {
            Rectangle rect = { menu_x+50, (float)(menu_y+100 + i * 40), 300, 30 };
            if (CheckCollisionPointRec(mouse, rect)) {
                hoveredIndex = i;
                if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                    connectTo( availablePorts[i].c_str() );
                }
            }
        }
        if (isConnected) OpenLogFile(); // Open log when connected
    } else {
        // logging, and updating log file
        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t);

        // Calculate a unique ID for every 30-minute block in a day (0 to 47)
        int checkBlock = (now->tm_hour * 2) + (now->tm_min >= 30 ? 1 : 0);

        // We also check tm_mday to ensure we rotate if the day changes at midnight
        if (checkBlock != currentHalfHourBlock) {
            currentHalfHourBlock = checkBlock;
            OpenLogFile(); // Creates a new file with the current timestamp
            
            // Optional: Print to console so you know rotation happened
            TraceLog(LOG_INFO, "SerialManager: Rotating log file for new 30-minute block.");
        }
        // end of log file updating

        // READ LOGIC: Process incoming data
        char temp[128];
        int bytesRead = port->readSerial(temp, sizeof(temp) - 1);
        
        if (bytesRead > 0) {
            temp[bytesRead] = '\0';
            internalBuffer += temp;

            // Process lines to keep log clean and memory low
            size_t nPos;
            while ((nPos = internalBuffer.find('\n')) != std::string::npos) {
                std::string line = internalBuffer.substr(0, nPos);
                
                // Clean carriage returns if coming from Arduino's println
                if (!line.empty() && line.back() == '\r') line.pop_back();

                std::string timestampedLine = GetTimestamp() + line;

                // 1. Update UI (we show the timestamped version)
                lastReceivedLine = timestampedLine; 
                
                // 2. Write to Disk
                if (logFile.is_open()) {
                    logFile << timestampedLine << std::endl;
                    logFile.flush(); 
                }

                internalBuffer.erase(0, nPos + 1);
            }
        }
    }
}

// Call this in Draw loop
void SerialManager::drawMenu()
{
    DrawRectangle( menu_x, menu_y, menu_w, menu_h, Fade(BLACK, 0.9f) );
    if (!isConnected) {
        DrawText("Select Serial Port (R to refresh):", menu_x+50, menu_y+70, 20, GRAY);
        for (auto i = 0; i < availablePorts.size(); i++) {
            Color col = (i == hoveredIndex) ? LIME : LIGHTGRAY;
            DrawRectangleLines(menu_x+50, menu_y+100 + i * 40, 300, 30, col);
            DrawText(availablePorts[i].c_str(), menu_x+60, menu_y+107 + i * 40, 18, col);
        }
    } else {
        DrawText(TextFormat("Connected: %s", activePortName.c_str()), menu_x+10, menu_y+10, 15, GREEN);
        DrawText(TextFormat("Latest: %s", lastReceivedLine.c_str()), menu_x+10, menu_y+30, 20, LIME);
        DrawText("Logging to file...", menu_x+10, menu_y+55, 15, GRAY);
    }
}

void SerialManager::sendData(const char* data)
{
    if (isConnected && port) port->writeSerial(data);
}

