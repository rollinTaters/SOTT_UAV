/*
	MIT License
	Copyright (c) 2025 rollinTaters
*/

#pragma once
#include <thread>

class TractionMotor
{
  public:
    TractionMotor();
    ~TractionMotor();
    TractionMotor( const TractionMotor& )  = delete;
    TractionMotor( const TractionMotor&& ) = delete;

    bool startControlLoop();
    bool stopControlLoop();

    bool setSpeed( float );   // (m/s)
    float getSpeed() const;         // (m/s)

  private:

    float m_target_speed = 0;   // m/s
    float m_current_speed = 0;  // m/s

    float m_error = 0;

    bool m_run_loop = false;
    void m_control_loop();
    std::thread *m_control_thread = nullptr;

};
