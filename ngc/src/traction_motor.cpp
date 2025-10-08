/*
	MIT License
	Copyright (c) 2025 rollinTaters
*/

#include "traction_motor.hpp"


TractionMotor::TractionMotor()
{
}

TractionMotor::~TractionMotor()
{
    stopControlLoop();
}

bool TractionMotor::startControlLoop()
{
    if( m_control_thread != nullptr )
        return false;
    m_run_loop = true;
    m_control_thread = new std::thread( &TractionMotor::m_control_loop, this );
    return true;
}

bool TractionMotor::stopControlLoop()
{
    if( m_control_thread == nullptr )
        return false;
    m_run_loop = false;
    m_control_thread->join();
    m_control_thread = nullptr;
    return true;
}


bool TractionMotor::setSpeed( float target )
{
    m_target_speed = target;
    return true;
}

float TractionMotor::getSpeed() const
{
    return m_current_speed;
}

void TractionMotor::m_control_loop()
{
    while( m_run_loop )
    {
        // TODO read a sensor to get the current speed
        m_error = m_target_speed - m_current_speed;

        // FIXME im gonna hard set this for now
        m_current_speed = m_target_speed;

        // TODO maybe a PID
        
        // TODO this is where we do GPIO magic
    }
}
