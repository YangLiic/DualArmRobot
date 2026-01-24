/****************************************************************
 * Inovance Servo - Standalone Version (No ROS)
 **/
#pragma once
#include "hw_can_usb.h"
#include <iostream>

namespace pg
{

class InovanceServo : public CanInterfaceUsb
{
private:
    uint32_t node_id_;
    bool motor_enabled_;
    
    void decode() override;
    
public:
    InovanceServo(const std::string& port_name, int baud_rate, uint32_t node_id = 0x601);
    ~InovanceServo();
    
    bool enableMotor();
    bool disableMotor();
    bool setVelocityMode();
    bool setVelocity(int32_t rpm);
    bool stopMotor();
    
    bool nmtStart();
    bool nmtPreOperational();
    
    bool isEnabled() const { return motor_enabled_; }
};

} // namespace pg
