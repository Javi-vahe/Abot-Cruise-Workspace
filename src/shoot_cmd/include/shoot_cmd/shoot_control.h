#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#include <string>
#include <vector>
#include "ros/ros.h"
#include "SerialPort.h"
#include "std_msgs/String.h"


class Shoot_Control
{
private:
    SerialPort s;
public:
    Shoot_Control();
    ~Shoot_Control();
    bool init(const char* port_name, int baudrate); 
    void mSleep(int mtime);
    void uSleep(int utime);
    void shoot();
    void stop_shoot(); 
};

#endif
