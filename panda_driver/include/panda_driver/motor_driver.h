#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

class MotorDriver
{

public:
    MotorDriver(){};
    virtual ~MotorDriver(){};

    virtual void setMotorPositions(std_msgs::Float64MultiArray position) = 0;
    virtual std::vector<double> getMotorPositions() = 0;
};
