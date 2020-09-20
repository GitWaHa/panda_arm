#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include "panda_driver/motor_driver.h"

class GripperMotorDriver : public MotorDriver
{

public:
    GripperMotorDriver() = delete;
    GripperMotorDriver(std::string cmd_topic_name);
    virtual ~GripperMotorDriver();

    virtual void setMotorPositions(std_msgs::Float64MultiArray position);

    virtual std::vector<double> getMotorPositions();

private:
    void subStateCallBack(const sensor_msgs::JointState::ConstPtr &msg);

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher pub_command_; //发送关节位置
    ros::Subscriber sub_state_;  //获取关节位置
    std::vector<double> position_;
};
