#include "panda_driver/gripper_motor_driver.h"

GripperMotorDriver::GripperMotorDriver(std::string cmd_topic_name) : GripperMotorDriver::MotorDriver()
{
    pub_command_ = nh_.advertise<std_msgs::Float64MultiArray>(cmd_topic_name, 100);

    sub_state_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 100, boost::bind(&GripperMotorDriver::subStateCallBack, this, _1));
}

GripperMotorDriver::~GripperMotorDriver()
{
}

void GripperMotorDriver::setMotorPositions(std_msgs::Float64MultiArray position)
{
    if (position.data.size() <= 0)
    {
        ROS_ERROR("position.data.size() is < 0");
        return;
    }
    else if (position.data.size() == 1)
    {
        double tmp = position.data.at(0);
        position.data.push_back(tmp);
    }

    pub_command_.publish(position);
}

std::vector<double> GripperMotorDriver::getMotorPositions()
{
    return position_;
}

void GripperMotorDriver::subStateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{
    position_ = msg->position;
}