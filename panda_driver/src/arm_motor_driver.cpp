#include "panda_driver/arm_motor_driver.h"

ArmMotorDriver::ArmMotorDriver(std::string cmd_topic_name) : ArmMotorDriver::MotorDriver()
{
    pub_command_ = nh_.advertise<std_msgs::Float64MultiArray>(cmd_topic_name, 100);

    sub_state_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 100, boost::bind(&ArmMotorDriver::subStateCallBack, this, _1));
}

ArmMotorDriver::~ArmMotorDriver()
{
}

void ArmMotorDriver::setMotorPositions(std_msgs::Float64MultiArray position)
{
    pub_command_.publish(position);
}

std::vector<double> ArmMotorDriver::getMotorPositions()
{
    return position_;
}

void ArmMotorDriver::subStateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{
    position_ = msg->position;
}