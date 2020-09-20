#pragma once

#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64MultiArray.h>

#include "panda_driver/motor_driver.h"

class TrajectoryFollow
{
public:
    using Action = control_msgs::FollowJointTrajectoryAction;
    using GoalHandle = actionlib::ServerGoalHandle<Action>;

    using TrajectoryPoint = trajectory_msgs::JointTrajectoryPoint;

    enum State
    {
        Stop,
        Running,
        Complete,
    };

public:
    TrajectoryFollow() = delete;
    TrajectoryFollow(MotorDriver *driver);
    ~TrajectoryFollow();

    void setGoalTrajectory(GoalHandle goal_handle);

    void startTrajectoryFollow(GoalHandle goal_handle);
    void trajectoryFollow();
    void stopTrajectoryFollow();

    void spinOnce();

private:
    TrajectoryPoint linearInterpolationTra(const TrajectoryPoint &start,
                                           const TrajectoryPoint &end,
                                           const ros::Duration &current_diff);

private:
    /* data */
    MotorDriver *motor_driver_;
    GoalHandle goal_handle_;
    State state_ = Stop;

    ros::Time start_time_;
    std::size_t current_tra_index;
};
