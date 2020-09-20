#pragma once

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64MultiArray.h>

#include "panda_driver/motor_driver.h"
#include "panda_driver/trajectory_follow.h"

namespace panda
{

    class ActionServer
    {
    public:
        using Action = control_msgs::FollowJointTrajectoryAction;
        using GoalHandle = actionlib::ServerGoalHandle<Action>;
        using Server = actionlib::ActionServer<Action>;

    public:
        ActionServer() = delete;
        // ActionServer(std::string server_name, std::string motor_name);
        ActionServer(std::string server_name, TrajectoryFollow *tra_follow);
        ~ActionServer();

        /*
    @ function:　开启服务
    */
        void startServer()
        {
            ROS_INFO("\033[1;36m arm action server is start");
            server_.start();
        }

        void goalCallBack(GoalHandle goal);
        void cancelCallBack(GoalHandle goal);

    private:
        /* data */
        ros::NodeHandle nh_;
        Server server_;                       //action server
        TrajectoryFollow *tarjectory_follow_; //轨迹跟踪 server
    };

    /*
@ function:     构造函数
@ server_name:  产生的action话题名字空间
@ tra_follow:   轨迹跟踪服务
*/
    ActionServer::ActionServer(std::string server_name, TrajectoryFollow *tra_follow)
        : server_(nh_, server_name, false), tarjectory_follow_(tra_follow)
    {
        server_.registerGoalCallback(boost::bind(&ActionServer::goalCallBack, this, _1));
        server_.registerCancelCallback(boost::bind(&ActionServer::cancelCallBack, this, _1));
    }

    ActionServer::~ActionServer()
    {
    }

    void ActionServer::goalCallBack(GoalHandle goal_handle)
    {
        ROS_INFO("\033[1;36m -------arm goal is Accepted!-------");
        goal_handle.setAccepted();

        tarjectory_follow_->startTrajectoryFollow(goal_handle);
    }

    void ActionServer::cancelCallBack(GoalHandle goal)
    {
        ROS_INFO("panda_arm_controller/follow_joint_trajectory: Cancel");
        tarjectory_follow_->stopTrajectoryFollow();
        // goal.setCanceled();
    }

} // namespace panda
