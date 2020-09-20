#include "panda_driver/trajectory_follow.h"

TrajectoryFollow::TrajectoryFollow(MotorDriver *driver)
    : motor_driver_(driver)
{
}

TrajectoryFollow::~TrajectoryFollow()
{
    delete motor_driver_;
}

void TrajectoryFollow::setGoalTrajectory(GoalHandle goal_handle)
{
    goal_handle_ = goal_handle;
}

void TrajectoryFollow::startTrajectoryFollow(GoalHandle goal_handle)
{
    if (!goal_handle.isValid())
    {
        ROS_ERROR("[TrajectoryFollow] in startTrajectoryFollow, goal_handle is not Valid!");
        goal_handle.setAborted();
        return;
    }

    setGoalTrajectory(goal_handle);
    state_ = Running;
    start_time_ = ros::Time::now();
    current_tra_index = 0;
}

void TrajectoryFollow::trajectoryFollow()
{
    switch (state_)
    {
    case Stop:
        /* code */
        break;

    case Running:
        /* code */
        {
            auto goal = goal_handle_.getGoal();
            auto points = goal->trajectory.points;

            //找到与当前时间对应的轨迹点
            auto current_time = ros::Time::now() - start_time_;
            while (current_tra_index < points.size() && current_time > points.at(current_tra_index).time_from_start)
            {
                current_tra_index++;
            }
            if (current_tra_index >= points.size())
            {
                state_ = Complete;
                break;
            }

            //线性插值
            TrajectoryPoint point;
            if (current_tra_index >= 1)
            {
                point = linearInterpolationTra(points.at(current_tra_index - 1),
                                               points.at(current_tra_index),
                                               current_time - points.at(current_tra_index - 1).time_from_start);
            }
            else
                point = points.at(current_tra_index);

            //设置关节点击位置
            std_msgs::Float64MultiArray arm_pos;
            for (int j = 0; j < point.positions.size(); j++)
            {
                double pos = point.positions[j];
                arm_pos.data.push_back(point.positions[j]);
            }

            motor_driver_->setMotorPositions(arm_pos);
        }
        break;

    case Complete:
        /* code */
        {
            ros::Duration all_time = ros::Time::now() - start_time_;
            std::cout << "\033[1;36m"
                      << "运行总时间 ms :" << all_time.sec * 1000 + all_time.nsec / 1000000 << std::endl;

            ros::Duration(0.3).sleep();
            goal_handle_.setSucceeded();
            state_ = Stop;

            ROS_INFO("\033[1;36m TrajectoryFollow is Succeed!");
        }
        break;

    default:
    {
        ROS_ERROR("\033[1;31m TrajectoryFollow state is error!!!!");
    }
    break;
    }
}

void TrajectoryFollow::stopTrajectoryFollow()
{
    state_ = Stop;
    if (goal_handle_.isValid())
    {
        goal_handle_.setCanceled();
        ROS_INFO("\033[1;36m TrajectoryFollow is Canceled!");
    }
    else
    {
        ROS_INFO("\033[1;36m goal_handle_ is not exist!");
    }
}

void TrajectoryFollow::spinOnce()
{
    trajectoryFollow();
}

TrajectoryFollow::TrajectoryPoint TrajectoryFollow::linearInterpolationTra(const TrajectoryPoint &start,
                                                                           const TrajectoryPoint &end,
                                                                           const ros::Duration &current_diff)
{
    auto total_diff = end.time_from_start - start.time_from_start;
    double total_diff_ms = total_diff.sec * 1000 + total_diff.nsec / 1000000;
    double current_diff_ms = current_diff.sec * 1000 + current_diff.nsec / 1000000;
    double scale = current_diff_ms / total_diff_ms;

    if (scale <= 0 || scale >= 1)
        return end;

    trajectory_msgs::JointTrajectoryPoint result;
    for (int i = 0; i < start.positions.size(); i++)
    {
        double tmp = start.positions.at(i) + (end.positions.at(i) - start.positions.at(i)) * scale;
        result.positions.push_back(tmp);
    }
    return std::move(result);
}