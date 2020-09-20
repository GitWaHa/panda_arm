#include "panda_driver/action_server.h"
#include "panda_driver/trajectory_follow.h"
#include "panda_driver/arm_motor_driver.h"
#include "panda_driver/gripper_motor_driver.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_action_server");

    // 多线程spin
    ros::AsyncSpinner spiner(4);
    spiner.start();

    // arm
    MotorDriver *arm_driver = new ArmMotorDriver("panda_arm_controller/command");
    TrajectoryFollow arm_tarjectory_follow(arm_driver);
    panda::ActionServer arm_action("panda_arm_controller/follow_joint_trajectory", &arm_tarjectory_follow);
    arm_action.startServer();

    // gripper
    MotorDriver *gripper_driver = new GripperMotorDriver("panda_gripper_controller/command");
    TrajectoryFollow gripper_tarjectory_follow(gripper_driver);
    panda::ActionServer gripper_action("panda_gripper_controller/follow_joint_trajectory", &gripper_tarjectory_follow);
    gripper_action.startServer();

    ros::Time start = ros::Time::now();
    int count = 0;
    ros::Rate rate(1000);
    while (ros::ok())
    {
        arm_tarjectory_follow.spinOnce();
        gripper_tarjectory_follow.spinOnce();

        // ros::spinOnce();
        rate.sleep();

        /* 帧率打印 */
        // count++;
        // if ((ros::Time::now() - start) > ros::Duration(1))
        // {
        //     std::cout << "\033[1;32m"
        //               << "帧率 :" << count << std::endl;
        //     start = ros::Time::now();
        //     count = 0;
        // }
    }

    return 0;
}