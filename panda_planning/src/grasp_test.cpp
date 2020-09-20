#include <random>
#include <iostream>
// ROS
#include <ros/ros.h>
#include <panda_grasp/MyGraspPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/SolidPrimitive.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <gqcnn/GQCNNGraspPlanner.h>
#include <panda_planning/scene_perception.h>
#include <panda_planning/grasp_call.h>

typedef pcl::PointCloud<pcl::PointXYZ> PCL_CloudXYZ;

bool move_to_init(moveit::planning_interface::MoveGroupInterface &group,
                  ScenePreception &scene_preception,
                  GraspCall &graspPose);
void add_collision_table(ScenePreception &scene_preception);

void openGripper(trajectory_msgs::JointTrajectory &posture);
void closedGripper(trajectory_msgs::JointTrajectory &posture);

bool pick(moveit::planning_interface::MoveGroupInterface &move_group,
          ScenePreception &scene_preception,
          GraspCall &graspPose);
bool place(moveit::planning_interface::MoveGroupInterface &group);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_arm_grasp_test");
    GraspCall grasp_pose;
    ScenePreception scene_preception;
    scene_preception.remove_unprotected_collision_objects();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(10);
    // group.setNumPlanningAttempts(1);

    add_collision_table(scene_preception);
    scene_preception.add_protection_obj("table1");

    bool result = true;
    while (result)
    {
        scene_preception.remove_unprotected_collision_objects();
        scene_preception.add_protection_obj("object");

        result = move_to_init(group, scene_preception, grasp_pose);

        scene_preception.add_pcd2_collision_objects(true);

        result = pick(group, scene_preception, grasp_pose);

        ros::Duration(3.0).sleep();
        if (result == true)
        {
            result = place(group);
        }
        scene_preception.remove_protection_obj("object");

        ros::Duration(1.0).sleep();
    }

    ROS_INFO("grasp was end\n");

    ros::waitForShutdown();
    return 0;
}

bool move_to_init(moveit::planning_interface::MoveGroupInterface &group,
                  ScenePreception &scene_preception,
                  GraspCall &graspPose)
{
    /*眼在手外，直接惠傲初始位置*/
    // group.setNamedTarget("init");
    // group.move();

    /*眼在手上，设定第一次目标位置*/
    geometry_msgs::Pose target_pose;

    target_pose.position.x = 0.1;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.9;

    tf2::Quaternion ocm_orientation;
    ocm_orientation.setRPY(0, M_PI / 3, 0);
    target_pose.orientation = tf2::toMsg(ocm_orientation);
    group.setPoseTarget(target_pose);
    group.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("(constraints) %s", success ? "success" : "FAILED");
    if (success)
    {
        group.execute(my_plan);
    }
    else
    {
        return false;
    }
    ros::Duration(2.0).sleep();
    scene_preception.merge_point_cloud2(graspPose.get_point_cloud2(), true);

    /*眼在手上，设定第二次目标位置*/
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.7;

    ocm_orientation.setRPY(0, M_PI / 6, -M_PI / 4);
    target_pose.orientation = tf2::toMsg(ocm_orientation);
    group.setPoseTarget(target_pose);

    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("(constraints) %s", success ? "success" : "FAILED");
    if (success)
    {
        group.execute(my_plan);
    }
    else
    {
        return false;
    }
    ros::Duration(1.0).sleep();
    scene_preception.merge_point_cloud2(graspPose.get_point_cloud2(), false);

    /*眼在手上，设定第三次目标位置*/
    target_pose.position.x = 0.1;
    target_pose.position.y = -0.3;
    target_pose.position.z = 0.7;

    ocm_orientation.setRPY(0, M_PI / 6, M_PI / 4);
    target_pose.orientation = tf2::toMsg(ocm_orientation);
    group.setPoseTarget(target_pose);

    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("(constraints) %s", success ? "success" : "FAILED");

    if (success)
    {
        group.execute(my_plan);
    }
    else
    {
        return false;
    }
    ros::Duration(1.0).sleep();
    scene_preception.merge_point_cloud2(graspPose.get_point_cloud2(), false);

    /*眼在手上，回到第一次*/
    target_pose.position.x = 0.1;
    target_pose.position.y = 0;
    target_pose.position.z = 0.9;

    ocm_orientation.setRPY(0, M_PI / 3, 0);
    target_pose.orientation = tf2::toMsg(ocm_orientation);
    group.setPoseTarget(target_pose);

    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("(constraints) %s", success ? "success" : "FAILED");

    if (success)
    {
        group.execute(my_plan);
    }
    else
    {
        return false;
    }
    ros::Duration(1.0).sleep();
}

bool pick(moveit::planning_interface::MoveGroupInterface &move_group,
          ScenePreception &scene_preception,
          GraspCall &grasp_pose)
{
    panda_grasp::MyGraspPose::Response srv_grasp_res;
    if (grasp_pose.callGraspPose(scene_preception.get_merge_cloud2()))
    {
        srv_grasp_res = grasp_pose.getGraspPose();
        std::cout << "get the grasp num: " << srv_grasp_res.grasp_num << std::endl;
    }
    else
    {
        ROS_WARN("get the grasp false");
        return false;
    }

    if (srv_grasp_res.grasp_num == 0)
    {
        ROS_WARN("no grasp return");
        return false;
    }

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(srv_grasp_res.grasp_num);

    // move_group.setEndEffectorLink("panda_grasp_frame");
    std::string grasp_frame_name = move_group.getEndEffectorLink();
    ROS_INFO("grasp_frame_name: %s", grasp_frame_name.c_str());

    for (int i = 0; i < srv_grasp_res.grasp_num; i++)
    {
        scene_preception.recover_locked_obj();
        // Setting grasp pose
        grasps[i].grasp_pose.header.frame_id = "panda_link0";

        //orientation
        double roll, pitch, yaw;

        tf2::Quaternion quat;
        tf2::fromMsg(srv_grasp_res.grasp_pose.at(i).pose.orientation, quat);
        tf2::Matrix3x3 orientation(quat);
        orientation.getRPY(roll, pitch, yaw);

        std::cout << " roll:" << roll / 3.1415926 * 180 << std::endl
                  << " pitch:" << pitch / 3.1415926 * 180 << std::endl
                  << " yaw:" << yaw / 3.1415926 * 180 << std::endl;

        grasps[i].grasp_pose.pose.orientation = tf2::toMsg(quat);
        grasps[i].grasp_pose.pose.position = srv_grasp_res.grasp_pose.at(i).pose.position;
        // grasps[i].grasp_pose.pose.position.z += 0.1;

        scene_preception.add_grasp_collision_objects(grasps[i].grasp_pose.pose);

        // Setting pre-grasp approach
        /* Defined with respect to frame_id */
        grasps[i].pre_grasp_approach.direction.header.frame_id = "panda_link0";
        /* Direction is set as positive x axis */
        grasps[i].pre_grasp_approach.direction.vector = srv_grasp_res.approach.at(i);
        // grasps[0].pre_grasp_approach.direction.vector.x = 1;
        // grasps[0].pre_grasp_approach.direction.vector.z = 0;
        grasps[i].pre_grasp_approach.min_distance = 0.05;
        grasps[i].pre_grasp_approach.desired_distance = 0.115;

        // Setting post-grasp retreat
        grasps[i].post_grasp_retreat.direction.header.frame_id = "panda_link0";
        /* Direction is set as positive z axis */
        grasps[i].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[i].post_grasp_retreat.min_distance = 0.2;
        grasps[i].post_grasp_retreat.desired_distance = 0.25;

        // Setting posture of eef before grasp
        openGripper(grasps[i].pre_grasp_posture);

        // Setting posture of eef during grasp

        closedGripper(grasps[i].grasp_posture);

        move_group.setSupportSurfaceName("table1");
        moveit::planning_interface::MoveItErrorCode result = move_group.pick("object", grasps[i]);

        std::cout << "get the " << i << " grasp pose: \n"
                  << grasps[i].grasp_pose.pose << std::endl;
        std::cout << "get the " << i << " pre_grasp_approach: \n"
                  << srv_grasp_res.approach.at(i) << std::endl;
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {

            return true;
        }
    }

    // Set support surface as table1.
    // move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    // moveit::planning_interface::MoveItErrorCode result = move_group.pick("object", grasps);

    return false;
}

bool place(moveit::planning_interface::MoveGroupInterface &group)
{
    /*路径约束*/
    // robot_state::RobotState start_state(*group.getCurrentState());
    // group.setStartState(start_state);
    geometry_msgs::PoseStamped start_pose = group.getCurrentPose();

    // /*  旋转方向约束,末端保持一定角度*/
    // moveit_msgs::OrientationConstraint ocm;
    // ocm.link_name = "panda_grasp_frame";
    // ocm.header.frame_id = "panda_link0";
    //
    // tf2::Quaternion ocm_orientation;
    // // ocm_orientation.setRPY(0, 0, 0);
    // ocm.orientation = start_pose.pose.orientation;
    // ocm.absolute_x_axis_tolerance = 0.1;
    // ocm.absolute_y_axis_tolerance = 0.1;
    // ocm.absolute_z_axis_tolerance = 0.1;
    // ocm.weight = 1.0;

    // /*  位置约束,末端在设定范围内运动*/
    // moveit_msgs::PositionConstraint pcm;
    // pcm.link_name = "panda_grasp_frame";
    // pcm.header.frame_id = "panda_link0";
    // shape_msgs::SolidPrimitive pcm_shape;
    // pcm_shape.type = pcm_shape.BOX;
    // pcm_shape.dimensions.push_back(2);
    // pcm_shape.dimensions.push_back(2);
    // pcm_shape.dimensions.push_back(0.4);
    // pcm.constraint_region.primitives.push_back(pcm_shape);
    // geometry_msgs::Pose pcm_pose;
    // pcm_pose.position.x = 0;
    // pcm_pose.position.y = 0;
    // pcm_pose.position.z = 0.7;
    // tf2::Quaternion pcm_orientation;
    // pcm_orientation.setRPY(0, 0, 0);
    // pcm_pose.orientation = tf2::toMsg(pcm_orientation);
    // pcm.constraint_region.primitive_poses.push_back(pcm_pose);
    // pcm.target_point_offset.x = 0.1;
    // pcm.target_point_offset.y = 0.1;
    // pcm.target_point_offset.z = 0.1;
    // pcm.weight = 1.0;

    // // Now, set it as the path constraint for the group.
    // moveit_msgs::Constraints constraints;
    // // constraints.orientation_constraints.push_back(ocm);
    // constraints.position_constraints.push_back(pcm);
    // group.setPathConstraints(constraints);

    /*  设定目标位置*/
    geometry_msgs::Pose target_pose = start_pose.pose;
    std::cout << "target_pose::" << target_pose << std::endl;

    target_pose.position.x = 0.0;
    target_pose.position.y = 0.4;
    target_pose.position.z = 0.7;

    tf2::Quaternion ocm_orientation;
    ocm_orientation.setRPY(0, 0, M_PI / 2);
    target_pose.orientation = tf2::toMsg(ocm_orientation);
    group.setPoseTarget(target_pose);
    group.setPlanningTime(10.0);

    /*  路径约束后，进行路径规划*/
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("(constraints) %s", success ? "success" : "FAILED");

    group.clearPathConstraints();
    group.setPlanningTime(1);
    if (success)
    {
        group.execute(my_plan);
    }
    else
    {
        return false;
    }

    // /*笛卡尔空间规划，从抓取位置到放置位置，直线运动*/
    // geometry_msgs::PoseStamped start_pose = group.getCurrentPose();
    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(start_pose.pose);

    // geometry_msgs::Pose target_pose = start_pose.pose;
    // std::cout << "target_pose::" << target_pose << std::endl;

    // // target_pose.position.z += 0.05;
    // // waypoints.push_back(target_pose);

    // target_pose.position.x = 0.0;
    // target_pose.position.y = 0.5;
    // target_pose.position.z = 0.52;
    // tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, M_PI / 2);
    // target_pose.orientation = tf2::toMsg(orientation);
    // waypoints.push_back(target_pose);

    // group.setMaxVelocityScalingFactor(0.1);

    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // plan.trajectory_ = trajectory;
    // group.execute(plan);

    // // ros::Duration(6.0).sleep();
    // ROS_INFO("(Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    /*夹爪打开放下物体*/
    static const std::string GRASP_GROUP = "panda_gripper";
    moveit::planning_interface::MoveGroupInterface move_group(GRASP_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(GRASP_GROUP, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    std::cout << joint_group_positions.size() << std::endl;
    joint_group_positions[0] = 0.04; // radians
    joint_group_positions[1] = 0.04;
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success_gripper = (move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.execute(gripper_plan);
    ROS_INFO("(joint space goal) %s", success_gripper ? "success" : "FAILED");

    group.detachObject("object");

    // /*直接ｐｌａｃｅ规划*/
    // std::vector<moveit_msgs::PlaceLocation> place_location;
    // place_location.resize(1);

    // // Setting place location pose
    // place_location[0].place_pose.header.frame_id = "panda_link0";

    // tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, 0);
    // place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // /* While placing it is the exact location of the center of the object. */
    // place_location[0].place_pose.pose.position.x = 0;
    // place_location[0].place_pose.pose.position.y = 0.5;
    // place_location[0].place_pose.pose.position.z = 0.52;

    // // Setting pre-place approach
    // place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    // /* Direction is set as negative z axis */
    // place_location[0].pre_place_approach.direction.vector.z = -1.0;
    // place_location[0].pre_place_approach.min_distance = 0.095;
    // place_location[0].pre_place_approach.desired_distance = 0.115;

    // // Setting post-grasp retreat
    // place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    // /* Direction is set as negative y axis */
    // place_location[0].post_place_retreat.direction.vector.y = -1.0;
    // place_location[0].post_place_retreat.min_distance = 0.1;
    // place_location[0].post_place_retreat.desired_distance = 0.25;

    // // Setting posture of eef after placing object
    // openGripper(place_location[0].post_place_posture);

    // // Set support surface as table2.
    // // group.setSupportSurfaceName("table2");

    // // Call place to place the object using the place locations given.
    // moveit::planning_interface::MoveItErrorCode result = group.place("object", place_location);

    // ROS_INFO("place result:%d", result);
    // std::cout << "place result:" << result << std::endl;
    // if (result != 1)
    //     return false;

    return true;
}

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void add_collision_table(ScenePreception &scene_preception)
{
    moveit_msgs::CollisionObject collision_object;

    // // Add the first table where the cube will originally be kept.
    collision_object.id = "table1";
    collision_object.header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    // collision_object.primitives[0].dimensions[0] = 0.4;
    // collision_object.primitives[0].dimensions[1] = 0.4;
    // collision_object.primitives[0].dimensions[2] = 0.2;
    collision_object.primitives[0].dimensions[0] = 0.41;
    collision_object.primitives[0].dimensions[1] = 0.41;
    collision_object.primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = 0.5;
    collision_object.primitive_poses[0].position.y = 0;
    collision_object.primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL
    collision_object.operation = collision_object.ADD;

    scene_preception.add_collision_obj(collision_object);
}
