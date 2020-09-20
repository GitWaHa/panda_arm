#pragma once

#include <ros/ros.h>
#include <panda_grasp/MyGraspPose.h>
#include <gqcnn/GQCNNGraspPlanner.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <iostream>

class ScenePreception
{
private:
    /* data */
    ros::NodeHandle nh_;

    sensor_msgs::PointCloud2 point_cloud_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::vector<std::string> protection_objects_id_;
    std::vector<moveit_msgs::CollisionObject> lock_collision_objects_;

public:
    ScenePreception();
    ~ScenePreception();

    void merge_point_cloud2(sensor_msgs::PointCloud2 pcd, bool clear = false);
    const sensor_msgs::PointCloud2 get_merge_cloud2()
    {
        return point_cloud_;
    };

    void add_collision_obj(moveit_msgs::CollisionObject obj);
    void add_collision_objs(moveit_msgs::CollisionObject objs);

    void add_protection_obj(std::string id);
    void remove_protection_obj(std::string id);

    void recover_locked_obj();

    void remove_unprotected_collision_objects();

    void add_grasp_collision_objects(const geometry_msgs::Pose object_pose);
    void add_pcd2_collision_objects(bool lock = false);
    void add_pcd2_collision_objects(const sensor_msgs::PointCloud2 point_cloud2, bool lock = false);
};