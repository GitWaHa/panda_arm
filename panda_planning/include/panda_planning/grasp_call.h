#pragma once

#include <ros/ros.h>
#include <panda_grasp/MyGraspPose.h>
#include <gqcnn/GQCNNGraspPlanner.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>

class GraspCall
{
private:
    /* data */
    ros::NodeHandle nh_;
    // ros::Publisher pub_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber color_image_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber camera_info_sub_;
    // ros::ServiceServer server_;
    ros::ServiceClient client_;

    panda_grasp::MyGraspPose srv_;
    gqcnn::GQCNNGraspPlanner gqcnn_srv_;

    sensor_msgs::PointCloud2 point_cloud_;
    sensor_msgs::Image color_image_;
    sensor_msgs::Image depth_image_;
    sensor_msgs::CameraInfo camera_info_;

    std::string target_frame_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

public:
    GraspCall();
    ~GraspCall();

    void point_cloud_call_back(const sensor_msgs::PointCloud2ConstPtr &input);
    void color_image_call_back(const sensor_msgs::ImageConstPtr &input);
    void depth_image_call_back(const sensor_msgs::ImageConstPtr &input);
    void camera_info_call_back(const sensor_msgs::CameraInfoConstPtr &input);

    bool callGraspPose();
    bool callGraspPose(const sensor_msgs::PointCloud2 &pcd);

    panda_grasp::MyGraspPose::Response getGraspPose();
    void transform_pose(std::string server_name);

    const sensor_msgs::PointCloud2 get_point_cloud2()
    {
        return point_cloud_;
    }
};