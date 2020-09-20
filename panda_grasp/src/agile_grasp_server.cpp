#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <panda_grasp/MyGraspPose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <agile_grasp/learning.h>
#include <agile_grasp/localization.h>
#include <agile_grasp/config_file.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

typedef pcl::PointCloud<pcl::PointXYZ> PCL_CloudXYZ;

class CalculateGraspPose
{
private:
    ros::NodeHandle nh_;
    // ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::ServiceServer server_;
    tf::TransformListener listener_;

public:
    CalculateGraspPose()
    {
        sub_ = nh_.subscribe("/camera1/depth/conversionPoints", 1, &CalculateGraspPose::subCallBack, this);

        server_ = nh_.advertiseService("/panda_grasp/get_pose", &CalculateGraspPose::serverCallBack, this);

        // pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera1/depth/conversionPoints", 1);
    }
    ~CalculateGraspPose() {}

    void subCallBack(const sensor_msgs::PointCloud2ConstPtr &input);
    bool serverCallBack(panda_grasp::MyGraspPose::Request &req, panda_grasp::MyGraspPose::Response &res);

    Eigen::Vector3d getWorldApproch(const Eigen::Vector3d &camera_grasp_approach);
    Eigen::Vector3d getWorldPosition(const Eigen::Vector3d &camera_grasp_position);
    Eigen::Vector3d getWorldQuaternion(const Eigen::Vector3d &world_grasp_approach, const Eigen::Vector3d &world_grasp_axis);
    Eigen::Vector3d getWorldAxis(const Eigen::Vector3d &camera_grasp_axis);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_grasp_pose");

    //Rc_w  Rc_w
    // double roll, pitch, yaw;

    // tf2::Quaternion quat1;
    // quat1.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    // tf2::Matrix3x3 orientation1(quat1);
    // tf2::Quaternion quat0;
    // quat0.setRPY(0, -M_PI / 2, 0);
    // tf2::Matrix3x3 orientation0(quat0);

    // orientation0 = orientation1 * orientation0.inverse();
    // orientation0.getRPY(roll, pitch, yaw);

    // std::cout << " roll:" << roll / 3.1415926 * 180 << std::endl
    //           << " pitch:" << pitch / 3.1415926 * 180 << std::endl
    //           << " yaw:" << yaw / 3.1415926 * 180 << std::endl;

    CalculateGraspPose get_pose;
    ros::Duration(1).sleep();
    ros::spin();

    return 0;
}

bool CalculateGraspPose::serverCallBack(panda_grasp::MyGraspPose::Request &req, panda_grasp::MyGraspPose::Response &res)
{
    // read PCD filename from command line
    std::string file_name_left;
    std::string file_name_right;
    file_name_left = "/home/waha/catkin_ws/src/panda_grasp/pcd_data/rgbdCloud.pcd";
    file_name_right = "";

    // read SVM filename from command line
    std::string svm_file_name("/home/waha/catkin_ws/src/agile_grasp/svm_032015_20_20_same");
    std::string config_filename("/home/waha/catkin_ws/src/panda_grasp/config/agile_parameter.cfg");

    /* read number of samples, number of threads and min handle inliers
		 * from command line */
    ConfigFile config_file(config_filename);
    config_file.ExtractKeys();
    int num_samples = config_file.getValueOfKey("num_samples", 1000);
    int num_threads = config_file.getValueOfKey("num_threads", 1);
    int min_inliers = config_file.getValueOfKey("min_inliers", 1);
    int plot_mode = config_file.getValueOfKey("plot_mode", 0);
    double taubin_radius = config_file.getValueOfKey("taubin_radius", 0.03);
    double hand_radius = config_file.getValueOfKey("hand_radius", 0.08);
    std::cout << "num_samples: " << num_samples << std::endl;
    std::cout << "num_threads: " << num_threads << std::endl;
    std::cout << "min_inliers: " << min_inliers << std::endl;
    std::cout << "plot_mode: " << plot_mode << std::endl;
    std::cout << "taubin_radius: " << taubin_radius << std::endl;
    std::cout << "hand_radius: " << hand_radius << std::endl;

    // camera poses for 2-camera Baxter setup
    Eigen::Matrix4d base_tf, sqrt_tf;

    base_tf << 0, 0.445417, 0.895323, 0.215,
        1, 0, 0, -0.015,
        0, 0.895323, -0.445417, 0.23,
        0, 0, 0, 1;

    sqrt_tf << 0.9366, -0.0162, 0.3500, -0.2863,
        0.0151, 0.9999, 0.0058, 0.0058,
        -0.3501, -0.0002, 0.9367, 0.0554,
        0, 0, 0, 1;

    // workspace dimensions
    std::vector<double> workspace_cfg = config_file.getValueOfKeyAsStdVectorDouble("workspace", "0 0.7 -0.4 0.4 0.4 1.0");
    Eigen::VectorXd workspace(6);
    workspace << workspace_cfg.at(0), workspace_cfg.at(1), workspace_cfg.at(2),
        workspace_cfg.at(3), workspace_cfg.at(4), workspace_cfg.at(5);
    std::cout << "workspace: " << workspace_cfg.at(0) << workspace_cfg.at(1) << workspace_cfg.at(2)
              << workspace_cfg.at(3) << workspace_cfg.at(4) << workspace_cfg.at(5) << std::endl;

    // set-up parameters for the hand search
    Localization loc(num_threads, true, plot_mode);
    loc.setCameraTransforms(base_tf * sqrt_tf.inverse(), base_tf * sqrt_tf);
    // loc.setCameraTransforms(base_tf * sqrt_tf.inverse(), base_tf * sqrt_tf.inverse());
    loc.setWorkspace(workspace);
    loc.setNumSamples(num_samples);
    loc.setNeighborhoodRadiusTaubin(taubin_radius);
    loc.setNeighborhoodRadiusHands(hand_radius);
    loc.setFingerWidth(0.01);
    loc.setHandOuterDiameter(0.09);
    loc.setHandDepth(0.06);
    loc.setInitBite(0.01);
    loc.setHandHeight(0.02);
    std::cout << "Localizing hands ...\n";

    // test with randomly sampled indices
    std::vector<GraspHypothesis> hands = loc.localizeHands(file_name_left, file_name_right);

    std::vector<GraspHypothesis> antipodal_hands = loc.predictAntipodalHands(hands, svm_file_name);

    std::vector<Handle> handles = loc.findHandles(antipodal_hands, min_inliers, 0.005);

    tf::TransformListener listener;
    int handle_count = 0;
    for (int i = 0; i < handles.size(); i++)
    {
        Eigen::Vector3d center = handles.at(i).getCenter();
        Eigen::Vector3d axis = handles.at(i).getAxis();
        Eigen::Vector3d approach = handles.at(i).getApproach();

        if (center(2) > 0.42)
        {
            handle_count++;
            res.grasp_num = handle_count;

            Eigen::Vector3d world_quaternion = CalculateGraspPose::getWorldQuaternion(approach, axis);
            std::cout << "handle_count: " << handle_count << std::endl;

            geometry_msgs::PoseStamped grasp_pose_msg;
            grasp_pose_msg.pose.position.x = center(0);
            grasp_pose_msg.pose.position.y = center(1);
            grasp_pose_msg.pose.position.z = center(2);

            tf2::Quaternion orientation;
            orientation.setRPY(world_quaternion(0), world_quaternion(1), world_quaternion(2));
            grasp_pose_msg.pose.orientation = tf2::toMsg(orientation);
            res.grasp_pose.push_back(tf2::toMsg(grasp_pose_msg));

            geometry_msgs::Vector3 approach_msg;
            approach_msg.x = approach(0);
            approach_msg.y = approach(1);
            approach_msg.z = approach(2);
            res.approach.push_back(approach_msg);

            std::cout << "world_position:\n"
                      << center << std::endl;
            std::cout << "world_quaternion:\n"
                      << world_quaternion << std::endl;
            std::cout << " roll:" << world_quaternion(0) / 3.1415926 * 180
                      << " pitch:" << world_quaternion(1) / 3.1415926 * 180
                      << " yaw:" << world_quaternion(2) / 3.1415926 * 180 << std::endl;
            std::cout << "world_approach:\n"
                      << approach << std::endl;
        }
    }

    if (handle_count == 0)
    {
        ROS_WARN("warning : no find handle");
        return false;
    }
    return true;
}

void CalculateGraspPose::subCallBack(const sensor_msgs::PointCloud2ConstPtr &input)
{
    PCL_CloudXYZ::Ptr rgbdCloud(new PCL_CloudXYZ);
    pcl::fromROSMsg(*input, *rgbdCloud);

    pcl::io::savePCDFileASCII("/home/waha/catkin_ws/src/panda_grasp/pcd_data/rgbdCloud.pcd", *rgbdCloud);
    // cout << "保存一次点云数据" << endl;
}

Eigen::Vector3d CalculateGraspPose::getWorldQuaternion(const Eigen::Vector3d &world_grasp_approach, const Eigen::Vector3d &world_grasp_axis)
{
    Eigen::Vector3d wrold_quaternion;
    float roll, pitch, yaw;

    if (world_grasp_axis(1) < 0)
    {
        roll = std::acos(world_grasp_axis(2) / std::hypot(world_grasp_axis(1), world_grasp_axis(2)));
    }
    else
    {
        roll = -std::acos(world_grasp_axis(2) / std::hypot(world_grasp_axis(1), world_grasp_axis(2)));
    }

    double hypot_xz = std::hypot(world_grasp_approach(0), world_grasp_approach(2));
    if (world_grasp_approach(2) > 0)
    {
        pitch = std::acos(world_grasp_approach(0) / hypot_xz);
    }
    else
    {
        pitch = -std::acos(world_grasp_approach(0) / hypot_xz);
    }

    double hypot_xyz = std::hypot(hypot_xz, world_grasp_approach(1));
    if (world_grasp_approach(1) > 0)
    {
        yaw = std::acos(hypot_xz / hypot_xyz);
    }
    else
    {
        yaw = -std::acos(hypot_xz / hypot_xyz);
    }

    wrold_quaternion << roll, pitch, yaw;
    return wrold_quaternion;
}

Eigen::Vector3d CalculateGraspPose::getWorldPosition(const Eigen::Vector3d &camera_grasp_position)
{
    Eigen::Vector3d wrold_position;

    geometry_msgs::PoseStamped cameraPose, cameraToWorldPosi;
    cameraPose.header.frame_id = "camera_link";
    cameraPose.pose.position.x = camera_grasp_position(0);
    cameraPose.pose.position.y = camera_grasp_position(1);
    cameraPose.pose.position.z = camera_grasp_position(2);
    cameraPose.pose.orientation.w = 1.0;

    try
    {

        listener_.waitForTransform("camera_link", "panda_link0", ros::Time(0), ros::Duration(3.0));
        listener_.transformPose("panda_link0", cameraPose, cameraToWorldPosi);
    }
    catch (const tf::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
    }
    wrold_position << cameraToWorldPosi.pose.position.x,
        cameraToWorldPosi.pose.position.y,
        cameraToWorldPosi.pose.position.z;

    return wrold_position;
}

Eigen::Vector3d CalculateGraspPose::getWorldAxis(const Eigen::Vector3d &camera_grasp_axis)
{
    Eigen::Vector3d wrold_axis;

    geometry_msgs::Vector3Stamped cameraAxis, cameraToWorldAxis;
    cameraAxis.header.frame_id = "camera_link";
    cameraAxis.vector.x = camera_grasp_axis(0);
    cameraAxis.vector.y = camera_grasp_axis(1);
    cameraAxis.vector.z = camera_grasp_axis(2);

    try
    {

        listener_.waitForTransform("camera_link", "panda_link0", ros::Time(0), ros::Duration(3.0));
        listener_.transformVector("panda_link0", cameraAxis, cameraToWorldAxis);
    }
    catch (const tf::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
    }
    wrold_axis << cameraToWorldAxis.vector.x,
        cameraToWorldAxis.vector.y,
        cameraToWorldAxis.vector.z;

    return wrold_axis;
}

Eigen::Vector3d CalculateGraspPose::getWorldApproch(const Eigen::Vector3d &camera_grasp_approach)
{
    Eigen::Vector3d wrold_approach;

    geometry_msgs::Vector3Stamped cameraApproach, cameraToWorldApproach;
    cameraApproach.header.frame_id = "camera_link";
    cameraApproach.vector.x = camera_grasp_approach(0);
    cameraApproach.vector.y = camera_grasp_approach(1);
    cameraApproach.vector.z = camera_grasp_approach(2);

    try
    {

        listener_.waitForTransform("camera_link", "panda_link0", ros::Time(0), ros::Duration(3.0));
        listener_.transformVector("panda_link0", cameraApproach, cameraToWorldApproach);
    }
    catch (const tf::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
    }
    wrold_approach << cameraToWorldApproach.vector.x,
        cameraToWorldApproach.vector.y,
        cameraToWorldApproach.vector.z;

    return wrold_approach;
}