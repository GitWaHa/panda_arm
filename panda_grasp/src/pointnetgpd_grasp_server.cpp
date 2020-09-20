#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <panda_grasp/MyGraspPose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <gpd/config_file.h>
#include <gpd/grasp_detector.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

typedef pcl::PointCloud<pcl::PointXYZ> PCL_CloudXYZ;

Eigen::Matrix3d calculate_rotation_matrix(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter);
double calculateAngle(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter);

class CalculateGraspPose
{
private:
    ros::NodeHandle nh_;
    // ros::Publisher pub_;
    // ros::Subscriber sub_;
    ros::ServiceServer server_;
    tf::TransformListener listener_;

public:
    CalculateGraspPose()
    {
        // sub_ = nh_.subscribe("/camera1/depth/conversionPoints", 1, &CalculateGraspPose::subCallBack, this);
        server_ = nh_.advertiseService("/panda_grasp/get_pose", &CalculateGraspPose::serverCallBack, this);
    }
    ~CalculateGraspPose() {}

    // void subCallBack(const sensor_msgs::PointCloud2ConstPtr &input);
    bool serverCallBack(panda_grasp::MyGraspPose::Request &req, panda_grasp::MyGraspPose::Response &res);

    bool checkFileExists(const std::string &file_name);

    Eigen::Vector3d getWorldApproch(const Eigen::Vector3d &camera_grasp_approach);
    Eigen::Vector3d getWorldPosition(const Eigen::Vector3d &camera_grasp_position);
    Eigen::Vector3d getWorldQuaternion(const Eigen::Vector3d &world_grasp_approach, const Eigen::Vector3d &world_grasp_axis);
    Eigen::Vector3d getWorldAxis(const Eigen::Vector3d &camera_grasp_axis);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpd_calculatet_grasp_pose");

    CalculateGraspPose calculatet_pose;
    ros::Duration(1).sleep();
    ros::spin();

    return 0;
}

bool CalculateGraspPose::serverCallBack(panda_grasp::MyGraspPose::Request &req, panda_grasp::MyGraspPose::Response &res)
{

    PCL_CloudXYZ::Ptr rgbdCloud(new PCL_CloudXYZ);
    pcl::fromROSMsg(req.point_cloud, *rgbdCloud);

    pcl::io::savePCDFileASCII("/home/waha/catkin_ws/src/panda_grasp/pcd_data/rgbdCloud.pcd", *rgbdCloud);
    ROS_INFO("savePCD /home/waha/catkin_ws/src/panda_grasp/pcd_data/rgbdCloud.pcd");

    // Read arguments from command line.
    std::string config_filename("/home/waha/catkin_ws/src/panda_grasp/config/gpd_parameter.cfg");
    std::string pcd_filename = "/home/waha/catkin_ws/src/panda_grasp/pcd_data/rgbdCloud.pcd";
    if (!checkFileExists(config_filename))
    {
        printf("Error: config file not found!\n");
        return (-1);
    }
    if (!checkFileExists(pcd_filename))
    {
        printf("Error: PCD file not found!\n");
        return (-1);
    }

    // Read parameters from configuration file.
    gpd::util::ConfigFile config_file(config_filename);
    config_file.ExtractKeys();

    // Set the camera position. Assumes a single camera view.
    std::vector<double> camera_position =
        config_file.getValueOfKeyAsStdVectorDouble("camera_position",
                                                   "0.5 -0.5 0.7");

    Eigen::Matrix3Xd view_points(3, 1);
    view_points << camera_position[0], camera_position[1], camera_position[2];

    // Load point cloud from file.
    gpd::util::Cloud cloud(pcd_filename, view_points);
    if (cloud.getCloudOriginal()->size() == 0)
    {
        std::cout << "Error: Input point cloud is empty or does not exist!\n";
        return (-1);
    }

    // Load surface normals from file.
    // if (argc > 3)
    // {
    //     std::string normals_filename = argv[3];
    //     cloud.setNormalsFromFile(normals_filename);
    //     std::cout << "Loaded surface normals from file: " << normals_filename
    //               << "\n";
    // }

    gpd::GraspDetector detector(config_filename);

    // Preprocess the point cloud.
    detector.preprocessPointCloud(cloud);

    // If the object is centered at the origin, reverse all surface normals.
    bool centered_at_origin =
        config_file.getValueOfKey<bool>("centered_at_origin", false);
    if (centered_at_origin)
    {
        printf("Reversing normal directions ...\n");
        cloud.setNormals(cloud.getNormals() * (-1.0));
    }

    // Detect grasp poses.
    std::vector<std::unique_ptr<gpd::candidate::Hand>> handles = detector.detectGrasps(cloud);

    int handle_count = 0;
    for (int i = 0; i < handles.size(); i++)
    {
        Eigen::Vector3d position = handles.at(i).get()->getPosition();
        Eigen::Vector3d axis = handles.at(i).get()->getAxis();
        Eigen::Vector3d approach = handles.at(i).get()->getApproach();
        Eigen::Vector3d euler = handles.at(i).get()->getOrientation().eulerAngles(0, 1, 2);
        Eigen::Vector3d binormal = handles.at(i).get()->getBinormal();

        if (position(2) > 0.42)
        {
            handle_count++;
            std::cout << "#######  handle_count: " << handle_count << " #######: " << std::endl;
            res.grasp_num = handle_count;

            Eigen::Vector3d world_euler = CalculateGraspPose::getWorldQuaternion(approach, axis);

            double roll, pitch, yaw;
            roll = world_euler(0);
            pitch = world_euler(1);
            yaw = world_euler(2);

            geometry_msgs::PoseStamped grasp_pose_msg;
            grasp_pose_msg.header.frame_id = req.point_cloud.header.frame_id;
            grasp_pose_msg.pose.position.x = position(0);
            grasp_pose_msg.pose.position.y = position(1);
            grasp_pose_msg.pose.position.z = position(2);

            tf2::Quaternion orientation;
            orientation.setRPY(roll, pitch, yaw);
            grasp_pose_msg.pose.orientation = tf2::toMsg(orientation);
            res.grasp_pose.push_back(tf2::toMsg(grasp_pose_msg));

            geometry_msgs::Vector3 approach_msg;
            approach_msg.x = approach(0);
            approach_msg.y = approach(1);
            approach_msg.z = approach(2);
            res.approach.push_back(approach_msg);

            std::cout << "world_position:\n"
                      << position << std::endl;
            std::cout << " roll1:" << world_euler(0) / 3.1415926 * 180
                      << " pitch1:" << world_euler(1) / 3.1415926 * 180
                      << " yaw1:" << world_euler(2) / 3.1415926 * 180 << std::endl;
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

// void CalculateGraspPose::subCallBack(const sensor_msgs::PointCloud2ConstPtr &input)
// {
//     PCL_CloudXYZ::Ptr rgbdCloud(new PCL_CloudXYZ);
//     pcl::fromROSMsg(*input, *rgbdCloud);

//     pcl::io::savePCDFileASCII("/home/waha/catkin_ws/src/panda_grasp/pcd_data/rgbdCloud.pcd", *rgbdCloud);
//     // cout << "保存一次点云数据" << endl;
// }

bool CalculateGraspPose::checkFileExists(const std::string &file_name)
{
    std::ifstream file;
    file.open(file_name.c_str());
    if (!file)
    {
        std::cout << "File " + file_name + " could not be found!\n";
        return false;
    }
    file.close();
    return true;
}

Eigen::Vector3d CalculateGraspPose::getWorldQuaternion(const Eigen::Vector3d &world_grasp_approach, const Eigen::Vector3d &world_grasp_axis)
{
    /*根据approach向量计算旋转矩阵，此时只能保证approach与机械臂抓取参考坐标系x轴重合*/
    Eigen::Vector3d temp;
    temp(0) = 1;
    temp(1) = 0;
    temp(2) = 0;
    Eigen::Matrix3d rotation_matrix_1 = calculate_rotation_matrix(temp, world_grasp_approach);
    Eigen::Vector3d euler_1 = rotation_matrix_1.eulerAngles(0, 1, 2);

    /*根据　axis　向量计算与x轴重合后的y轴与axis的角度*/
    temp(0) = 0;
    temp(1) = 0;
    temp(2) = 1;
    Eigen::Vector3d axis_z_1 = rotation_matrix_1 * temp;
    double roll3_angle2 = calculateAngle(axis_z_1, world_grasp_axis);
    // std::cout << "roll3_angle2: " << roll3_angle2 << std::endl;

    //欧拉角转换为旋转矩阵
    Eigen::Vector3d ea(0, 0, roll3_angle2);
    Eigen::Matrix3d rotation_matrix_2;
    rotation_matrix_2 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    // cout << "rotation matrix =\n" << rotation_matrix_2 << endl;

    /*先旋转至x轴重合，在旋转至y轴重合*/
    rotation_matrix_2 = rotation_matrix_2 * rotation_matrix_1;
    Eigen::Vector3d euler_2 = rotation_matrix_2.eulerAngles(0, 1, 2);

    return euler_2;
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

#define MY_PI 3.1415926
double calculateAngle(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    double ab, a1, b1, cosr;
    ab = vectorBefore.x() * vectorAfter.x() + vectorBefore.y() * vectorAfter.y() + vectorBefore.z() * vectorAfter.z();
    a1 = sqrt(vectorBefore.x() * vectorBefore.x() + vectorBefore.y() * vectorBefore.y() + vectorBefore.z() * vectorBefore.z());
    b1 = sqrt(vectorAfter.x() * vectorAfter.x() + vectorAfter.y() * vectorAfter.y() + vectorAfter.z() * vectorAfter.z());
    cosr = ab / a1 / b1;
    return (acos(cosr));
}

inline Eigen::Vector3d calculateRotAxis(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    return Eigen::Vector3d(vectorBefore.y() * vectorAfter.z() - vectorBefore.z() * vectorAfter.y(),
                           vectorBefore.z() * vectorAfter.y() - vectorBefore.x() * vectorAfter.z(),
                           vectorBefore.x() * vectorAfter.y() - vectorBefore.y() * vectorAfter.x());
}

Eigen::Matrix3d calculate_rotation_matrix(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    Eigen::Vector3d vector = calculateRotAxis(vectorBefore, vectorAfter);
    double angle = calculateAngle(vectorBefore, vectorAfter);
    Eigen::AngleAxisd rotationVector(angle, vector.normalized());
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rotMatrix = rotationVector.toRotationMatrix(); //所求旋转矩阵

    return rotMatrix;
}