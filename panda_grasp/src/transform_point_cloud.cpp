#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

typedef pcl::PointXYZRGB PCL_Point;
typedef pcl::PointCloud<PCL_Point> PCL_Cloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(PCL_Cloud::ConstPtr cloud,
                                                               PCL_Cloud::ConstPtr cloud2);

class TransformPointCloud2
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher depth_pub_;
    ros::Subscriber sub_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    std::string target_frame_;
    std::string source_frame_;

public:
    TransformPointCloud2()
        : tf2_listener_(tf2_buffer_),
          target_frame_("panda_link0"),
          source_frame_("camera_link")
    {
        sub_ = nh_.subscribe("/camera1/depth/points", 1, &TransformPointCloud2::call_back, this);

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera1/transform/depth/point", 1);
        // depth_pub_ = nh_.advertise<sensor_msgs::Image>("/camera1/transform/depth/image", 1);
    }
    ~TransformPointCloud2() {}

    void call_back(const sensor_msgs::PointCloud2ConstPtr &input);
};

void TransformPointCloud2::call_back(const sensor_msgs::PointCloud2ConstPtr &input)
{
    PCL_Cloud::Ptr rgbdCloud(new PCL_Cloud);
    PCL_Cloud::Ptr transformCloud(new PCL_Cloud);
    pcl::fromROSMsg(*input, *rgbdCloud); //cloud is the output

    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tf2_buffer_.lookupTransform(target_frame_, source_frame_,
                                                       ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    int size = rgbdCloud->size();
    for (int i = 0; i < size; i++)
    {
        PCL_Point p;
        geometry_msgs::PointStamped tfP;
        PCL_Point rgbdP = rgbdCloud->points.at(i);

        if (abs(rgbdP.x) < 1 && abs(rgbdP.y) < 1 && abs(rgbdP.z) < 1)
        {
            /*
            /仿真摄像头视觉是朝向+x方向的，但是点云建立却是朝向＋z方向的，会导致点云坐标无法对应到世界坐标系，
            /所以需要将点云绕相机坐标系旋转至与视觉方向对齐
            /功能与相机外参效果一样，但没找到如何修改仿真相机外参的方法
            */
            geometry_msgs::TransformStamped transform;
            transform.child_frame_id = "camera_link";
            transform.header.frame_id = "camera_link";
            tf2::Quaternion quat1;
            quat1.setRPY(0, 0, -M_PI / 2);
            tf2::Matrix3x3 orientation1(quat1);
            tf2::Quaternion quat2;
            quat2.setRPY(0, M_PI / 2, 0);
            tf2::Matrix3x3 orientation2(quat2);
            //先绕z旋转－９０，在绕y旋转９０度即可对齐
            orientation1 = orientation2 * orientation1;
            //求得旋转矩阵后，利用坐标变换，将点云与视觉方向对齐
            orientation1.getRotation(quat1);
            geometry_msgs::Quaternion q;
            q.w = quat1.w();
            q.x = quat1.x();
            q.y = quat1.y();
            q.z = quat1.z();
            transform.transform.rotation = q;
            tfP.point.x = rgbdP.x;
            tfP.point.y = rgbdP.y;
            tfP.point.z = rgbdP.z;
            tf2::doTransform(tfP, tfP, transform);
            p.x = tfP.point.x;
            p.y = tfP.point.y;
            p.z = tfP.point.z;

            /*直接变换，效果同旋转矩阵变换一样*/
            // p.x = rgbdP.z;
            // p.y = -rgbdP.x;
            // p.z = -rgbdP.y;

            /*将点云变换到世界坐标系下*/
            tfP.point.x = p.x;
            tfP.point.y = p.y;
            tfP.point.z = p.z;
            tf2::doTransform(tfP, tfP, transformStamped);
            p.x = tfP.point.x;
            p.y = tfP.point.y;
            p.z = tfP.point.z;

            transformCloud->points.push_back(p);
        }
    }
    transformCloud->width = transformCloud->points.size();
    transformCloud->height = 1;

    //pcl可视化点云
    // static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer(rgbdCloud, transformCloud);
    // viewer->updatePointCloud(rgbdCloud, "rgbdCloud");
    // viewer->updatePointCloud(transformCloud, "transformCloud");
    // viewer->spinOnce(5);

    //发布转换后的PointCloud2
    if (transformCloud->points.size() != 0)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*transformCloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = target_frame_;
        // std::cout << msg.header.frame_id << std::endl;
        pub_.publish(msg);

        static bool save_flag = true;
        if (save_flag)
        {
            save_flag = false;
            pcl::io::savePCDFileASCII("/home/waha/catkin_ws/src/panda/panda_grasp/pcd_data/transformCloud.pcd", *transformCloud);
            ROS_INFO("save transformCloud.pcd");
        }
    }

    // // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    // float angularResolution = (float)(1.0f * (M_PI / 180.0f)); //   1.0 degree in radians
    // float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));   // 360.0 degree in radians
    // float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
    // Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    // pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    // float noiseLevel = 0.00;
    // float minRange = 0.0f;
    // int borderSize = 1;

    // pcl::RangeImage rangeImage;
    // rangeImage.createFromPointCloud(*rgbdCloud, angularResolution, maxAngleWidth, maxAngleHeight,
    //                                 sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    // sensor_msgs::Image image_msgs;
    // // pcl::moveToROSMsg(msg, image_msgs);
    // pcl::toROSMsg(rangeImage, image_msgs);
    // depth_pub_.publish(image_msgs);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "preprocess_point_cloud");
    TransformPointCloud2 conver;

    // Spin
    ros::spin();
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(PCL_Cloud::ConstPtr cloud,
                                                               PCL_Cloud::ConstPtr cloud2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->initCameraParameters();

    int v1(0);
    // viewer->addCoordinateSystem();
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 rgbdCloud", v1);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud(cloud, "rgbdCloud");
    viewer->addCoordinateSystem(0.1, "rgbdCloud", v1);
    int v2(1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("Radius: 0.1", 10, 10, "v2 Cloud", v2);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
    viewer->addPointCloud(cloud2, "transformCloud");
    viewer->addCoordinateSystem(0.1, "transformCloud", v2);

    return (viewer);
}