#include <panda_planning/grasp_call.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PCL_Point;
typedef pcl::PointCloud<PCL_Point> PCL_Cloud;

GraspCall::GraspCall()
    : tf2_listener_(tf2_buffer_),
      target_frame_("panda_link0")
{

    client_ = nh_.serviceClient<panda_grasp::MyGraspPose>("/panda_grasp/get_pose");

    point_cloud_sub_ = nh_.subscribe("/camera1/transform/depth/point", 1, &GraspCall::point_cloud_call_back, this);
    color_image_sub_ = nh_.subscribe("/camera1/color/image_raw", 1, &GraspCall::color_image_call_back, this);
    depth_image_sub_ = nh_.subscribe("/camera1/depth/image_raw", 1, &GraspCall::depth_image_call_back, this);
    camera_info_sub_ = nh_.subscribe("/camera1_ir/depth/camera_info", 1, &GraspCall::camera_info_call_back, this);
}

GraspCall::~GraspCall()
{
}

bool GraspCall::callGraspPose(const sensor_msgs::PointCloud2 &pcd)
{
    srv_.request.header.frame_id = target_frame_;
    srv_.request.point_cloud = pcd;
    srv_.request.color_image = color_image_;
    srv_.request.depth_image = depth_image_;
    srv_.request.camera_info = camera_info_;

    if (client_.call(srv_))
    {
        if (srv_.response.grasp_num > 0)
        {
            ROS_INFO("success call %s GraspPose,find %d grasp pose", srv_.response.server_name.c_str(), srv_.response.grasp_num);

            transform_pose(srv_.response.server_name);
        }
        else
        {
            ROS_WARN("call service /panda_grasp/get_pose success,but 0 grasp_num");
            return false;
        }
    }
    else
    {
        ROS_WARN("call service /panda_grasp/get_pose faild");
        return false;
    }
    return true;
}

bool GraspCall::callGraspPose()
{
    srv_.request.header.frame_id = target_frame_;
    srv_.request.point_cloud = point_cloud_;
    srv_.request.color_image = color_image_;
    srv_.request.depth_image = depth_image_;
    srv_.request.camera_info = camera_info_;

    if (client_.call(srv_))
    {
        if (srv_.response.grasp_num > 0)
        {
            ROS_INFO("success call %s GraspPose,find %d grasp pose", srv_.response.server_name.c_str(), srv_.response.grasp_num);

            transform_pose(srv_.response.server_name);
        }
        else
        {
            ROS_WARN("call service /panda_grasp/get_pose success,but 0 grasp_num");
            return false;
        }
    }
    else
    {
        ROS_WARN("call service /panda_grasp/get_pose faild");
        return false;
    }
    return true;
}

void GraspCall::transform_pose(std::string server_name)
{
    if (srv_.response.grasp_num > 0)
    {
        if (server_name == "dexnet")
        {
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

            for (int i = 0; i < srv_.response.grasp_num; i++)
            {
                geometry_msgs::TransformStamped transform_approach;
                transform_approach.child_frame_id = "camera_link";
                transform_approach.header.frame_id = "camera_link";
                transform_approach.transform.rotation = srv_.response.grasp_pose.at(i).pose.orientation;
                geometry_msgs::Vector3 approach;
                approach.x = 1;
                approach.y = 0;
                approach.z = 0;
                //根据grasp_pose计算出approach方向，因为server 端没有赋值approach
                tf2::doTransform(approach, srv_.response.approach.at(i), transform_approach);

                tf2::doTransform(srv_.response.grasp_pose.at(i), srv_.response.grasp_pose.at(i), transform);
                tf2::doTransform(srv_.response.approach.at(i), srv_.response.approach.at(i), transform);
                // std::cout << srv_.response.grasp_pose.at(i).header.frame_id << std::endl;
            }
        }
        std::string source_frame = srv_.response.grasp_pose.at(0).header.frame_id;
        if (source_frame != target_frame_)
        {
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tf2_buffer_.lookupTransform(target_frame_, source_frame,
                                                               ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                return;
            }

            for (int i = 0; i < srv_.response.grasp_num; i++)
            {
                tf2::doTransform(srv_.response.grasp_pose.at(i), srv_.response.grasp_pose.at(i), transformStamped);
                tf2::doTransform(srv_.response.approach.at(i), srv_.response.approach.at(i), transformStamped);
            }
        }
    }
}

panda_grasp::MyGraspPose::Response GraspCall::getGraspPose()
{
    return srv_.response;
}

void GraspCall::point_cloud_call_back(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // PCL_Cloud::Ptr rgbdCloud(new PCL_Cloud);
    // pcl::fromROSMsg(*input, *rgbdCloud);
    point_cloud_ = *input;
}

void GraspCall::color_image_call_back(const sensor_msgs::ImageConstPtr &input)
{
    color_image_ = *input;
}

void GraspCall::depth_image_call_back(const sensor_msgs::ImageConstPtr &input)
{
    depth_image_ = *input;
}

void GraspCall::camera_info_call_back(const sensor_msgs::CameraInfoConstPtr &input)
{
    camera_info_ = *input;
}