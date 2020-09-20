#include <panda_planning/scene_perception.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/console/parse.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/segmentation/extract_clusters.h>

#include <random>
#include <iostream>

typedef pcl::PointXYZRGB PCL_Point;
typedef pcl::PointCloud<PCL_Point> PCL_Cloud;

ScenePreception::ScenePreception()
{
}

ScenePreception::~ScenePreception()
{
}

void ScenePreception::merge_point_cloud2(sensor_msgs::PointCloud2 pcd, bool clear)
{
    PCL_Cloud::Ptr new_pcd(new PCL_Cloud);
    pcl::fromROSMsg(pcd, *new_pcd);

    if (!clear)
    {
        PCL_Cloud::Ptr original_pcd(new PCL_Cloud);
        pcl::fromROSMsg(point_cloud_, *original_pcd);
        *new_pcd = *original_pcd + *new_pcd;
    }

    pcl::toROSMsg(*new_pcd, point_cloud_);
}

void ScenePreception::add_pcd2_collision_objects(bool lock)
{
    PCL_Cloud::Ptr rgbdCloud(new PCL_Cloud);
    pcl::fromROSMsg(point_cloud_, *rgbdCloud);

    pcl::StopWatch time;
    std::cout << "cloud_xyzrgb: " << rgbdCloud->size() << " data points." << std::endl;
    PCL_Cloud::Ptr cloud_filtered(new PCL_Cloud);
    /****** 体素下采样 ********/
    pcl::VoxelGrid<PCL_Point> sor;
    sor.setInputCloud(rgbdCloud);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor.filter(*cloud_filtered);

    std::cout << "cloud_filtered: " << cloud_filtered->size() << " data points." << std::endl;
    std::cout << "下采样分割运行时间" << time.getTime() << "ms" << std::endl;

    int size = cloud_filtered->size();
    std::random_device rd;     //Get a random seed from the OS entropy device, or whatever
    std::mt19937_64 eng(rd()); //Use the 64-bit Mersenne Twister 19937 generator

    std::uniform_int_distribution<unsigned long long> distr(0, size); // distribution in range [1, 6]

    std::vector<std::string> object_ids;

    std::vector<moveit_msgs::CollisionObject> collision_objects(size);

    for (int i = 0; i < size; i++)
    {

        PCL_Point p;
        geometry_msgs::PointStamped tfP;
        // PCL_Point rgbdP = cloud_filtered->points.at(distr(eng));
        PCL_Point rgbdP = cloud_filtered->points.at(i);

        if (abs(rgbdP.z) > 0.41 && rgbdP.x > 0.3) //if (abs(rgbdP.z) > 0.41 && rgbdP.x > 0.3)
        {
            moveit_msgs::CollisionObject collision_objects_temp;

            collision_objects_temp.header.frame_id = "panda_link0";
            collision_objects_temp.id = "object_" + std::to_string(i);

            /* Define the primitive and its dimensions. */
            collision_objects_temp.primitives.resize(1);
            collision_objects_temp.primitives[0].type = collision_objects[0].primitives[0].BOX;
            collision_objects_temp.primitives[0].dimensions.resize(3);
            collision_objects_temp.primitives[0].dimensions[0] = 0.03;
            collision_objects_temp.primitives[0].dimensions[1] = 0.03;
            collision_objects_temp.primitives[0].dimensions[2] = 0.03;

            /* Define the pose of the object. */
            collision_objects_temp.primitive_poses.resize(1);
            collision_objects_temp.primitive_poses[0].position.x = rgbdP.x;
            collision_objects_temp.primitive_poses[0].position.y = rgbdP.y;
            collision_objects_temp.primitive_poses[0].position.z = rgbdP.z;

            // END_SUB_TUTORIAL
            collision_objects_temp.operation = collision_objects[0].ADD;

            collision_objects.at(i) = collision_objects_temp;
        }
    }
    if (lock == true)
    {
        lock_collision_objects_.assign(collision_objects.begin(), collision_objects.end());
    }
    planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void ScenePreception::add_pcd2_collision_objects(const sensor_msgs::PointCloud2 point_cloud2, bool lock)
{
    PCL_Cloud::Ptr rgbdCloud(new PCL_Cloud);
    pcl::fromROSMsg(point_cloud2, *rgbdCloud);

    pcl::StopWatch time;
    std::cout << "cloud_xyzrgb: " << rgbdCloud->size() << " data points." << std::endl;
    PCL_Cloud::Ptr cloud_filtered(new PCL_Cloud);
    /****** 体素下采样 ********/
    pcl::VoxelGrid<PCL_Point> sor;
    sor.setInputCloud(rgbdCloud);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor.filter(*cloud_filtered);

    std::cout << "cloud_filtered: " << cloud_filtered->size() << " data points." << std::endl;
    std::cout << "下采样分割运行时间" << time.getTime() << "ms" << std::endl;

    int size = cloud_filtered->size();
    std::random_device rd;     //Get a random seed from the OS entropy device, or whatever
    std::mt19937_64 eng(rd()); //Use the 64-bit Mersenne Twister 19937 generator

    std::uniform_int_distribution<unsigned long long> distr(0, size); // distribution in range [1, 6]

    std::vector<std::string> object_ids;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    for (int i = 0; i < size; i++)
    {

        PCL_Point p;
        geometry_msgs::PointStamped tfP;
        // PCL_Point rgbdP = cloud_filtered->points.at(distr(eng));
        PCL_Point rgbdP = cloud_filtered->points.at(i);

        if (abs(rgbdP.z) > 0.41 && rgbdP.x > 0.3)
        {
            moveit_msgs::CollisionObject collision_objects_temp;

            collision_objects_temp.header.frame_id = "panda_link0";
            collision_objects_temp.id = "object_" + std::to_string(i);

            /* Define the primitive and its dimensions. */
            collision_objects_temp.primitives.resize(1);
            collision_objects_temp.primitives[0].type = collision_objects[0].primitives[0].BOX;
            collision_objects_temp.primitives[0].dimensions.resize(3);
            collision_objects_temp.primitives[0].dimensions[0] = 0.03;
            collision_objects_temp.primitives[0].dimensions[1] = 0.03;
            collision_objects_temp.primitives[0].dimensions[2] = 0.03;

            /* Define the pose of the object. */
            collision_objects_temp.primitive_poses.resize(1);
            collision_objects_temp.primitive_poses[0].position.x = rgbdP.x;
            collision_objects_temp.primitive_poses[0].position.y = rgbdP.y;
            collision_objects_temp.primitive_poses[0].position.z = rgbdP.z;

            // END_SUB_TUTORIAL
            collision_objects_temp.operation = collision_objects[0].ADD;

            collision_objects.push_back(collision_objects_temp);
        }
    }
    if (lock == true)
    {
        lock_collision_objects_.assign(collision_objects.begin(), collision_objects.end());
    }
    planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void ScenePreception::add_grasp_collision_objects(const geometry_msgs::Pose object_pose)
{
    moveit_msgs::CollisionObject collision_object;

    // // Add the first table where the cube will originally be kept.
    collision_object.id = "object";
    collision_object.header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = 0.08;
    collision_object.primitives[0].dimensions[1] = 0.08;
    collision_object.primitives[0].dimensions[2] = 0.08;
    // collision_object.primitives[0].dimensions[0] = 0.001;
    // collision_object.primitives[0].dimensions[1] = 0.001;
    // collision_object.primitives[0].dimensions[2] = 0.001;

    /* Define the pose of the table. */
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position = object_pose.position;
    // END_SUB_TUTORIAL
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_.applyCollisionObject(collision_object);

    std::map<std::string, moveit_msgs::CollisionObject> obj_map = planning_scene_interface_.getObjects();
    for (std::map<std::string, moveit_msgs::CollisionObject>::iterator it = obj_map.begin(); it != obj_map.end(); it++)
    {
        moveit_msgs::CollisionObject obj_collision = (*it).second;

        // std::cout << (*it).first << std::endl;
        // std::cout << collision_objects[0].primitive_poses[0] << obj_collision.primitive_poses[0].position << std::endl;
        bool flag_protection_id = false;
        for (int i = 0; i < protection_objects_id_.size(); i++)
        {
            if ((*it).first == protection_objects_id_.at(i))
                flag_protection_id = true;
        }
        if (flag_protection_id == false)
        {
            if ((collision_object.primitive_poses[0].position.x - 0.12 < obj_collision.primitive_poses[0].position.x &&
                 obj_collision.primitive_poses[0].position.x < collision_object.primitive_poses[0].position.x + 0.12) &&
                (collision_object.primitive_poses[0].position.y - 0.12 < obj_collision.primitive_poses[0].position.y &&
                 obj_collision.primitive_poses[0].position.y < collision_object.primitive_poses[0].position.y + 0.12) &&
                (collision_object.primitive_poses[0].position.z - 0.2 < obj_collision.primitive_poses[0].position.z &&
                 obj_collision.primitive_poses[0].position.z < collision_object.primitive_poses[0].position.z + 0.2))
            {
                obj_collision.primitive_poses[0].position.x = 100;
                obj_collision.primitive_poses[0].position.y = 100;
                obj_collision.primitive_poses[0].position.z = 100;
                // planning_scene_interface.removeCollisionObjects(remove_obg_id);
                planning_scene_interface_.applyCollisionObject(obj_collision);
            }
        }
    }
}

void ScenePreception::recover_locked_obj()
{
    planning_scene_interface_.applyCollisionObjects(lock_collision_objects_);
}

void ScenePreception::remove_unprotected_collision_objects()
{
    std::map<std::string, moveit_msgs::CollisionObject> obj_map = planning_scene_interface_.getObjects();
    for (std::map<std::string, moveit_msgs::CollisionObject>::iterator it = obj_map.begin(); it != obj_map.end(); it++)
    {
        moveit_msgs::CollisionObject obj_collision = (*it).second;

        // std::cout << (*it).first << std::endl;
        // std::cout << collision_objects[0].primitive_poses[0] << obj_collision.primitive_poses[0].position << std::endl;
        bool flag_protection_id = false;
        for (int i = 0; i < protection_objects_id_.size(); i++)
        {
            if ((*it).first == protection_objects_id_.at(i))
                flag_protection_id = true;
        }
        if (flag_protection_id == false)
        {
            {
                obj_collision.primitive_poses[0].position.x = 100;
                obj_collision.primitive_poses[0].position.y = 100;
                obj_collision.primitive_poses[0].position.z = 100;
                // planning_scene_interface.removeCollisionObjects(remove_obg_id);
                planning_scene_interface_.applyCollisionObject(obj_collision);
            }
        }
    }
}

void ScenePreception::add_collision_obj(moveit_msgs::CollisionObject obj)
{
    planning_scene_interface_.applyCollisionObject(obj);
}

void ScenePreception::add_collision_objs(moveit_msgs::CollisionObject objs)
{
    planning_scene_interface_.applyCollisionObject(objs);
}

void ScenePreception::add_protection_obj(std::string id)
{
    protection_objects_id_.push_back(id);
}

void ScenePreception::remove_protection_obj(std::string id)
{

    //在vector中查找指定元素
    std::vector<std::string>::iterator iter = std::find(protection_objects_id_.begin(), protection_objects_id_.end(), id);

    //删除指定元素
    if (iter != protection_objects_id_.end())
        protection_objects_id_.erase(iter);
}