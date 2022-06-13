#ifndef USER_SERVER_ROS_H
#define USER_SERVER_ROS_H

#include <string>
#include <thread>   
#include <mutex>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <random>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/PositionTarget.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <tf/tf.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

// class user_server_ros
// {
//     private:

//         ros::NodeHandle _nh;
//         ros::Publisher _goal_pub, _pcl_pub;
        
//         ros::Subscriber _pose_sub;

//         ros::Timer _target_tracker_timer;

//         geometry_msgs::Point goal_pose;

//         std::string _id, _odom_or_pose;

//         Eigen::Vector3d goal;

//         pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;        

//         void traj_optimization_update_timer(const ros::TimerEvent &);
        
//         void command_update_timer_idx(const ros::TimerEvent &);

//         void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

//         void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

//         void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

//         void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);


//     public:
    
//         trajectory_server::bspline_server::pva_cmd cmd;
//         trajectory_msgs::JointTrajectoryPoint traj_vector_msg;
//         geometry_msgs::PoseStamped pose;

//         user_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
//         {
//             _nh.param<std::string>("agent_id", _id, "drone0");
//             _nh.param<std::string>("odom_or_pose", _odom_or_pose, "pose");

//             /** @brief Subscriber that receives position data of uav */
//             if (_odom_or_pose.compare("pose") == 0)
//                 _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
//                     "/" + _id + "/mavros/local_position/pose", 20, &user_server_ros::pose_callback, this);
//             else if (_odom_or_pose.compare("odom") == 0)
//                 _pose_sub = _nh.subscribe<nav_msgs::Odometry>(
//                     "/" + _id + "/mavros/local_position/pose", 20, &user_server_ros::odom_callback, this);
//             else
//                 throw std::logic_error("[ERROR] no pose data subscriber found");

//             /** @brief Publisher that publishes goal vector */
//             _goal_pub = _nh.advertise<geometry_msgs::PoseStamped>(
//                     "/" + _id + "/goal", 20);

//             /** @brief Publisher that publishes pointcloud */
//             _pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>(
//                     "/" + _id + "/local_pcl", 20);
//         }

//         ~user_server_ros(){}

//         /** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
//         pcl::PointCloud<pcl::PointXYZ>::Ptr 
//             pcl2_converter(sensor_msgs::PointCloud2 _pc)
//         {
//             pcl::PCLPointCloud2 pcl_pc2;
//             pcl_conversions::toPCL(_pc, pcl_pc2);

//             pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
//             pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
            
//             return tmp_cloud;
//         }
        
// };

// #endif