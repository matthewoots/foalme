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
#include <sensor_msgs/JointState.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

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

class user_server_ros
{
    private:

        struct agent_state
        {
            int id;
            Eigen::Vector3d pos;
            Eigen::Quaterniond q;
            ros::Time t;
        };

        ros::NodeHandle _nh;

        ros::Publisher _goal_pub, _pcl_pub;
        
        ros::Subscriber _pose_sub;

        ros::Timer _target_tracker_timer, _cloud_timer;

        geometry_msgs::Point goal_pose;

        std::string _file_location;

        Eigen::Vector3d goal;

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;  

        double _tracker_timer_hz, _cloud_hz; 

        vector<agent_state> agents;   

        sensor_msgs::PointCloud2 cloud_msg;

        void target_update_timer(const ros::TimerEvent &)
        {
            std::string _id;
            /** @brief Publisher that publishes goal vector */
            _goal_pub = _nh.advertise<geometry_msgs::PoseStamped>("/" + _id + "/goal", 20);
            
        }

        void cloud_update_timer(const ros::TimerEvent &)
        {
            _pcl_pub.publish(cloud_msg);
        }
        
        void pose_callback(const sensor_msgs::JointState::ConstPtr &msg)
        {
            int vector_index = -1;
            Affine3d nwu_transform = Affine3d::Identity();
            // Local position in NWU frame
            nwu_transform.translation() = Vector3d(
                msg->position[0], msg->position[1], msg->position[2]);
            // Local rotation in NWU frame
            // msg->pose.effort.w
            // msg->pose.effort.x
            // msg->pose.effort.y
            // msg->pose.effort.z
            nwu_transform.linear() = Quaterniond(
                msg->effort[0], msg->effort[1], msg->effort[2], msg->effort[3]).toRotationMatrix();
            
            if (!agents.empty())
            {
                int idx = stoi(msg->name[0]);
                for (int i = 0; i < agents.size(); i++)
                {
                    if (idx == agents[i].id)
                    {
                        vector_index = i;
                        break;
                    }
                }

                if (vector_index < 0)
                {
                    agent_state new_agent;
                    new_agent.id = idx;
                    new_agent.pos = nwu_transform.translation();
                    new_agent.q = nwu_transform.linear();
                    new_agent.t = msg->header.stamp;

                    agents.push_back(new_agent);
                    return;
                }

                if ((msg->header.stamp - agents[idx].t).toSec() > 0)
                {
                    agents[idx].pos = nwu_transform.translation();
                    agents[idx].q = nwu_transform.linear();
                    agents[idx].t = msg->header.stamp;
                    return;
                }

            }
            else
            {
                int idx = stoi(msg->name[0]);
                agent_state new_agent;
                new_agent.id = idx;
                new_agent.pos = nwu_transform.translation();
                new_agent.q = nwu_transform.linear();
                new_agent.t = msg->header.stamp;
                
                agents.push_back(new_agent);
            }

        }

    public:

        user_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("file_location", _file_location, "/cloud.pcd");
            _nh.param<double>("tracker_timer_hz", _tracker_timer_hz, 5.0);
            _nh.param<double>("cloud_timer_hz", _cloud_hz, 2.0);

            /** @brief Subscriber that receives position data of uavs published by simple_quad_simulator */
            _pose_sub = _nh.subscribe<sensor_msgs::JointState>(
                "/agent/pose", 200, &user_server_ros::pose_callback, this);

            /** @brief Publisher that publishes pointcloud */
            _pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cloud", 20);

            /** @brief Timer that handles drone tracking */
		    _target_tracker_timer = _nh.createTimer(
                ros::Duration(1/_tracker_timer_hz), &user_server_ros::target_update_timer, this, false, false);

            /** @brief Timer that handles drone state at each time frame */
            _cloud_timer = _nh.createTimer(
                ros::Duration(1/_cloud_hz), &user_server_ros::cloud_update_timer, this, false, false);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the point cloud

            ifstream myFile(_file_location);
            if(!myFile.fail()){
                pcl::io::loadPCDFile<pcl::PointXYZ>(_file_location, *cloud);// Load the pcd file

                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);
            }    
            else
                std::cout << "[user_server] " << KRED << 
                    "No pcd file found" << KNRM << std::endl;

            _target_tracker_timer.start();
            _cloud_timer.start();
        }

        ~user_server_ros(){}

        /** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            pcl2_converter(sensor_msgs::PointCloud2 _pc)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(_pc, pcl_pc2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
            
            return tmp_cloud;
        }
        
};

#endif