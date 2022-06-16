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
#include <mutex>

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
            int mission;
        };

        struct agent_waypoint
        {
            vector<Eigen::Vector3d> waypoints;
            int id;
        };

        ros::NodeHandle _nh;

        ros::Publisher _goal_pub, _pcl_pub;
        
        ros::Subscriber _pose_sub;

        ros::Timer _target_tracker_timer, _cloud_timer;

        std::string _file_location, _single_or_multi, _formation;

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;  

        bool _valid_cloud;

        int _agent_number;

        double _tracker_timer_hz, _cloud_hz;

        std::mutex agents_mutex; 

        vector<agent_state> agents;   

        sensor_msgs::PointCloud2 cloud_msg;

        vector<agent_waypoint> agent_waypoints;
        // vector<Eigen::Vector3d> waypoints;

        ros::Time module_start_time;

        void target_update_timer(const ros::TimerEvent &)
        {
            std::lock_guard<std::mutex> agents_lock(agents_mutex);
            if ((ros::Time::now() - module_start_time).toSec() < 3.0)
            {
                return;
            }
            
            if (agent_waypoints[0].waypoints.empty() && !agents.empty() && _agent_number > 1)
            {
                for (int i = 0; i < agents.size(); i++)
                {
                    // 1. antipodal
                    // 2. horizontal-line
                    // 3. vertical-line
                    // 4. top-down-facing
                    // 5. left-right-facing
                    if (_formation.compare("left-right-facing") == 0 || 
                        _formation.compare("vertical-line") == 0)
                    {
                        agent_waypoints[i].waypoints.push_back(
                            Eigen::Vector3d(
                            -agents[i].pos.x(), 
                            -agents[i].pos.y(), 
                            agents[i].pos.z()));
                    }

                    if (_formation.compare("antipodal") == 0)
                    {
                        Eigen::Vector3d opp_vector = Eigen::Vector3d(
                            -agents[i].pos.x(), 
                            -agents[i].pos.y(), 
                            agents[i].pos.z());
                        
                        double opp_vector_norm = opp_vector.norm();

                        agent_waypoints[i].waypoints.push_back(opp_vector);
                    }

                    if (_formation.compare("top-down-facing") == 0 || 
                        _formation.compare("horizontal-line") == 0)
                    {
                        agent_waypoints[i].waypoints.push_back(
                            Eigen::Vector3d(
                            -agents[i].pos.x(), 
                            -agents[i].pos.y(), 
                            agents[i].pos.z()));
                    }
                }
            }

            if (agents.empty())
                return;
            
            for (int i = 0; i < agents.size(); i++)
            {
                std::string _id;
                _id = "drone" + to_string(agents[i].id);

                if (agents[i].mission >= 0)
                {
                    if ((agent_waypoints[agents[i].id].waypoints[agents[i].mission] - agents[i].pos).norm() >= 0.3)
                        continue;
                }
                
                /** @brief Publisher that publishes goal vector */
                _goal_pub = _nh.advertise<geometry_msgs::PoseStamped>("/" + _id + "/goal", 40, true);
                ros::Time start_time = ros::Time::now();
                bool early_break = false;
                
                if (agent_waypoints[agents[i].id].waypoints.empty())
                    continue;

                if (agents[i].mission < (int)agent_waypoints[agents[i].id].waypoints.size()-1)
                {
                    agents[i].mission = agents[i].mission + 1;
                }
                else
                    return;
                
                geometry_msgs::PoseStamped goal;
                goal.pose.position.x = agent_waypoints[agents[i].id].waypoints[agents[i].mission].x(); 
                goal.pose.position.y = agent_waypoints[agents[i].id].waypoints[agents[i].mission].y(); 
                goal.pose.position.z = agent_waypoints[agents[i].id].waypoints[agents[i].mission].z();
                

                while (_goal_pub.getNumSubscribers() < 1) {
                    // wait for a connection to publisher
                    // you can do whatever you like here or simply do nothing
                    if ((ros::Time::now() - start_time).toSec() > 2.0)
                    {
                        early_break = true;
                        break;
                    }
                }

                if (early_break)
                    continue;

                _goal_pub.publish(goal);
                _goal_pub.publish(goal);
                
                start_time = ros::Time::now();
                if (agents[i].id == 0)
                {
                    // Somehow agent 0 will suffer from not receiving the command
                    // Hence, need to wait awhile longer
                    while ((ros::Time::now() - start_time).toSec() < 0.25)
                    {
                        // Wait
                    } 
                }
                else
                {
                    while ((ros::Time::now() - start_time).toSec() < 0.01)
                    {
                        // Wait
                    }
                }
                
                std::cout << "[user_server] " << KGRN << 
                    "published waypoint " << agents[i].mission << KNRM << std::endl;
                
            }
        }

        void cloud_update_timer(const ros::TimerEvent &)
        {
            if (!_valid_cloud)
                return;
            
            cloud_msg.header.frame_id = "/world";
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

            std::lock_guard<std::mutex> agents_lock(agents_mutex);

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
                    new_agent.mission = -1;

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
                new_agent.mission = -1;
                
                agents.push_back(new_agent);
            }

        }

    public:

        user_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("file_location", _file_location, "cloud.pcd");
            _nh.param<double>("tracker_timer_hz", _tracker_timer_hz, 5.0);
            _nh.param<double>("cloud_timer_hz", _cloud_hz, 2.0);
            _nh.param<int>("agents", _agent_number, 1);
            _nh.param<std::string>("single_or_multi", _single_or_multi, "single");
            _nh.param<std::string>("formation", _formation, "antipodal");
            std::vector<double> waypoint_vector;
            _nh.getParam("waypoints", waypoint_vector);

            if (_single_or_multi.compare("single") == 0)
            {
                _agent_number = 1;
                agent_waypoint aw;
                aw.id = 0;
                // number of waypoints 
                int size_of_waypoints = (int)waypoint_vector.size() / 3;
                
                for (int i = 0; i < size_of_waypoints; i++)
                {
                    aw.waypoints.push_back(
                        Eigen::Vector3d(waypoint_vector[3*i + 0], 
                        waypoint_vector[3*i + 1], 
                        waypoint_vector[3*i + 2]));
                }

                std::cout << "[user_server] size of waypoints: " << KGRN 
                    << aw.waypoints.size() << KNRM << std::endl;

                agent_waypoints.push_back(aw);
            }
            else if (_single_or_multi.compare("multi") == 0)
            {
                std::cout << "[user_server] " << KRED
                    << "multi agent mode detected" << KNRM << std::endl;
                for (int i = 0; i < _agent_number; i++)
                {
                    agent_waypoint aw;
                    aw.id = i;
                    agent_waypoints.push_back(aw);
                }
            }
            else
            {
                std::cout << "[user_server] " << KRED
                    << "_single_or_multi not valid" << KNRM << std::endl;
                return;
            }

            

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

            std::cout << "[user_server] Opening pcd file from " << KGRN << _file_location << KNRM << " ..." << std::endl;
            ifstream myFile(_file_location);
            if(!myFile.fail()){
                _valid_cloud = true;
                pcl::io::loadPCDFile<pcl::PointXYZ>(_file_location, *cloud);// Load the pcd file

                pcl::toROSMsg(*cloud, cloud_msg);
                std::cout << "[user_server] " << KGRN << 
                    "pcd file found" << KNRM << " with size " << cloud->points.size() << std::endl;
            }    
            else
            {
                _valid_cloud = false;
                std::cout << "[user_server] " << KRED << 
                    "No pcd file found" << KNRM << std::endl;
            }

            module_start_time = ros::Time::now();
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