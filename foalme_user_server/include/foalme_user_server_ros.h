/*
 * foalme_user_server_ros.h
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

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

#include <trajectory_msgs/JointTrajectory.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf/tf.h>

#include <CSVWriter.h>
#include "boost/date_time/posix_time/posix_time.hpp"

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
            Eigen::Vector3d vel;
            Eigen::Quaterniond q;
            double distance;
            ros::Time t;
            int mission;
            ros::Time trajectory_start_time;
            double compute_time;
        };

        struct agent_waypoint
        {
            vector<Eigen::Vector3d> waypoints;
            int id;
        };

        ros::NodeHandle _nh;

        ros::Publisher _goal_pub, _pcl_pub;
        
        ros::Subscriber _pose_sub, _trajectory_sub;

        ros::Timer _target_tracker_timer, _cloud_timer, _logging_timer;

        std::string _file_location, _single_or_multi, _formation;

        vector<std::string> _log_file_name;

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;  

        bool _valid_cloud, _logger_not_started = true;

        int _agent_number, _logging_counter;

        double _tracker_timer_hz, _cloud_hz, _logging_hz;

        std::mutex agents_mutex; 

        vector<agent_state> agents, stats;   

        sensor_msgs::PointCloud2 cloud_msg;

        vector<agent_waypoint> agent_waypoints;
        // vector<Eigen::Vector3d> waypoints;

        ros::Time module_start_time, logging_start_time;

        void target_update_timer(const ros::TimerEvent &);

        void cloud_update_timer(const ros::TimerEvent &);

        void logging_timer(const ros::TimerEvent &);
        
        void pose_callback(const sensor_msgs::JointState::ConstPtr &msg);

        void trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

    public:

        user_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("file_location", _file_location, "cloud.pcd");
            _nh.param<double>("tracker_timer_hz", _tracker_timer_hz, 2.0);
            _nh.param<double>("cloud_timer_hz", _cloud_hz, 2.0);
            _nh.param<double>("logging_timer_hz", _logging_hz, 2.0);
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

            /** @brief Subscriber that receives trajectory data of uavs published by trajectory_server_ros */
            _trajectory_sub = _nh.subscribe<trajectory_msgs::JointTrajectory>(
                "/trajectory/points", 200, &user_server_ros::trajectory_callback, this);

            /** @brief Publisher that publishes pointcloud */
            _pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cloud", 20);

            /** @brief Timer that handles drone tracking */
		    _target_tracker_timer = _nh.createTimer(
                ros::Duration(1/_tracker_timer_hz), &user_server_ros::target_update_timer, this, false, false);

            /** @brief Timer that handles drone state at each time frame */
            _cloud_timer = _nh.createTimer(
                ros::Duration(1/_cloud_hz), &user_server_ros::cloud_update_timer, this, false, false);

            /** @brief Timer that handles logging */
		    _logging_timer = _nh.createTimer(
                ros::Duration(1/_logging_hz), &user_server_ros::logging_timer, this, false, false);

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

            global_cloud = cloud;

            module_start_time = ros::Time::now();
            logging_start_time = ros::Time::now();
            _target_tracker_timer.start();
            _cloud_timer.start();

            std::string path(getenv("HOME"));
            path += "/Documents/";

            for (int i = 0; i < _agent_number; i++)
            {
                _log_file_name.push_back(path + "drone" + to_string(i) + "_log_file.csv");
                // Write headers into the file
                CSVWriter csv;
                csv << "time" << "computation_time" << "speed" << "total_distance" << "collision_detection";
                csv.writeToFile(_log_file_name[i]);

                agent_state new_agent;
                new_agent.distance = 0.0;
                new_agent.id = i;
                new_agent.mission = -1;
                new_agent.compute_time = 0.0;
                new_agent.t = ros::Time::now();
                new_agent.trajectory_start_time = ros::Time::now();
                new_agent.pos = Eigen::Vector3d(sqrt(-1), sqrt(-1), sqrt(-1));
                agents.push_back(new_agent);
                new_agent.id = 0;
                stats.push_back(new_agent);
            }

        }

        ~user_server_ros()
        {
            std::string path(getenv("HOME"));
            path += "/Documents/";

            std::string _log_file_name = 
                path + "total_stats_log_file.csv";
            // Write headers into the file
            CSVWriter csv;
            csv.newRow() << "drone" << "average_computation_time" << "total_distance" << "flight_time";
            for (int i = 0; i < _agent_number; i++)
            {
                csv.newRow() << 
                    i <<
                    stats[i].compute_time <<
                    stats[i].distance <<
                    (stats[i].t - stats[i].trajectory_start_time).toSec();
            }
            cout << csv << endl;
            csv.writeToFile(_log_file_name);
        }

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

        bool kdtree_collide_pcl_bool(
            Eigen::Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr obs, double c)
        {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

            kdtree.setInputCloud(obs);

            pcl::PointXYZ searchPoint;
            searchPoint.x = point.x();
            searchPoint.y = point.y();
            searchPoint.z = point.z();

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

            float radius = (float)c;

            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
                if (pointIdxRadiusSearch.empty())
                    return false;

                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    return true;
                }
            }

            return false;
        }
        
};

#endif