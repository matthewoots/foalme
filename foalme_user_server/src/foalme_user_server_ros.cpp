/*
 * foalme_user_server_ros.cpp
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

#include <foalme_user_server_ros.h>

void user_server_ros::target_update_timer(const ros::TimerEvent &)
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
                    agents[i].pos.x(), 
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
                    agents[i].pos.y(), 
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
        // std::cout << "[user_server] " << KGRN << _id << KNRM << 
        //     " mission " << KGRN << agents[i].mission << KNRM << 
        //     " waypoint " << KGRN << agent_waypoints[agents[i].id].waypoints.size() << KNRM << std::endl;     

        if (agents[i].mission >= 0)
        {
            // std::cout << "[user_server] " << KGRN << _id << KNRM << 
            //     (agent_waypoints[agents[i].id].waypoints[agents[i].mission] - agents[i].pos).norm() << KNRM << std::endl;
            if ((agent_waypoints[agents[i].id].waypoints[agents[i].mission] - agents[i].pos).norm() >= 0.4)
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
        {
            continue;
        }
        
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
                continue;
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

        if (_logger_not_started)
        {
            _logging_timer.start();
            _logger_not_started = false;
        }
        
        std::cout << "[user_server] " << KGRN << 
            "published waypoint " << agents[i].mission << KNRM << std::endl;
        
    }
}

void user_server_ros::cloud_update_timer(const ros::TimerEvent &)
{
    if (!_valid_cloud)
        return;
    
    cloud_msg.header.frame_id = "/world";
    _pcl_pub.publish(cloud_msg);

}

void user_server_ros::logging_timer(const ros::TimerEvent &)
{

    for (int i = 0; i < _agent_number; i++)
    {
        double safety_radius = 0.3;
        std::string str = "";
        for (int j = 0; j < _agent_number; j++)
        {
            if (i == j)
                continue;
            double distance = (agents[i].pos - agents[j].pos).norm();
            // safety_radius * 2 because if both radius were to intersect with each other
            if (distance < safety_radius * 2)
            {
                str += to_string(j) + " ";
            }
        }
        std::lock_guard<std::mutex> agents_lock(agents_mutex);

        CSVWriter csv;
        csv.newRow() << 
            (agents[i].t - module_start_time).toSec() << 
            "computation_time" << 
            agents[i].vel.norm() <<
            agents[i].distance << 
            str;
        csv.writeToFile(_log_file_name[i], true);
    }
}

void user_server_ros::pose_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    int vector_index = -1;

    Eigen::Vector3d nwu_velocity = Eigen::Vector3d(
        msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Affine3d nwu_transform = Eigen::Affine3d::Identity();
    // Local position in NWU frame
    nwu_transform.translation() = Eigen::Vector3d(
        msg->position[0], msg->position[1], msg->position[2]);
    // Local rotation in NWU frame
    // msg->pose.effort.w
    // msg->pose.effort.x
    // msg->pose.effort.y
    // msg->pose.effort.z
    nwu_transform.linear() = Eigen::Quaterniond(
        msg->effort[0], msg->effort[1], msg->effort[2], msg->effort[3]).toRotationMatrix();

    std::lock_guard<std::mutex> agents_lock(agents_mutex);

    if (!agents.empty())
    {
        int idx = stoi(msg->name[0]);

        if ((msg->header.stamp - agents[idx].t).toSec() > 0)
        {
            if (!(isnan(agents[idx].pos.x()) || 
                isnan(agents[idx].pos.y()) ||
                isnan(agents[idx].pos.z())))
            {
                agents[idx].distance += 
                    (nwu_transform.translation() - agents[idx].pos).norm();
            }

            agents[idx].pos = nwu_transform.translation();
            agents[idx].q = nwu_transform.linear();
            agents[idx].vel = nwu_velocity;
            agents[idx].t = msg->header.stamp;

            return;
        }

    }

}