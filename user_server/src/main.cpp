#include <iostream>
#include <ros/ros.h>
#include "user_server_ros.h"

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "user_server_node");
    ros::NodeHandle nh("~");
    int threads = 2;
    ros::MultiThreadedSpinner spinner(threads);
    user_server_ros user_server_ros(nh);
    spinner.spin();
    return 0;

}