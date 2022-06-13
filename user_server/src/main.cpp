#include <iostream>
#include <ros/ros.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "user_server_node");
    ros::NodeHandle nh("~");
    
    ros::spin();
    return 0;

}