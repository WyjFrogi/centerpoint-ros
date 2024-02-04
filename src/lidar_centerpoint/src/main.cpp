#include "lidar_centerpoint/node.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "lidar_centerpoint_node");
    ROS_INFO("##############");
    centerpoint::LidarCenterPointNode lidar_center_point_node;
    return 0;
}