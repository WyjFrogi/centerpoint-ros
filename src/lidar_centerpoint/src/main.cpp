#include "lidar_centerpoint/node.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "lidar_centerpoint_node");
    centerpoint::LidarCenterPointNode lidar_center_point_node;
    ros::spin();
    return 0;
}