#include <ros/ros.h>

#include "map_odom_transform.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_odom_transform");
    ros::NodeHandle node_handle;

    MapOdomTransform transform;

    ros::spin();

    return EXIT_SUCCESS;
}
