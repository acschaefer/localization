#include <ros/ros.h>

#include "map_odom_transform.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_odom_transform");
    ros::NodeHandle node_handle;

    MapOdomTransform transform(node_handle);

    ros::Rate rate(1.0);
    while (ros::ok())
    {
        transform.broadcast_transform_map_odom();
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
