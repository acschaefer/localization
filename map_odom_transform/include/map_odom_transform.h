#ifndef MAP_ODOM_TRANSFORM_H_
#define MAP_ODOM_TRANSFORM_H_ MAP_ODOM_TRANSFORM_H_

#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class MapOdomTransform
{
protected:
    ros::NodeHandle node_handle_;
    geometry_msgs::TransformStamped map_odom_transform_;
    ros::Subscriber initial_pose_subscriber_;
    tf::TransformBroadcaster map_odom_transform_broadcaster_;


public:
    MapOdomTransform(ros::NodeHandle node_handle)
        : node_handle_(node_handle)
    {
        map_odom_transform_.header.frame_id = "map";
        map_odom_transform_.child_frame_id  = "odom";

        initial_pose_subscriber_ = node_handle_.subscribe(
                    "initialpose", 10, &MapOdomTransform::initial_pose_callback, this);
    }


    void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& initial_pose)
    {

        map_odom_transform_.header.stamp    = initial_pose.header.stamp;
        map_odom_transform_.transform.translation.x = initial_pose.pose.pose.position.x;
        map_odom_transform_.transform.translation.y = initial_pose.pose.pose.position.y;
        map_odom_transform_.transform.translation.z = initial_pose.pose.pose.position.z;
        map_odom_transform_.transform.rotation = initial_pose.pose.pose.orientation;

        broadcast_transform_map_odom();
    }


    void broadcast_transform_map_odom()
    {
        map_odom_transform_.header.stamp = ros::Time::now();
        map_odom_transform_broadcaster_.sendTransform(map_odom_transform_);
    }
};


#endif
