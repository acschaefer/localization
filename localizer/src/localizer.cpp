#include <visualization_msgs/MarkerArray.h>

#include "particle_filter.h"
#include "motion_model_3d.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle node_handle;
    ros::Publisher particle_publisher
            = node_handle.advertise<visualization_msgs::MarkerArray>("particles", 1u);

    std::vector<double> alpha(4, 0.1);
    boost::shared_ptr<MotionModel3d> motion_model
            = boost::shared_ptr<MotionModel3d>(new MotionModel3d(alpha));

    ParticleFilter particle_filter(motion_model);
    particle_filter.init(1e4);

    ros::Rate rate(1);
    while (ros::ok())
    {
        tf::Transform movement(tf::Transform::getIdentity());
        movement.getOrigin().setX(0.1);
        particle_filter.update_motion(movement);

        tf::Vector3 mean = particle_filter.get_mean();
        std::cout << "[" << mean[0] << "; " << mean[1] << "; " << mean[2] << "]" << std::endl;

        // Do not publish if no one is listening.S
        if (particle_publisher.getNumSubscribers() < 1)
            continue;

        // Create an empty message.
        visualization_msgs::MarkerArray marker_array;

        // Fill the marker array.
        std::vector<Particle> particles(particle_filter.get_particles());
        for (int i = 0; i < particles.size(); i++)
        {
            geometry_msgs::Pose marker_pose;
            tf::poseTFToMsg(particles[i].get_pose(), marker_pose);

            // Choose the size of the sphere so that it represents
            // the weight of the particle.
            geometry_msgs::Vector3 marker_scale;
            marker_scale.x = marker_scale.y = marker_scale.z
                    = std::max(0.1, particles[i].get_weight());

            // Color the particles red.
            std_msgs::ColorRGBA marker_color;
            marker_color.r = marker_color.a = 0.7f;
            marker_color.g = marker_color.b = 0.0f;

            visualization_msgs::Marker marker;
            marker.header.stamp       = ros::Time::now();
            marker.header.frame_id    = "map";
            marker.id                 = i;
            marker.type               = visualization_msgs::Marker::SPHERE;
            marker.action             = visualization_msgs::Marker::ADD;
            marker.pose               = marker_pose;
            marker.scale              = marker_scale;
            marker.color              = marker_color;
            marker.lifetime           = ros::Duration();
            marker.frame_locked       = true;

            marker_array.markers.push_back(marker);
        }

        particle_publisher.publish(marker_array);

        ros::spinOnce();
    }

    return 0;
}
