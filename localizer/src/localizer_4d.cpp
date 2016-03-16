#include <visualization_msgs/MarkerArray.h>

#include "particle_filter.h"
#include "motion_model_4d.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer4d");
    ros::NodeHandle node_handle;
    ros::Publisher particle_publisher
            = node_handle.advertise<visualization_msgs::MarkerArray>("particles", 1u);

    boost::shared_ptr<MotionModel4d> motion_model
            = boost::shared_ptr<MotionModel4d>(new MotionModel4d());
    std::vector<double> alpha(5, 0.0);
    alpha[0] = 0.4;
    alpha[1] = 0.4;
    alpha[2] = 0.1;
    alpha[3] = 0.1;
    alpha[4] = 0.1;
    motion_model->set_alpha(alpha);
    motion_model->set_start_pose(tf::Transform::getIdentity());
    motion_model->set_start_pose_variance(5.0, 1.0, (10.0/180.0) * M_PI);

    ParticleFilter<MotionModel4d, NoSensorModel> particle_filter;
    particle_filter.set_motion_model(motion_model);
    particle_filter.set_n_particles(1e4);

    ros::Rate rate(3);
    while (ros::ok())
    {
        tf::Transform movement(tf::Transform::getIdentity());
        movement.getOrigin().setX(0.1);
        particle_filter.update_motion(movement);

        tf::Transform mean = particle_filter.get_mean();
        std::cout << "[" << mean.getOrigin()[0] << "; "
                  << mean.getOrigin()[1] << "; "
                  << mean.getOrigin()[2] << "]" << std::endl;

        // Do not publish if no one is listening.
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
            marker_scale.x = std::max(0.15, particles[i].get_weight());
            marker_scale.y = marker_scale.z = marker_scale.x * 0.1;

            // Color the particles red.
            std_msgs::ColorRGBA marker_color;
            marker_color.r = marker_color.a = 0.7f;
            marker_color.g = marker_color.b = 0.0f;

            visualization_msgs::Marker marker;
            marker.header.stamp       = ros::Time::now();
            marker.header.frame_id    = "map";
            marker.id                 = i;
            marker.type               = visualization_msgs::Marker::ARROW;
            marker.action             = visualization_msgs::Marker::ADD;
            marker.pose               = marker_pose;
            marker.scale              = marker_scale;
            marker.color              = marker_color;
            marker.lifetime           = ros::Duration(10.0);
            marker.frame_locked       = true;

            marker_array.markers.push_back(marker);
        }

        particle_publisher.publish(marker_array);

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
