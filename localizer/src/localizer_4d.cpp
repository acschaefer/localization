#include <geometry_msgs/PoseArray.h>

#include "localizer/particle_filter.h"
#include "localizer/motion_model_4d.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer4d");
    ros::NodeHandle node_handle;
    ros::Publisher particle_publisher = node_handle.advertise<geometry_msgs::PoseArray>("particles", 1u);

    boost::shared_ptr<MotionModel4d> motion_model = boost::shared_ptr<MotionModel4d>(new MotionModel4d());
    std::vector<double> alpha(5, 0.0);
    alpha[0] = 0.4;
    alpha[1] = 0.4;
    alpha[2] = 0.1;
    alpha[3] = 0.1;
    alpha[4] = 0.1;
    motion_model->set_alpha(alpha);
    motion_model->set_start_pose(tf::Transform::getIdentity());
    motion_model->set_start_pose_variance(5.0, 1.0, (10.0/180.0)*M_PI);

    ParticleFilter<MotionModel4d, NoSensorModel> particle_filter;
    particle_filter.set_motion_model(motion_model);
    particle_filter.set_n_particles(1000u);
    particle_filter.init();

    tf::Vector3 translation(0.1, 0.0, 0.0);
    tf::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, 0.1);
    tf::Transform movement(rotation, translation);

    ros::Rate rate(5);
    while (ros::ok())
    {
        particle_filter.update_motion(movement);

        tf::Vector3 mean = particle_filter.get_mean();
        std::cout << "[" << mean[0] << "; " << mean[1] << "; " << mean[2] << "]" << std::endl;

        // Do not publish if no one is listening.
        if (particle_publisher.getNumSubscribers() < 1)
        {
            rate.sleep();
            continue;
        }

        // Create a particle cloud message.
        geometry_msgs::PoseArray pose_array;

        pose_array.header.stamp     = ros::Time::now();
        pose_array.header.frame_id  = "map";

        // Fill the cloud with particles.
        std::vector<Particle> particles(particle_filter.get_particles());
        for (size_t i = 0u; i < particles.size(); ++i)
        {
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(particles[i].pose, pose);

            pose_array.poses.push_back(pose);
        }

        particle_publisher.publish(pose_array);

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
