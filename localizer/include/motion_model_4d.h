#ifndef MOTION_MODEL_4D_H_
#define MOTION_MODEL_H_4D_ MOTION_MODEL_4D_H_

// Base class.
#include "motion_model_3d.h"


/// 4D motion model for use with class ParticleFilter for robot localization.
/// 4D means that particles move in the (x, y, z, yaw) space.
/// All other coordinates are zero.
class MotionModel4d : public MotionModel3d
{
protected:
    /// Variance of the start pose in z-direction.
    double var_z_;


public:
    /// Default constructor.
    /// Initializes all members to default values.
    MotionModel4d()
        : MotionModel3d()
    {
        alpha_.push_back(1.0);
        var_z_ = 1.0;
    }


    /// Set the start pose variances.
    void set_start_pose_variance(double var_xy, double var_z, double var_yaw)
    {
        var_xy_     = var_xy;
        var_z_      = var_z;
        var_yaw_    = var_yaw;
    }


    /// Scatter all particles around the previously given start pose according
    /// to the given variance values.
    void init(std::vector<Particle>& particles)
    {
        // Scatter the particles in x, z, and yaw.
        MotionModel3d::init(particles);

        // Scatter the particles along the z-axis.
        GaussNumberGenerator z_generator(0.0, var_z_);
        for (int p = 0; p < particles.size(); p++)
        {
            tf::Vector3 position(particles[p].get_pose().getOrigin());
            position.setZ(z_generator());
            particles[p].set_pose(
                        tf::Transform(particles[p].get_pose().getRotation(),
                                      position));
        }
    }


    /// Sample a robot pose based on the last robot pose, the last movement
    /// and the previously specified motion uncertainty parameters.
    tf::Transform sample_pose(const tf::Transform& last_pose,
                              const tf::Transform& movement)
    {
        // Sample the pose in x, y, and yaw.
        tf::Transform new_pose = MotionModel3d::sample_pose(last_pose, movement);

        // Add noise to the z-coordinate.
        GaussNumberGenerator z_generator(
                    new_pose.getOrigin().getZ(),
                    alpha_[4]*movement.getOrigin().length());
        tf::Vector3 new_position(new_pose.getOrigin());
        new_position.setZ(z_generator());
        new_pose.setOrigin(new_position);

        return new_pose;
    }
};


#endif
