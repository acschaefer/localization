#ifndef MOTION_MODEL_4D_H_
#define MOTION_MODEL_4D_H_ MOTION_MODEL_4D_H_

// Base class.
#include "localizer/motion_model_3d.h"


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


    /// Set the robot start pose.
    void set_start_pose(tf::Transform start_pose)
    {
        // Make sure the pose is 4D. Set all other coordinates to 0.
        tf::Matrix3x3 rotation(start_pose.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);
        roll = pitch = 0.0;
        rotation.setRPY(roll, pitch, yaw);
        tf::Quaternion start_quaternion;
        rotation.getRotation(start_quaternion);
        start_pose.setRotation(start_quaternion);

        start_pose_ = start_pose;
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
        GaussNumberGenerator z_generator(start_pose_.getOrigin().z(), var_z_);
        for (int p = 0; p < particles.size(); p++)
        {
            tf::Vector3 position(particles[p].pose.getOrigin());
            position.setZ(z_generator());
            particles[p].pose = tf::Transform(particles[p].pose.getRotation(),
                                              position);
        }
    }

    /// Applies noisy motion to all particles.
    /// Samples robot poses based on the last movement
    /// and the given motion uncertainty parameters.
    /// \param[in] movement robot movement w.r.t. the robot frame.
    /// \param[in, out] particles particles to move.
    virtual void move_particles(const tf::Transform& movement,
                                std::vector<Particle>& particles)
    {
        // Move the particles in x, y, and yaw.
        MotionModel3d::move_particles(movement, particles);

        // Add noise to the z-coordinate.
        GaussNumberGenerator z_generator(
                    0.0,
                    alpha_[4] * movement.getOrigin().length());

        for (int p = 0; p < particles.size(); p++)
            particles[p].pose.getOrigin()
                    += tf::Vector3(0.0, 0.0, z_generator());
    }
};


#endif
