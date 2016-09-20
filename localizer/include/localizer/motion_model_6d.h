#ifndef MOTION_MODEL_6D_H_
#define MOTION_MODEL_6D_H_ MOTION_MODEL_6D_H_

// Base class.
#include "localizer/motion_model.h"

// Eigen.
#include <eigen3/Eigen/Dense>


/// Simplified 6D motion model for use with class ParticleFilter for robot localization.
class MotionModel6d : public MotionModel
{
protected:
    /// Motion covariance.
    Eigen::Matrix<double,6,6> covariance_;

    /// Variance of the start pose.
    std::vector<double> start_variance_;


public:
    /// Default constructor.
    /// Initializes all members to default values.
    MotionModel6d()
        : covariance_(Eigen::Matrix<double,6,6>::Identity() * 0.1),
          start_variance_(6, 0.1)
    {
    }


    /// Set the start pose variances.
    virtual void set_start_pose_variance(std::vector<double> variance)
    {
        start_variance_ = variance;
    }


    /// Set the motion covariance.
    virtual void set_motion_covariance(Eigen::Matrix<double,6,6> covariance)
    {
        covariance_ = covariance;
    }


    /// Scatter all particles around the start pose according to the variance values.
    virtual void init(std::vector<Particle>& particles)
    {
        const tf::Matrix3x3 rotation(start_pose_.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        // Create randomly distributed start poses.
        GaussVectorGenerator random_origin(start_pose_.getOrigin(),
                                           tf::Vector3(start_variance_[0], start_variance_[1], start_variance_[2]));
        GaussVectorGenerator random_rotation(tf::Vector3(roll, pitch, yaw),
                                             tf::Vector3(start_variance_[3], start_variance_[4], start_variance_[5]));
        for (size_t p = 0; p < particles.size(); ++p)
        {
            particles[p].pose.setOrigin(random_origin());
            tf::Vector3 rotation(random_rotation());
            tf::Quaternion quaternion;
            quaternion.setRPY(rotation.x(), rotation.y(), rotation.z());
            particles[p].pose.setRotation(quaternion);
        }
    }

    /// Applies noisy motion to all particles.
    /// Samples robot poses based on the last movement and on the given motion uncertainty parameters.
    /// \param[in] movement robot movement w.r.t. the robot frame.
    /// \param[in, out] particles particles to move.
    virtual void move_particles(const tf::Transform& movement, std::vector<Particle>& particles)
    {
        // Compute the Euler angles of the rotation.
        tfScalar roll, pitch, yaw;
        const tf::Matrix3x3 rotation(movement.getRotation());
        rotation.getRPY(roll, pitch, yaw);

        // Compute the variance of the motion increment.
        Eigen::Matrix<double,6,1> increment;
        increment << movement.getOrigin().x(), movement.getOrigin().y(), movement.getOrigin().z(), roll, pitch, yaw;
        Eigen::Matrix<double,6,1> variance;
        variance = covariance_ * increment;

        // Add noise to the motion increments.
        GaussVectorGenerator random_translation(movement.getOrigin(),
                                                tf::Vector3(variance(0,0), variance(1,0), variance(2,0)));
        GaussVectorGenerator random_rotation(tf::Vector3(roll, pitch, yaw),
                                             tf::Vector3(variance(3,0), variance(4,0), variance(5,0)));
        for (size_t p = 0; p < particles.size(); ++p)
        {
            // Generate random translation and rotation.
            tf::Vector3 noisy_translation(random_translation());
            tf::Vector3 noisy_rotation_rpy(random_rotation());
            tf::Quaternion noisy_rotation;
            noisy_rotation.setRPY(noisy_rotation_rpy.x(), noisy_rotation_rpy.y(), noisy_rotation_rpy.z());
            tf::Transform noisy_movement(noisy_rotation, noisy_translation);

            // Move the particle.
            particles[p].pose = particles[p].pose * noisy_movement;
        }
    }
};


#endif
