#ifndef MOTION_MODEL_3D_H_
#define MOTION_MODEL_H_3D_ MOTION_MODEL_3D_H_

// Base class.
#include "motion_model.h"

// Random number generators required for sampling.
#include "random_generators.h"


/// 3D motion model for use with class ParticleFilter for robot localization.
/// 3D means that particles move in the (x, y, yaw) space.
/// All other coordinates are zero.
class MotionModel3d : public MotionModel
{
protected:
    /// ParticleFilter needs to access to init(), but no one else.
    /// Thus, make ParticleFilter this class' friend, no matter what
    /// the template parameters of ParticleFilter are.
    template<typename MotionModelT, typename SensorModelT>
    friend class ParticleFilter;


protected:
    /// Motion uncertainty parameters according to the probabilistic motion
    /// model described in the book "Probabilistic Robotics" by Thrun et al.
    std::vector<double> alpha_;

    /// Variance in the distance between the initial particle poses
    /// and the given start position.
    double var_xy_;

    /// Yaw-angle variance of the start pose.
    /// Unit: [rad].
    double var_yaw_;


public:
    /// Default constructor.
    /// Initializes members to default values.
    MotionModel3d()
        : alpha_(std::vector<double>(4, 1.0)),
          var_xy_(1.0), var_yaw_(0.1)
    {
    }


    /// Set the motion uncertainty parameters.
    void set_alpha(const std::vector<double>& alpha)
    {
        alpha_ = alpha;
    }


    /// Set the robot start pose.
    virtual void set_start_pose(tf::Transform start_pose)
    {
        // Make sure the pose is 3D. Set all other coordinates to 0.
        start_pose.getOrigin().setZ(0.0);
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
    void set_start_pose_variance(double var_xy, double var_yaw)
    {
        var_xy_     = var_xy;
        var_yaw_    = var_yaw;
    }


protected:
    /// Caculates the weighted mean pose of all particles.
    virtual tf::Transform get_mean(const std::vector<Particle>& particles)
    {
        tf::Vector3 mean_translation = MotionModel::get_mean(particles).getOrigin();

        double sum_sin_yaw, sum_cos_yaw;
        sum_sin_yaw = sum_cos_yaw = 0.0;
        for (int p = 0; p < particles.size(); p++)
        {
            tf::Matrix3x3 rotation(particles[p].get_pose().getRotation());
            double roll, pitch, yaw;
            rotation.getRPY(roll, pitch, yaw);

            sum_sin_yaw += std::sin(yaw) * particles[p].get_weight();
            sum_cos_yaw += std::cos(yaw) * particles[p].get_weight();
        }

        double mean_yaw = std::atan2(sum_sin_yaw, sum_cos_yaw);

        tf::Matrix3x3 mean_rotation;
        mean_rotation.setRPY(0.0, 0.0, mean_yaw);

        tf::Transform mean_pose(mean_rotation, mean_translation);
        return mean_pose;
    }


    /// Scatter all particles around the previously given start pose according
    /// to the given variance values.
    virtual void init(std::vector<Particle>& particles)
    {
        // Get the yaw angle of the start pose.
        tf::Matrix3x3 rotation(start_pose_.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);
        roll = pitch = 0.0;

        // Create the Gaussian number generator for the yaw angle.
        GaussNumberGenerator yaw_generator(yaw, var_yaw_);

        // Create the vector generator.
        VectorPolarGenerator vector_generator(0.0, var_xy_, 0.0, 2.0*M_PI, 1.0);

        // Sample the start poses.
        for (int p = 0; p < particles.size(); p++)
        {
            rotation.setRPY(roll, pitch, yaw_generator());
            tf::Vector3 translation(start_pose_.getOrigin() + vector_generator());
            translation.setZ(0.0);
            particles[p].set_pose(tf::Transform(rotation, translation));
        }
    }


    /// Sample a robot pose based on the last robot pose, the last movement
    /// and the previously specified motion uncertainty parameters.
    /// \param[in] last_pose odometry reading before the movement.
    /// \param[in] movement robot movement w.r.t. the robot frame.
    virtual tf::Transform sample_pose(const tf::Transform& last_pose, tf::Transform movement)
    {
        // Transform the movement from the robot frame into the map frame.
        movement.setOrigin((last_pose * movement).getOrigin() - last_pose.getOrigin());

        // Compute the last pose's Euler angles.
        tfScalar last_roll, last_pitch, last_yaw;
        tf::Matrix3x3 last_rotation(last_pose.getRotation());
        last_rotation.getRPY(last_roll, last_pitch, last_yaw);

        // Compute the Euler angle increments.
        tfScalar d_roll, d_pitch, d_yaw;
        tf::Matrix3x3 d_rotation(movement.getRotation());
        d_rotation.getRPY(d_roll, d_pitch, d_yaw);

        // Compute the translation.
        double d_x          = movement.getOrigin().x();
        double d_y          = movement.getOrigin().y();
        double d_z          = movement.getOrigin().z();

        // Decompose the movement into atomic movements according to the
        // motion model.
        double rot1         = std::atan2(d_y, d_x) - last_yaw;
        double trans        = std::sqrt(d_x*d_x + d_y*d_y);
        double rot2         = d_yaw - rot1;

        // Calculate the variance of the atomic movements.
        double var_rot1     = alpha_[0]*std::abs(rot1) + alpha_[1]*trans;
        double var_trans    = alpha_[2]*trans + alpha_[3]*(std::abs(rot1)+std::abs(rot2));
        double var_rot2     = alpha_[0]*std::abs(rot2) + alpha_[1]*trans;

        // Calculate the noise.
        double rot1_noisy   = rot1;
        double trans_noisy  = trans;
        double rot2_noisy   = rot2;

        // Sample from Gaussian distributions.
        // Avoid zero variance.
        if (var_rot1 > 0.0)
        {
            GaussNumberGenerator rot1_generator(0.0, var_rot1);
            rot1_noisy -= rot1_generator();
        }
        if (var_trans > 0.0)
        {
            GaussNumberGenerator trans_generator(0.0, var_trans);
            trans_noisy -= trans_generator();
        }
        if (var_rot2 > 0.0)
        {
            GaussNumberGenerator rot2_generator(0.0, var_rot2);
            rot2_noisy -= rot2_generator();
        }

        // Compose the robot pose after the noisy movement.
        tf::Vector3 position(
            last_pose.getOrigin().x() + trans_noisy * std::cos(last_yaw+rot1_noisy),
            last_pose.getOrigin().y() + trans_noisy * std::sin(last_yaw+rot1_noisy),
            0.0);

        tf::Matrix3x3 orientation;
        orientation.setRPY(0.0, 0.0, last_yaw + rot1_noisy + rot2_noisy);

        tf::Transform new_pose(orientation, position);
        return new_pose;
    }
};


#endif
