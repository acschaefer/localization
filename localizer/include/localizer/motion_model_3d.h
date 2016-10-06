#ifndef MOTION_MODEL_3D_H_
#define MOTION_MODEL_3D_H_ MOTION_MODEL_3D_H_

// Base class.
#include "localizer/motion_model.h"

// Random number generators required for sampling.
#include "localizer/random_generators.h"


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

    /// Translation threshold.
    /// Translations less than this threshold are interpreted as odometry noise.
    /// This threshold is necessary as translation noise, especially with high
    /// x-y covariance, can induce large -- and virtual -- rotation values.
    double translation_threshold_;


public:
    /// Default constructor.
    /// Initializes members to default values.
    MotionModel3d()
        : alpha_(std::vector<double>(4, 1.0)),
          var_xy_(1.0), var_yaw_(0.1),
          translation_threshold_(1.0e-3)
    {
    }


    /// Sets the motion uncertainty parameters.
    void set_alpha(const std::vector<double>& alpha)
    {
        alpha_ = alpha;
    }


    /// Sets the robot start pose.
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


    /// Sets the start pose variances.
    void set_start_pose_variance(double var_xy, double var_yaw)
    {
        var_xy_     = var_xy;
        var_yaw_    = var_yaw;
    }


    /// Sets the translation threshold.
    void set_translation_threshold(double threshold)
    {
        translation_threshold_ = std::max(0.0, threshold);
    }


protected:
    /// Calculates the weighted mean pose of all particles.
    virtual tf::Transform get_mean(const std::vector<Particle>& particles)
    {
        /*
        tf::Vector3 mean_translation
            = MotionModel::get_mean(particles).getOrigin();

        double sum_sin_yaw, sum_cos_yaw;
        sum_sin_yaw = sum_cos_yaw = 0.0;
        for (size_t p = 0; p < particles.size(); p++)
        {
            tf::Matrix3x3 rotation(particles[p].pose.getRotation());
            double roll, pitch, yaw;
            rotation.getRPY(roll, pitch, yaw);

            sum_sin_yaw += std::sin(yaw) * particles[p].weight;
            sum_cos_yaw += std::cos(yaw) * particles[p].weight;
        }

        double mean_yaw = std::atan2(sum_sin_yaw, sum_cos_yaw);

        tf::Matrix3x3 mean_rotation;
        mean_rotation.setRPY(0.0, 0.0, mean_yaw);

        tf::Transform mean_pose(mean_rotation, mean_translation);
        return mean_pose;
        */
        return tf::Transform();
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
        for (size_t p = 0; p < particles.size(); p++)
        {
            rotation.setRPY(roll, pitch, yaw_generator());
            tf::Vector3 translation(start_pose_.getOrigin()+vector_generator());
            translation.setZ(0.0);
            particles[p].pose = tf::Transform(rotation, translation);
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
        // Compute the Euler angle increments.
        tfScalar d_roll, d_pitch, d_yaw;
        const tf::Matrix3x3 d_rotation(movement.getRotation());
        d_rotation.getRPY(d_roll, d_pitch, d_yaw);

        // Compute the translational increment.
        const double d_x = movement.getOrigin().x();
        const double d_y = movement.getOrigin().y();

        // Decompose the movement into atomic movements according to the
        // motion model.
        // If the translation does not exceed the threshold,
        // set the first rotation to 0.
        const double sign_d_x = (d_x >= 0.0 ? 1.0 : -1.0);
        const double trans = sign_d_x * std::sqrt(d_x*d_x + d_y*d_y);
        double rot1 = 0.0;
        if (std::abs(trans) >= translation_threshold_)
        {
            if (d_x == 0.0)
                rot1 = sign_d_x * 0.5 * M_PI;
            else
                rot1 = std::atan(d_y / d_x);
        }
        double rot2 = d_yaw - rot1;

        // Calculate the variance of the atomic movements.
        const double var_rot1     = alpha_[0]*std::abs(rot1)
                                    + alpha_[1]*std::abs(trans);
        const double var_trans    = alpha_[2]*std::abs(trans)
                                    + alpha_[3]*(std::abs(rot1)+std::abs(rot2));
        const double var_rot2     = alpha_[0]*std::abs(rot2)
                                    + alpha_[1]*std::abs(trans);

        // Add noise to the movement and move the particles.
        for (size_t p = 0; p < particles.size(); p++)
        {
            const tf::Transform& last_pose = particles[p].pose;

            // Compute the last pose's Euler angles.
            tfScalar last_roll, last_pitch, last_yaw;
            const tf::Matrix3x3 last_rotation(last_pose.getRotation());
            last_rotation.getRPY(last_roll, last_pitch, last_yaw);

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

            // Compute the robot pose w.r.t. the map after the noisy movement.
            particles[p].pose.setOrigin(tf::Vector3(
                last_pose.getOrigin().x()
                    + trans_noisy * std::cos(last_yaw+rot1_noisy),
                last_pose.getOrigin().y()
                    + trans_noisy * std::sin(last_yaw+rot1_noisy),
                last_pose.getOrigin().z()));

            tf::Matrix3x3 orientation;
            orientation.setRPY(0.0, 0.0, last_yaw + rot1_noisy + rot2_noisy);
            particles[p].pose.setBasis(orientation);
        }
    }
};


#endif
