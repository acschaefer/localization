#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

// ROS coordinate transformations.
#include <tf/tf.h>


/// Particle class for use with the ParticleFilter class for robot localization.
/// Stores all the information relevant for a particle of particle filter
/// used for robot localization: pose and weight.
class Particle
{
protected:
    /// Robot pose.
    tf::Transform pose_;

    /// Weight of the particle.
    double weight_;


public:
    /// Constructor.
    /// Initializes the particle to the given pose and weight.
    Particle(const tf::Transform& pose = tf::Transform::getIdentity(), double weight = 1.0)
    {
        set_pose(pose);
        set_weight(weight);
    }


    /// Sets the particle's pose.
    void set_pose(const tf::Transform& pose = tf::Transform::getIdentity())
    {
        pose_ = pose;
    }


    /// Returns the particle's pose.
    tf::Transform get_pose() const
    {
        return pose_;
    }


    /// Sets the particles weight.
    /// \param[in] weight particle weight. Negative weights are set to zero.
    void set_weight(double weight)
    {
        weight_ = std::max(std::numeric_limits<double>::epsilon(), weight);
    }


    /// Returns the particle's weight.
    double get_weight() const
    {
        return weight_;
    }
};


#endif
