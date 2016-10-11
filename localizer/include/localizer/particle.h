#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

// ROS coordinate transformations.
#include <tf/tf.h>


/// Particle struct for use in robot localization.
/// Stores the information relevant for a particle used in robot localization: 
/// pose and localization error.
struct Particle
{
    /// Robot pose.
    tf::Transform pose;

    /// Localization error.
    double error;


    /// Constructor.
    /// Initializes the particle to the given pose and error.
    Particle(const tf::Transform& pose = tf::Transform::getIdentity(), double error = 0.0)
        : pose(pose),
          error(error)
    {
    }
};


#endif
