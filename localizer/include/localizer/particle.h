#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

// ROS coordinate transformations.
#include <tf/tf.h>


/// Particle struct for use with the ParticleFilter class for robot localization.
/// Stores all the information relevant for a particle of particle filter
/// used for robot localization: pose and weight.
struct Particle
{
    /// Robot pose.
    tf::Transform pose;

    /// Weight of the particle.
    double weight;


    /// Constructor.
    /// Initializes the particle to the given pose and weight.
    Particle(const tf::Transform& pose = tf::Transform::getIdentity(), double weight = 1.0)
        : pose(pose),
          weight(weight)
    {
    }
};


#endif
