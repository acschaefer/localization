#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

/// Standard template library.
#include <vector>

// Particles used by the particle filter.
#include "particle.h"


/// Motion model for use with the ParticleFilter class for robot localization.
class MotionModel
{
protected:
    /// Initial robot position.
    tf::Transform start_pose_;


public:
    /// Default constructor.
    MotionModel()
        : start_pose_(tf::Transform::getIdentity())
    {
    }


    /// Define where to initialize the robot.
    void set_start_pose(const tf::Transform& start_pose)
    {
        start_pose_ = start_pose;
    }


    /// Initializes the particle filter with the given number of particles.
    virtual void init(std::vector<Particle>& particles)
    {
        for (int p = 0; p < particles.size(); p++)
            particles[p].set_pose(start_pose_);
    }


    /// Apply noisy motion to all particles.
    void move_particles(const tf::Transform& movement,
                                std::vector<Particle>& particles)
    {
        for (int p = 0; p < particles.size(); p++)
        {
            tf::Transform new_pose = sample_pose(particles[p].get_pose(), movement);
            particles[p].set_pose(new_pose);
        }
    }


protected:
    /// Sample the new pose after noisy motion.
    virtual tf::Transform sample_pose(const tf::Transform& last_pose,
                                      const tf::Transform& movement) = 0;
};


#endif
