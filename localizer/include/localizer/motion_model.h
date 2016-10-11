#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

/// Standard libraries.
#include <vector>

// Particles used by the particle filter.
#include "localizer/particle.h"


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


    /// Defines where to initialize the robot.
    virtual void set_start_pose(tf::Transform start_pose)
    {
        start_pose_ = start_pose;
    }


    /// Returns the start pose.
    virtual tf::Transform get_start_pose() const
    {
        return start_pose_;
    }


    /// Initializes the particle filter with the given number of particles.
    virtual void init(std::vector<Particle>& particles)
    {
        for (size_t i = 0u; i < particles.size(); ++i)
            particles[i].pose = start_pose_;
    }


    /// Applies noisy motion to all particles.
    virtual void move_particles(const tf::Transform& movement, std::vector<Particle>& particles) = 0;
};


#endif
