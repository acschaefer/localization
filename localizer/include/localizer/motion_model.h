#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

/// Standard library.
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
    virtual tf::Transform get_start_pose()
    {
        return start_pose_;
    }


    /// Initializes the particle filter with the given number of particles.
    virtual void init(std::vector<Particle>& particles)
    {
        for (int p = 0; p < particles.size(); p++)
            particles[p].pose = start_pose_;
    }


    /// Applies noisy motion to all particles.
    virtual void move_particles(const tf::Transform& movement,
                                std::vector<Particle>& particles) = 0;


    /// Calculates the weighted mean pose of all particles.
    /// \return mean pose of all particles.
    /// \pre The particle weights are normalized.
    /// \note The returned mean pose's rotation is always identity.
    virtual tf::Transform get_mean(const std::vector<Particle>& particles)
    {
        tf::Vector3 mean_vector;
        mean_vector.setZero();

        for (int p = 0; p < particles.size(); p++)
            mean_vector += particles[p].pose.getOrigin() * particles[p].weight;

        tf::Transform mean_pose(tf::Transform::getIdentity());
        mean_pose.setOrigin(mean_vector);

        return mean_pose;
    }
};


#endif
