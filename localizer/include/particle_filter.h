#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include <vector>

#include <boost/shared_ptr.hpp>

#include "particle.h"
#include "motion_model.h"


class ParticleFilter
{
protected:
    boost::shared_ptr<MotionModel> motion_model_;
    std::vector<Particle> particles_;


public:
    void set_motion_model(boost::shared_ptr<MotionModel> motion_model)
    {
        motion_model_ = motion_model;
    }


    void init(unsigned int n_particles, const tf::Transform& start_pose = tf::Transform::getIdentity())
    {
        particles_.resize(n_particles, Particle());
        motion_model_->init(start_pose, particles_);
    }


    bool is_initialized()
    {
        return particles_.size() > 0;
    }


    void update_motion(const tf::Transform& movement)
    {
        motion_model_->update(movement, particles_);
    }


    std::vector<Particle> get_particles() const
    {
        return particles_;
    }


    tf::Vector3 get_mean()
    {
        normalize_particle_weights();

        tf::Vector3 mean;

        /// \todo consider the weights.
        for (int p = 0; p < particles_.size(); p++)
            mean += particles_[p].get_pose().getOrigin();

        mean /= particles_.size();

        return mean;
    }


protected:
    void normalize_particle_weights()
    {
        double cum_weight = 0.0;
        for (int p = 0; p < particles_.size(); p++)
            cum_weight += particles_[p].get_weight();

        for (int p = 0; p < particles_.size(); p++)
            particles_[p].set_weight(particles_[p].get_weight() / cum_weight);
    }
};


#endif
