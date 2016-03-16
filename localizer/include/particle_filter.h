#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include <vector>

#include <boost/shared_ptr.hpp>

#include "particle.h"
#include "motion_model.h"
#include "sensor_model.h"


template<typename MotionModelT, typename SensorModelT>
class ParticleFilter
{
protected:
    boost::shared_ptr<MotionModelT> motion_model_;
    boost::shared_ptr<SensorModelT> sensor_model_;
    std::vector<Particle> particles_;


public:
    void set_motion_model(boost::shared_ptr<MotionModelT> motion_model)
    {
        motion_model_ = motion_model;
    }


    boost::shared_ptr<MotionModelT> get_motion_model()
    {
        return motion_model_;
    }


    void set_sensor_model(boost::shared_ptr<SensorModelT> sensor_model)
    {
        sensor_model_ = sensor_model;
    }


    boost::shared_ptr<SensorModelT> get_sensor_model()
    {
        return sensor_model_;
    }


    void init(unsigned int n_particles)
    {
        particles_.resize(n_particles, Particle());
        motion_model_->init(particles_);
    }


    void set_n_particles(unsigned int n_particles)
    {
        particles_.resize(n_particles, Particle(get_mean()));
    }


    bool is_initialized()
    {
        return particles_.size() > 0;
    }


    void update_motion(const tf::Transform& movement)
    {
        if (is_initialized())
            motion_model_->move_particles(movement, particles_);
    }


    void integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        if (is_initialized())
            sensor_model_->compute_particle_weights(measurement, particles_);
    }


    std::vector<Particle> get_particles() const
    {
        return particles_;
    }


    tf::Transform get_mean()
    {
        tf::Vector3 mean_vector;
        mean_vector.setZero();

        if (is_initialized())
        {
            normalize_particle_weights();

            /// \todo consider the weights.
            for (int p = 0; p < particles_.size(); p++)
                mean_vector += particles_[p].get_pose().getOrigin();

            mean_vector /= particles_.size();
        }

        tf::Transform mean_pose(tf::Transform::getIdentity());
        mean_pose.setOrigin(mean_vector);

        return mean_pose;
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
