#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

// Standard template libraries.
#include <vector>

// Boost.
#include <boost/shared_ptr.hpp>

// Particles.
#include "particle.h"

// Motion model base class.
#include "motion_model.h"

// Sensor model base class.
#include "sensor_model.h"

// Random number generators.
#include "random_generators.h"


/// Partile filter for robot localization.
template<typename MotionModelT, typename SensorModelT>
class ParticleFilter
{
protected:
    /// Particles.
    std::vector<Particle> particles_;

    /// Motion model used for propagating the particles
    /// in the motion update step.
    boost::shared_ptr<MotionModelT> motion_model_;

    /// Sensor model used for weighting the particles
    /// in the sensor integration step.
    boost::shared_ptr<SensorModelT> sensor_model_;

    /// Indicates whether the particle filter has been initialized.
    bool initialized_;


public:
    /// Default constructor.
    ParticleFilter()
        : initialized_(false)
    {
    }


    /// Sets the motion model.
    void set_motion_model(boost::shared_ptr<MotionModelT> motion_model)
    {
        motion_model_ = motion_model;
    }


    /// Returns the motion model.
    boost::shared_ptr<MotionModelT> get_motion_model()
    {
        return motion_model_;
    }


    /// Sets the sensor model.
    void set_sensor_model(boost::shared_ptr<SensorModelT> sensor_model)
    {
        sensor_model_ = sensor_model;
    }


    /// Returns the motion model.
    boost::shared_ptr<SensorModelT> get_sensor_model()
    {
        return sensor_model_;
    }


    /// Initializes the particle filter.
    /// Scatters the particles around the given start pose.
    void init()
    {
        motion_model_->init(particles_);
        initialized_ = true;
    }


    /// Adapts the number of particles in use.
    void set_n_particles(unsigned int n_particles)
    {
        if (n_particles == particles_.size())
            return;

        /// \todo Delete the particles with the lowest weights.
        /// \todo Add particles scattered around the current mean.
        particles_.resize(n_particles, Particle(get_mean()));
        normalize_particle_weights();
    }


    /// Returns whether or not the filter has been initialized.
    bool is_initialized()
    {
        return initialized_;
    }


    /// Propagates the particles according to the given movement.
    /// Applies noisy motion as specified in the motion model.
    void update_motion(const tf::Transform& movement)
    {
        if (is_initialized())
            motion_model_->move_particles(movement, particles_);
    }


    /// Computes the weights for all particles according
    /// to the given sensor input.
    void integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        if (is_initialized())
        {
            sensor_model_->compute_particle_weights(measurement, particles_);
            resample();
        }
    }


    /// Resample the particles according to their weights.
    /// This algorithm is the low variance sampler taken from the book
    /// "Probabilistic Robotics" by Thrun et al.
    void resample()
    {
        // This algorithm assumes the particle weights are normalized.
        normalize_particle_weights();

        if (particles_.size() < 1)
            return;

        std::vector<Particle> resampled_particles;

        int M = particles_.size();

        UniformNumberGenerator generator(0.0, 1.0/M);
        double r = generator();

        double c = particles_[0].weight;

        int i = 0;

        for (int m = 1; m <= M; m++)
        {
            double U = r + (m-1)*(1.0/M);

            while (U > c)
            {
                i += 1;
                c += particles_[i].weight;
            }

            resampled_particles.push_back(particles_[i]);
        }

        particles_ = resampled_particles;
    }


    /// Returns the particle vector.
    std::vector<Particle> get_particles() const
    {
        return particles_;
    }


    /// Computes the weighted mean position of all particles.
    tf::Transform get_mean()
    {
        normalize_particle_weights();
        return motion_model_->get_mean(particles_);
    }


protected:
    /// Normalizes the particle weights so they sum up to 1.
    void normalize_particle_weights()
    {
        // Shift particle weights so the minimum weigt is 0.
        double min_weight = 0.0;
        for (int p = 0; p < particles_.size(); p++)
            min_weight = std::min(particles_[p].weight, min_weight);

        for (int p = 0; p < particles_.size(); p++)
            particles_[p].weight += -min_weight;

        // Sum up all weights.
        double total_weight = 0.0;
        for (int p = 0; p < particles_.size(); p++)
            total_weight += particles_[p].weight;

        // Divide the weight of each particle by the total weight.
        for (int p = 0; p < particles_.size(); p++)
            particles_[p].weight = particles_[p].weight / total_weight;
    }
};


#endif
