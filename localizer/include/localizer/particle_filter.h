#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

// Standard template libraries.
#include <vector>

// Boost.
#include <boost/shared_ptr.hpp>

// Particles.
#include "localizer/particle.h"

// Motion model base class.
#include "localizer/motion_model.h"

// Sensor model base class.
#include "localizer/sensor_model.h"

// Random number generators.
#include "localizer/random_generators.h"


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
    /// to the given sensor input and resamples.
    void integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        if (is_initialized())
        {
            sensor_model_->compute_particle_weights(measurement, particles_);
            resample();
        }
    }


    /// Computes the weights of all particles from multiple sensor readings and resamples them.
    void integrate_measurements(const std::vector<typename SensorModelT::Measurement>& measurements)
    {
        if (is_initialized())
        {
            for (size_t i = 0; i < measurements.size(); ++i)
                sensor_model_->compute_particle_weights(measurements[i], particles_);

            resample();
        }
    }


    /// Resample the particles according to their weights.
    /// This algorithm is the low variance sampler taken from the book
    /// "Probabilistic Robotics" by Thrun et al., MIT Press, 2005.
    void resample()
    {
        if (!is_initialized())
            return;

        if (particles_.size() < 1)
            return;

        // The low variance sampling algorithm assumes the particle weights are normalized,
        // so normalize them.
        normalize_particle_weights();

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

        // Set the particle weights to a uniform distribution.
        for (int p = 0; p < particles_.size(); p++)
            particles_[p].weight = 1.0 / M;
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


    /// Estimates the number of effective particles.
    int get_neff()
    {
        double sqw = 0.0;
        for (size_t i = 0; i < particles_.size(); ++i)
            sqw += std::pow(particles_[i].weight, 2.0);

        return 1.0/sqw;
    }


    /// Print the Cartesian coordinates and the weights of all particles.
    void print()
    {
        std::cout << particles_.size() << " particles:" << std::endl;
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            tf::Vector3& v = particles_[i].pose.getOrigin();
            std::cout << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]: " << particles_[i].weight;
            std::cout << std::endl;
        }
    }


protected:
    /// Normalizes the particle weights so they sum up to 1.
    /// \pre All weights are zero or positive.
    void normalize_particle_weights()
    {
        // Sum up all weights.
        double total_weight = 0.0;
        for (size_t p = 0; p < particles_.size(); ++p)
            total_weight += particles_[p].weight;

        // Divide the weight of each particle by the total weight.
        for (size_t p = 0; p < particles_.size(); ++p)
        {
            if (total_weight == 0.0)
                particles_[p].weight = 1.0 / particles_.size();
            else
                particles_[p].weight /= total_weight;
        }
    }
};


#endif
