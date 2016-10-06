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
            sensor_model_->compute_particle_errors(measurement, particles_);
    }


    /// Computes the weights of all particles from multiple sensor readings and resamples them.
    void integrate_measurements(const std::vector<typename SensorModelT::Measurement>& measurements)
    {
        if (is_initialized())
            for (size_t i = 0; i < measurements.size(); ++i)
                sensor_model_->compute_particle_errors(measurements[i], particles_);
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
        std::vector<double> weights = get_particle_weights();

        std::vector<Particle> resampled_particles;

        int M = particles_.size();

        UniformNumberGenerator generator(0.0, 1.0/M);
        double r = generator();

        double c = weights[0];

        int i = 0;

        for (int m = 1; m <= M; m++)
        {
            double U = r + (m-1)*(1.0/M);

            while (U > c)
            {
                i += 1;
                c += weights[i];
            }

            resampled_particles.push_back(particles_[i]);
        }

        particles_ = resampled_particles;

        // Reset the particle errors.
        for (int p = 0; p < particles_.size(); p++)
            particles_[p].error = 0.0;
    }


    /// Returns the particle vector.
    std::vector<Particle> get_particles() const
    {
        return particles_;
    }


    /// Computes the weighted mean position of all particles.
    tf::Transform get_mean()
    {
        return motion_model_->get_mean(particles_);
    }


    /// Estimates the number of effective particles.
    double get_neff()
    {
        std::vector<double> weights = get_particle_weights();
        double wsq = 0.0;
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            if (!std::isnan(weights[i]))
                wsq += std::pow(weights[i], 2.0);
        }

        return 1.0 / wsq;
    }


    /// Print the Cartesian coordinates, the errors and weights of all particles.
    void print()
    {
        std::vector<double> weights = get_particle_weights();

        std::cout << particles_.size() << " particles:" << std::endl;
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            tf::Vector3& v = particles_[i].pose.getOrigin();
            std::cout << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]: "
                      << particles_[i].error << ", " << weights[i] << std::endl;
        }
    }


protected:
    /// Computes the weight of all particles.
    std::vector<double> get_particle_weights() const
    {
        // Find the maximum particle error.
        double max_error = 0.0;
        for (size_t i = 0u; i < particles_.size(); ++i)
            max_error = std::max(max_error, particles_[i].error);

        // Convert the error values to weights.
        std::vector<double> weights(particles_.size());
        for (size_t i = 0u; i < particles_.size(); ++i)
            weights[i] = max_error - particles_[i].error;

        // Set all NaN particles to the minimum weight and sum up all weights.
        double total_weight = 0.0;
        for (size_t i = 0u; i < weights.size(); ++i)
        {
            if (std::isnan(weights[i]))
                weights[i] = 0.0;

            total_weight += weights[i];
        }

        // Normalize the weights
        for (size_t i = 0u; i < weights.size(); ++i)
        {
            if (total_weight == 0.0)
                weights[i] = 1.0 / weights.size();
            else
                weights[i] /= total_weight;
        }
    }
};


#endif
