#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

// Standard libraries.
#include <vector>

// Boost.
#include <boost/shared_ptr.hpp>

// Particles, motion model, and sensor model.
#include "localizer/particle.h"
#include "localizer/motion_model.h"
#include "localizer/sensor_model.h"

// Random number generators.
#include "localizer/random_generators.h"

// Geometry statistics.
#include "geo_statistics.h"


/// Partile filter for robot localization.
template<typename MotionModelT, typename SensorModelT>
class ParticleFilter
{
protected:
    /// Particles.
    std::vector<Particle> particles_;

    /// Motion model used for propagating the particles in the motion update step.
    boost::shared_ptr<MotionModelT> motion_model_;

    /// Sensor model used for weighting the particles in the sensor integration step.
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


    /// Sets the motion model.
    void set_motion_model(const MotionModelT& motion_model)
    {
        motion_model_ = boost::make_shared<MotionModelT>(motion_model);
    }


    /// Returns the motion model.
    boost::shared_ptr<MotionModelT> get_motion_model() const
    {
        return motion_model_;
    }


    /// Sets the sensor model.
    void set_sensor_model(boost::shared_ptr<SensorModelT> sensor_model)
    {
        sensor_model_ = sensor_model;
    }


    /// Sets the sensor model.
    void set_sensor_model(const SensorModelT& sensor_model)
    {
        sensor_model_ = boost::shared_ptr<SensorModelT>(sensor_model);
    }


    /// Returns the sensor model.
    boost::shared_ptr<SensorModelT> get_sensor_model() const
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
        /// \todo Add particles scattered around the current maximum.
        if (particles_.empty())
            particles_.resize(n_particles);
        else
            particles_.resize(n_particles, Particle(get_max()));
    }


    /// Returns whether or not the filter has been initialized.
    bool is_initialized() const
    {
        return initialized_;
    }


    /// Propagates the particles according to the given movement.
    /// Applies noisy motion as specified by the motion model.
    void update_motion(const tf::Transform& movement)
    {
        if (is_initialized())
            motion_model_->move_particles(movement, particles_);
    }


    /// Computes the localization errors for all particles according to the given sensor input.
    void integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        if (is_initialized())
            sensor_model_->compute_particle_errors(measurement, particles_);
    }


    /// Computes the localization errors of all particles from multiple sensor readings and resamples them.
    void integrate_measurements(const std::vector<typename SensorModelT::Measurement>& measurements)
    {
        if (is_initialized())
            for (size_t i = 0u; i < measurements.size(); ++i)
                sensor_model_->compute_particle_errors(measurements[i], particles_);
    }


    /// Resample the particles according to their weights.
    /// This algorithm is the low variance sampler taken from the book "Probabilistic Robotics"
    /// by Thrun et al., MIT Press, 2005.
    void resample()
    {
        if (!is_initialized() || particles_.empty())
            return;

        // The low variance sampling algorithm assumes the particle weights are normalized.
        std::vector<double> weights = get_weights();

        // Create the vector that will be filled with the resampled particles.
        std::vector<Particle> resampled_particles;

        // Execute the algorithm. The variable names are chosen according to the variables in the book.
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
        for (size_t i = 0u; i < particles_.size(); ++i)
            particles_[i].error = 0.0;
    }


    /// Returns the particle vector.
    std::vector<Particle> get_particles() const
    {
        return particles_;
    }


    /// Computes the weighted mean position of all particles and combines it with the orientation of the particle
    /// with the highest weight.
    tf::Transform get_mean() const
    {
        if (!is_initialized())
            tf::Transform::getIdentity();

        // Compute the weighted average of the poses of the particles.
        std::vector<tf::Transform> tf;
        for (size_t i = 0u; i < particles_.size(); ++i)
            tf.push_back(particles_[i].pose);

        return tfmean(tf, get_weights());
    }


    /// Determines the pose of the particle with the highest weight.
    tf::Transform get_max() const
    {
        if (!is_initialized())
            tf::Transform::getIdentity();

        // Compute the particle weights.
        std::vector<double> weights = get_weights();

        // Return the pose of the most likely particle.
        return particles_[std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()))].pose;
    }


    /// Estimates the number of effective particles.
    double get_neff() const
    {
        // Compute the particle weights.
        std::vector<double> weights = get_weights();

        // Compute the sum of the squared weights of all particles.
        double wsq = 0.0;
        for (size_t i = 0u; i < weights.size(); ++i)
        {
            if (!std::isnan(weights[i]))
                wsq += std::pow(weights[i], 2.0);
        }

        return 1.0 / wsq;
    }


    /// Print the Cartesian coordinates, the errors and weights of all particles.
    void print() const
    {
        std::vector<double> weights = get_weights();

        std::cout << particles_.size() << " particles:" << std::endl;
        for (size_t i = 0u; i < particles_.size(); ++i)
        {
            tf::Vector3& v = particles_[i].pose.getOrigin();
            std::cout << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]: "
                      << "error " << particles_[i].error << ", weight " << weights[i] << std::endl;
        }
    }


protected:
    /// Computes the weight of all particles.
    std::vector<double> get_weights() const
    {
        // Find the maximum particle error.
        double max_error = 0.0;
        for (size_t i = 0u; i < particles_.size(); ++i)
            max_error = std::max(max_error, particles_[i].error);

        // Convert the error values to weights and sum them up.
        double total_weight = 0.0;
        std::vector<double> weights(particles_.size(), 0.0);
        for (size_t i = 0u; i < particles_.size(); ++i)
        {
            if (!std::isnan(particles_[i].error))
                weights[i] = max_error - particles_[i].error;

            total_weight += weights[i];
        }

        // Normalize the weights.
        for (size_t i = 0u; i < weights.size(); ++i)
        {
            if (total_weight == 0.0)
                weights[i] = 1.0 / weights.size();
            else
                weights[i] /= total_weight;
        }

        return weights;
    }
};


#endif
