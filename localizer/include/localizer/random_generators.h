#ifndef RANDOM_VECTOR_GENERATOR_H_
#define RANDOM_VECTOR_GENERATOR_H_

// Standard template library.
#include <vector>

// Boost.
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>

// ROS coordinate transformations.
#include <tf/tf.h>


/// Base class for all number generators.
/// Holds a static member used to seed all derived generator instances.
/// In this way, the generated random numbers in a program -- even with
/// many random number generators -- are reproducible.
class NumberGenerator
{
protected:
    /// Seed value for random number generation.
    static int seed_;


protected:
    /// Constructor.
    /// Increases the seed value every time it is called.
    NumberGenerator()
    {
        seed_++;
    }
};


int NumberGenerator::seed_ = 0;


/// Generates random numbers sampled from a Gaussian distribution.
class GaussNumberGenerator : NumberGenerator
{
protected:
    /// Random number generator engine.
    boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> > generator_;


public:
    /// Constructor.
    /// Stores the mean and the variance of the
    /// normal distribution to sample from.
    GaussNumberGenerator(double mean = 0.0, double var = 1.0)
     : generator_(boost::mt19937(seed_),
                  boost::normal_distribution<double>(mean, std::max(std::numeric_limits<double>::epsilon(), var)))
    {
    }


    /// Returns a random number sampled from a Gaussian probability distribution.
    double operator()()
    {
        return generator_();
    }
};


/// Generates random numbers that are uniformly distributed within the specified interval.
class UniformNumberGenerator : NumberGenerator
{
protected:
    /// Random number generator engine.
    boost::random::variate_generator<boost::mt19937, boost::uniform_real<double> > generator_;


public:
    /// Constructor.
    /// Stores the interval the random numbers are sampled from.
    UniformNumberGenerator(double min = 0.0, double max = 1.0)
        : generator_(boost::mt19937(seed_),
                     boost::uniform_real<double>(min, std::max(min, max)))
    {
    }


    /// Returns a random number from the given interval.
    double operator()()
    {
        return generator_();
    }
};


/// Generates 3D vectors whose coordinates are sampled from Gaussian distributions.
class GaussVectorGenerator
{
protected:
    /// Gaussian random number generators for all dimensions.
    std::vector<GaussNumberGenerator> number_generators_;


public:
    /// Constructor.
    /// Stores the mean and the variance for all dimensions of the vector.
    GaussVectorGenerator(const tf::Vector3& mean, const tf::Vector3& var)
    {
        for (int row = 0; row < 3; row++)
            number_generators_.push_back(GaussNumberGenerator(mean[row], var[row]));
    }


    /// Returns a random vector.
    tf::Vector3 operator()()
    {
        tf::Vector3 vector;

        for (int row = 0; row < 3; row++)
            vector[row] = number_generators_[row]();

        return vector;
    }
};


/// Generates random vectors whose coordinates are uniformly distributed.
class UniformVectorGenerator
{
protected:
    /// Uniform random number generators for all dimensions.
    std::vector<UniformNumberGenerator> number_generators_;


public:
    /// Constructor.
    /// Stores the intervals for all dimensions of the vector.
    UniformVectorGenerator(const tf::Vector3& min, const tf::Vector3& max)
    {
        for (int row = 0; row < 3; row++)
            number_generators_.push_back(UniformNumberGenerator(min[row], max[row]));
    }


    /// Returns a random vector.
    tf::Vector3 operator()()
    {
        tf::Vector3 vector;

        for (int row = 0; row < 3; row++)
            vector[row] = number_generators_[row]();

        return vector;
    }
};


/// Generates random 3D vectors.
/// The vector is generated in the following way:
/// -# Sample an azimuth angle uniformly distributed in [0; 2*PI).
/// -# Sample a length from a Gaussian distribution with mean \c mean_radius
///    and variance \c var_radius.
/// -# Sample a z-coordinate from a Gaussian distribution with mean 0 and
///    variance \c var_z.
/// -# Construct the 3D vector from the sampled azimuth angle in the x-y plane,
///    the radius in the x-y plane, and the z coordinate.
class VectorPolarGenerator
{
protected:
    /// Random number generator for the vector length projected onto the x-y plane.
    GaussNumberGenerator radius_generator_;

    /// Random number generator for the vector's z-coordinate.
    GaussNumberGenerator z_generator_;

    /// Random number generator for the vector's azimuth angle in the x-y plane.
    UniformNumberGenerator angle_generator_;


public:
    /// Constructor.
    VectorPolarGenerator(double mean_radius, double var_radius,
                         double min_angle, double max_angle,
                         double var_z)
        : radius_generator_(mean_radius, var_radius),
          angle_generator_(min_angle, max_angle),
          z_generator_(0.0, var_z)
    {
    }


    /// Returns a random vector.
    tf::Vector3 operator()()
    {
        double radius   = radius_generator_();
        double angle    = angle_generator_();

        double x        = radius * std::cos(angle);
        double y        = radius * std::sin(angle);
        double z        = z_generator_();

        return tf::Vector3(x, y, z);
    }
};


#endif
