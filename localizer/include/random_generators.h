#ifndef RANDOM_VECTOR_GENERATOR_H_
#define RANDOM_VECTOR_GENERATOR_H_

#include <vector>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>

#include <tf/tf.h>


class NumberGenerator
{
protected:
    static int seed_;


protected:
    NumberGenerator()
    {
        seed_++;
    }
};


int NumberGenerator::seed_ = 0;


class GaussNumberGenerator : NumberGenerator
{
protected:
    boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> > generator_;


public:
    GaussNumberGenerator(double mean = 0.0, double var = 1.0)
     : generator_(boost::mt19937(seed_), boost::normal_distribution<double>(mean, var))
    {
    }


    double operator()()
    {
        return generator_();
    }
};


class UniformNumberGenerator : NumberGenerator
{
protected:
    boost::random::variate_generator<boost::mt19937, boost::uniform_real<double> > generator_;


public:
    UniformNumberGenerator(double min = 0.0, double max = 1.0)
        : generator_(boost::mt19937(seed_), boost::uniform_real<double>(min, max))
    {
    }


    double operator()()
    {
        return generator_();
    }
};


class GaussVectorGenerator
{
protected:
    std::vector<GaussNumberGenerator> number_generators_;


public:
    GaussVectorGenerator(const tf::Vector3& mean, const tf::Vector3& var)
    {
        for (int row = 0; row < 3; row++)
            number_generators_.push_back(GaussNumberGenerator(mean[row], var[row]));
    }


    tf::Vector3 operator()()
    {
        tf::Vector3 vector;

        for (int row = 0; row < 3; row++)
            vector[row] = number_generators_[row]();

        return vector;
    }
};


class UniformVectorGenerator
{
protected:
    std::vector<UniformNumberGenerator> number_generators_;


public:
    UniformVectorGenerator(const tf::Vector3& min, const tf::Vector3& max)
    {
        for (int row = 0; row < 3; row++)
            number_generators_.push_back(UniformNumberGenerator(min[row], max[row]));
    }


    tf::Vector3 operator()()
    {
        tf::Vector3 vector;

        for (int row = 0; row < 3; row++)
            vector[row] = number_generators_[row]();

        return vector;
    }
};


class VectorPolarGenerator
{
protected:
    GaussNumberGenerator radius_generator_;
    GaussNumberGenerator z_generator_;
    UniformNumberGenerator angle_generator_;


public:
    VectorPolarGenerator(double mean_radius, double var_radius,
                         double min_angle, double max_angle,
                         double var_z)
        : radius_generator_(mean_radius, var_radius),
          angle_generator_(min_angle, max_angle),
          z_generator_(0.0, var_z)
    {
    }


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
