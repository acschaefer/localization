#ifndef RANDOM_VECTOR_GENERATOR_H_
#define RANDOM_VECTOR_GENERATOR_H_

#include <vector>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>

#include <tf/tf.h>


class GaussNumberGenerator
{
protected:
    boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> > generator_;


public:
    GaussNumberGenerator(double mean = 0.0, double var = 1.0)
     : generator_(boost::mt19937(), boost::normal_distribution<double>(mean, var))
    {
        generator_.engine().seed();
    }


    double operator()()
    {
        return generator_();
    }
};


class UniformNumberGenerator
{
protected:
    boost::random::variate_generator<boost::mt19937, boost::uniform_real<double> > generator_;


public:
    UniformNumberGenerator(double min = 0.0, double max = 1.0)
        : generator_(boost::mt19937(), boost::uniform_real<double>(min, max))
    {
        generator_.engine().seed();
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

#endif
