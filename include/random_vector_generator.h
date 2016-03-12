#ifndef RANDOM_VECTOR_GENERATOR_H_
#define RANDOM_VECTOR_GENERATOR_H_

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>


template<int Dim>
class RandomVectorGenerator
{
protected:
    typedef Eigen::Matrix<double, Dim, 1>
            Vector;
    typedef boost::normal_distribution<double>
            NormalDistribution;
    typedef boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> >
            RandomNumberGenerator;


protected:
    std::vector<RandomNumberGenerator> random_number_generators_;


public:
    RandomVectorGenerator(const Vector& mean, const Vector& var) 
    {    
        for (int row = 0; row < Dim; row++)
        {
            NormalDistribution normal_distribution(mean[row], var[row]);
            RandomNumberGenerator random_number_generator(boost::mt19937(time(NULL)), normal_distribution);
            random_number_generators_.push_back(random_number_generator);
        }
    }


    virtual Vector generate_vector()
    {
        Vector vector;

        for (int row = 0; row < Dim; row++)
            vector[row] = random_number_generators_[row]();

        return vector;
    }
};


class RandomAngleAxisGenerator : protected RandomVectorGenerator<3>
{
protected:
    RandomNumberGenerator random_number_generator_angle_;


public:
    RandomAngleAxisGenerator(double angle_mean, double angle_var,
                             const Eigen::Vector3d& axis_mean, const Eigen::Vector3d& axis_var)
     : RandomVectorGenerator<3>(axis_mean, axis_var), 
       random_number_generator_angle_(boost::mt19937(time(NULL)), NormalDistribution(angle_mean, angle_var))
    {
    }	


    Eigen::AngleAxisd generate_angle_axis()
    {
	return Eigen::AngleAxisd(random_number_generator_angle_(), generate_vector().normalized());
    }
};


#endif
