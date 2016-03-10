#ifndef RANDOM_VECTOR_GENERATOR_H_
#define RANDOM_VECTOR_GENERATOR_H_

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>


class RandomVectorGenerator
{
protected:
    typedef boost::normal_distribution<double>
            NormalDistribution;
    typedef boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> >
            RandomNumberGenerator;


protected:
    std::vector<RandomNumberGenerator> random_number_generators_;


public:
    RandomVectorGenerator(const Eigen::Vector3d& std_dev = Eigen::Vector3d::Zero())
    {
        for (int i = 0; i < std_dev.cols(); i++)
        {
            NormalDistribution normal_distribution(0.0, std_dev[i]);
            RandomNumberGenerator random_number_generator(boost::mt19937(time(NULL)), normal_distribution);
            random_number_generators_.push_back(random_number_generator);
        }
    }


    Eigen::Vector3d get_vector()
    {
        Eigen::Vector3d vector;

        for (int i = 0; i < vector.cols(); i++)
            vector[i] = random_number_generators_[i]();

        return vector;
    }
};


class RandomQuaternionGenerator : protected RandomVectorGenerator
{
public:
    Eigen::Quaterniond get_quaternion()
    {
        Eigen::Vector3d euler_angles = get_vector();

        Eigen::AngleAxisd rotation_x(euler_angles[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotation_y(euler_angles[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation_z(euler_angles[2], Eigen::Vector3d::UnitX());

        Eigen::Quaterniond quaternion;
        quaternion = rotation_z * quaternion;
        quaternion = rotation_y * quaternion;
        quaternion = rotation_x * quaternion;

        return quaternion;
    }
};


#endif
