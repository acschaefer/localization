#ifndef RANDOM_VECTOR_GENERATOR_H_
#define RANDOM_VECTOR_GENERATOR_H_

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>


class RandomNumberGenerator
{
protected:
    boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> > generator_;


public:
	RandomNumberGenerator(double mean = 0.0, double var = 1.0)
	 : generator_(boost::mt19937(time(NULL)), boost::normal_distribution<double>(mean, var))
    {
    }


	double generate_number()
	{
		return generator_();
	}
};


template<int Dim>
class RandomVectorGenerator
{
protected:
    typedef Eigen::Matrix<double, Dim, 1> Vector;


protected:
    std::vector<RandomNumberGenerator> number_generators_;
    

public:
    RandomVectorGenerator(const Vector& mean, const Vector& var) 
    {    
        for (int row = 0; row < Dim; row++)
            number_generators_.push_back(RandomNumberGenerator(mean[row], var[row]));
    }


    virtual Vector generate_vector()
    {
        Vector vector;

        for (int row = 0; row < Dim; row++)
            vector[row] = number_generators_[row].generate_number();

        return vector;
    }
};


class RandomAngleAxisGenerator : protected RandomVectorGenerator<3>
{
protected:
    RandomNumberGenerator angle_generator_;


public:
    RandomAngleAxisGenerator(double angle_mean, double angle_var,
                             const Eigen::Vector3d& axis_mean, const Eigen::Vector3d& axis_var)
     : RandomVectorGenerator<3>(axis_mean, axis_var), 
       angle_generator_(angle_mean, angle_var)
    {
    }	


    Eigen::AngleAxisd generate_angle_axis()
    {
		return Eigen::AngleAxisd(angle_generator_.generate_number(), generate_vector().normalized());
    }
};


class RandomPoseGenerator  
{
protected:
    RandomVectorGenerator<3> vector_generator_;
    RandomAngleAxisGenerator angle_axis_generator_;


public:
    RandomPoseGenerator(const Eigen::Vector3d& position_mean, const Eigen::Vector3d& position_var,
                        double angle_mean, double angle_var,
                        const Eigen::Vector3d& axis_mean, const Eigen::Vector3d& axis_var)
     : vector_generator_(position_mean, position_var),
       angle_axis_generator_(angle_mean, angle_var, axis_mean, axis_var)
    {
    }


    Eigen::Isometry3d generate_pose()
    {
        return Eigen::Isometry3d().fromPositionOrientationScale(
                    vector_generator_.generate_vector(), 
                    angle_axis_generator_.generate_angle_axis(),
                    Eigen::Vector3d(1.0, 1.0, 1.0));
    }
};


#endif
