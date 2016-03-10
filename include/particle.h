#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>


class Particle
{
public:
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

private:
    Eigen::Isometry3d pose_;
    double weight_;

public:
    Particle(const Eigen::Vector3d& mean_position = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d& std_dev_position = Eigen::Vector3d::Identity(),
             const Eigen::Vector3d& mean_orientation = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d& std_dev_orientation = Eigen::Vector3d::Identity())
    {
        boost::mt19937 variate_generator(time(NULL));
        Eigen::Vector3d euler_angles;
        for (int i = 0; i < mean_orientation.cols(); i++)
        {
            boost::normal_distribution<double> normal_distribution(mean_orientation[i], std_dev_orientation[i]);
            boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> >
                random_number_generator(variate_generator, normal_distribution);

            euler_angles[i] = random_number_generator();
        }

        Eigen::AngleAxisd rotation_x(euler_angles[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotation_y(euler_angles[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation_z(euler_angles[2], Eigen::Vector3d::UnitX());

        pose_ = rotation_z * pose_;
        pose_ = rotation_y * pose_;
        pose_ = rotation_x * pose_;

        for (int i = 0; i < mean_position.cols(); i++)
        {
            boost::normal_distribution<double> normal_distribution(mean_position[i], std_dev_position[i]);
            boost::random::variate_generator<boost::mt19937, boost::normal_distribution<double> >
                random_number_generator(variate_generator, normal_distribution);

            pose_.translation()[i] = random_number_generator();
        }

        weight_ = 1.0;
    }


    Eigen::Isometry3d get_pose()
    {
        return pose_;
    }


    double get_weight()
    {
        return weight_;
    }
};


#endif
