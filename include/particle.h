#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

#include <Eigen/Dense>


class Particle
{
protected:
    Eigen::Isometry3d pose_;
    double weight_;


public:
    Particle(const Eigen::Isometry3d& pose = Eigen::Isometry3d::Identity(),
             double weight = 1.0)
    {
        pose_   = pose;
        weight_ = weight;
    }


    bool move(const Eigen::Isometry3d& movement)
    {
        pose_ = movement * pose_;
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
