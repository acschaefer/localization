#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

#include <Eigen/Dense>


template<typename RobotStateT>
class Particle
{
protected:
    RobotStateT pose_;
    double weight_;


public:
    Particle(const RobotStateT& pose = RobotStateT::Identity(), double weight = 1.0)
    {
        pose_   = pose;
        weight_ = weight;
    }


    void set_pose(const RobotStateT& pose)
    {
        pose_ = pose;
    }


    RobotStateT get_pose()
    {
        return pose_;
    }


    void set_weight(double weight)
    {
        weight_ = weight;
    }


    double get_weight()
    {
        return weight_;
    }
};


#endif
