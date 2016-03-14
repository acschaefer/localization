#ifndef PARTICLE_H_
#define PARTICLE_H_ PARTICLE_H_

#include <tf/tf.h>


class Particle
{
protected:
    tf::Transform pose_;
    double weight_;


public:
    Particle(const tf::Transform& pose = tf::Transform::getIdentity(), double weight = 1.0)
    {
        pose_   = pose;
        weight_ = weight;
    }


    void set_pose(const tf::Transform& pose = tf::Transform::getIdentity())
    {
        pose_ = pose;
    }


    tf::Transform& get_pose()
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
