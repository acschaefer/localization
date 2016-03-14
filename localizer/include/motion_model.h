#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

#include <vector>
#include <Eigen/Dense>

#include "particle.h"


template<typename RobotStateT>
class MotionModel
{
public:
    typedef RobotStateT RobotState;


public:
    void update(const RobotStateT& movement,
                std::vector<Particle<RobotStateT> >& particles)
    {
        for (int p = 0; p < particles.size(); p++)
        {
            RobotStateT new_pose = sample(movement, particles[p].get_pose());
            particles[p].set_pose(new_pose);
        }
    }


protected:
    virtual RobotStateT sample(const RobotStateT& last_pose,
                               const RobotStateT& movement)
    {
        return movement * last_pose;
    }
};


#endif
