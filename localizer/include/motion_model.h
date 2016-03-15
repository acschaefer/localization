#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

#include <vector>

#include "particle.h"


class MotionModel
{
public:
    virtual void init(const tf::Transform& start_pose, std::vector<Particle>& particles)
    {
        for (int p = 0; p < particles.size(); p++)
            particles[p].set_pose(start_pose);
    }


    void update(const tf::Transform& movement,
                std::vector<Particle>& particles)
    {
        for (int p = 0; p < particles.size(); p++)
        {
            tf::Transform new_pose = sample(particles[p].get_pose(), movement);
            particles[p].set_pose(new_pose);
        }
    }


protected:
    virtual tf::Transform sample(const tf::Transform& last_pose,
                                 const tf::Transform& movement)
    {
        return movement * last_pose;
    }
};


#endif
