#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

#include <Eigen/Dense>

#include "particle.h"
#include "random_generators.h"


class MotionModel
{
protected:
    RandomPoseGenerator pose_generator_;


public:
    MotionModel(const Eigen::Vector3d& position_var, double angle_var,
                const Eigen::Vector3d axis_var)
     : pose_generator_(Eigen::Vector3d::Zero(), position_var, 0.0, angle_var,
                       Eigen::Vector3d::Zero(), axis_var)
    {
    }


    void update(std::vector<Particle>& particles, const Eigen::Isometry3d& movement)
    {
        for (int p = 0; p < particles.size(); p++)
        {
            Eigen::Isometry3d noise = pose_generator_.generate_pose();
            particles[p].move(noise * movement);
        }
    }
};


#endif
