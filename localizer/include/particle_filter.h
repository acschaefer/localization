#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include <vector>

#include <Eigen/Dense>

#include "particle.h"
#include "motion_model.h"
#include "sensor_model.h"


template<typename MotionModelT>
class ParticleFilter
{
public:
    typedef typename MotionModelT::RobotState RobotStateT;


protected:
    MotionModelT motion_model_;
    std::vector<Particle<RobotStateT> > particles_;


public:
    ParticleFilter(const MotionModelT& motion_model)
     : motion_model_(motion_model)
    {
    }


    void init(unsigned int n_particles, const RobotStateT& start_pose = RobotStateT::Identity())
    {
        particles_.resize(n_particles, Particle<RobotStateT>(start_pose));
        motion_model_.update(RobotStateT::Identity(), particles_);
    }


    void update_motion(const RobotStateT& movement)
    {
        motion_model_.update(movement, particles_);
    }


    RobotStateT get_mean()
    {
        RobotStateT mean;

        for (int p = 0; p < particles_.size(); p++)
            mean.translation() += particles_[p].get_pose().translation();

        mean.translation() /= particles_.size();

        return mean;
    }
};


#endif
