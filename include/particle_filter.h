#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include <vector>

#include <Eigen/Dense>

#include "particle.h"
#include "motion_model.h"
#include "sensor_model.h"


class ParticleFilter
{
protected:
    std::vector<Particle> particles_;
    MotionModel motion_model_;
    SensorModel sensor_model_;


public:
    ParticleFilter(const MotionModel& motion_model, const SensorModel& sensor_model)
     : motion_model_(motion_model), sensor_model_(sensor_model)
    {
    }


    bool init(unsigned int n_particles, const Eigen::Isometry3d& start_pose)
    {
        particles_.resize(n_particles, Particle(start_pose));
        motion_model_.update(particles_, Eigen::Isometry3d::Identity());
    }


    bool update_motion(const Eigen::Isometry3d& movement)
    {
        motion_model_.update(particles_, movement);
    }


/*    bool integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        return true;
    }*/
};


#endif
