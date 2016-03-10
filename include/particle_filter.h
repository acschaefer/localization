#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include <vector>

#include <Eigen/Dense>

#include "particle.h"


template <typename MotionModelT, typename SensorModelT>
class ParticleFilter
{
protected:
    std::vector<Particle> particles_;
    MotionModelT motion_model_;
    SensorModelT sensor_model_;


public:
    bool init(unsigned int n_particles, const Eigen::Isometry3d& start_pose)
    {
        particles_.resize(n_particles, Particle(start_pose));
        motion_model_.update(particles_, Eigen::Isometry3d::Identity());
    }


    bool update_motion(const Eigen::Isometry3d& movement)
    {
        return motion_model_.update(particles_, movement);
    }


    bool integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        return true;
    }
};


#endif
