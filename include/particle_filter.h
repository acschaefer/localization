#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include <vector>

#include "particle.h"
#include "motion_model.h"
#include "sensor_model.h"


template <typename MotionModelT, typename SensorModelT>
class ParticleFilter
{
protected:
    std::vector<Particle> particles_;
    MotionModelT motion_model_;
    SensorModelT sensor_model_;


public:
    bool init(unsigned int n_particles)
    {
        particles_.resize(n_particles, Particle(0.5));
    }

    bool update_motion(const typename MotionModelT::State& state)
    {
        return true;
    }


    bool integrate_measurement(const typename SensorModelT::Measurement& measurement)
    {
        return true;
    }
};


#endif
