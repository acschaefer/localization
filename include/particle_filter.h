#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_ PARTICLE_FILTER_H_

#include "motion_model.h"
#include "sensor_model.h"


class ParticleFilter
{
protected:
    MotionModel motion_model_;
    SensorModel sensor_model_;

public:
    bool update_motion(MotionModel::State state)
    {
        return true;
    }
};


#endif
