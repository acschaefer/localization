#ifndef SENSOR_MODEL_H_
#define SENSOR_MODEL_H_ SENSOR_MODEL_H_

#include <particle.h>


class SensorModel
{
public:
    typedef int Measurement;


public:
    virtual void compute_weights(const Measurement& measurement,
                                 std::vector<Particle>& particles) = 0;
};


#endif
