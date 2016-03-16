#ifndef SENSOR_MODEL_H_
#define SENSOR_MODEL_H_ SENSOR_MODEL_H_

#include <particle.h>


template<typename MeasurementT>
class SensorModel
{
public:
    typedef MeasurementT Measurement;


public:
    virtual void compute_particle_weights(const MeasurementT& measurement,
                                          std::vector<Particle>& particles) = 0;
};


class SensorModelExample : public SensorModel<int>
{
public:
    void compute_particle_weights(const int& measurement,
                                  std::vector<Particle>& particles)
    {
    }
};


#endif
