#ifndef SENSOR_MODEL_H_
#define SENSOR_MODEL_H_ SENSOR_MODEL_H_

// Particles used by the particle filter.
#include <particle.h>


/// Sensor model for use with the ParticleFilter class for robot localization.
template<typename MeasurementT>
class SensorModel
{
public:
    /// This typedef grants subordinate classes access
    /// to the template type by calling SensorModelImplementation::Measurement.
    typedef MeasurementT Measurement;


public:
    /// Computes the weights of the particles
    /// according to the given measurement.
    virtual void compute_particle_weights(const MeasurementT& measurement,
                                          std::vector<Particle>& particles) = 0;
};


/// Void sensor model implementation for particle filters
/// without a sensor model.
class NoSensorModel : public SensorModel<int>
{
public:
    void compute_particle_weights(const int& measurement,
                                  std::vector<Particle>& particles)
    {
    }
};


#endif
