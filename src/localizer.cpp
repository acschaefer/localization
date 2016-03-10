#include <Eigen/Dense>

#include "particle_filter.h"
#include "motion_model.h"
#include "sensor_model.h"


int main(int argc, char** argv)
{
    ParticleFilter<MotionModel, SensorModel> particle_filter;
    particle_filter.init(1e4, Eigen::Isometry3d::Identity());
    particle_filter.update_motion(Eigen::Isometry3d::Identity());
    particle_filter.integrate_measurement(3);

    return 0;
}
