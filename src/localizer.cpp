#include "particle_filter.h"


int main(int argc, char** argv)
{
    ParticleFilter<MotionModel, SensorModel> particle_filter;
    particle_filter.init(1e4);
    particle_filter.update_motion(4);
    particle_filter.integrate_measurement(3);

    return 0;
}
