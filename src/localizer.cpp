#include <Eigen/Dense>

#include "particle_filter.h"
#include "motion_model.h"
#include "sensor_model.h"


int main(int argc, char** argv)
{
    Eigen::Vector3d position_var(0.050, 0.050, 0.010);
    const double angle_var = (5.0 / 180.0) * M_PI;
    Eigen::Vector3d axis_var(0.100, 0.100, 0.0);

    MotionModel motion_model(position_var, angle_var, axis_var);
    SensorModel sensor_model;
    ParticleFilter particle_filter(motion_model, sensor_model);
    particle_filter.init(1e3, Eigen::Isometry3d::Identity());

    for (int i = 0; i < 100; i++)
    {
        Eigen::Isometry3d movement;
		movement.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
        particle_filter.update_motion(movement);
        //particle_filter.integrate_measurement(3);
    }

    return 0;
}
