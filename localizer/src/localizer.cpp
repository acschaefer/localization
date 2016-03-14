#include <Eigen/Dense>

#include "particle_filter.h"
#include "motion_model_3d.h"


int main(int argc, char** argv)
{
    std::vector<double> alpha(4, 0.1);
    MotionModel3d motion_model(alpha);
    ParticleFilter<MotionModel3d> particle_filter(motion_model);
    particle_filter.init(1e4, Eigen::Isometry2d::Identity());

    for (int i = 0; i < 100; i++)
    {
        Eigen::Isometry2d movement;
        movement.translation().x() = 1.0;
        particle_filter.update_motion(movement);

        Eigen::Vector2d mean = particle_filter.get_mean().translation();
        std::cout << "[" << mean[0] << "; " << mean[1] << "]" << std::endl;
    }

    return 0;
}
