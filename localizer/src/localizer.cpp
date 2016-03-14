#include "particle_filter.h"
#include "motion_model_3d.h"


int main(int argc, char** argv)
{
    std::vector<double> alpha(4, 0.1);
    boost::shared_ptr<MotionModel3d> motion_model
            = boost::shared_ptr<MotionModel3d>(new MotionModel3d(alpha));

    ParticleFilter particle_filter(motion_model);
    particle_filter.init(1e4);

    for (int i = 0; i < 100; i++)
    {
        tf::Transform movement;
        movement.getOrigin().setX(1.0);
        particle_filter.update_motion(movement);

        tf::Vector3 mean = particle_filter.get_mean();
        std::cout << "[" << mean[0] << "; " << mean[1] << "; " << mean[2] << "]" << std::endl;
    }

    return 0;
}
