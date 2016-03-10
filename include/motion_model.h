#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_ MOTION_MODEL_H_

#include <Eigen/Dense>

#include "particle.h"
#include "random_vector_generator.h"


class MotionModel
{
protected:
    Eigen::Vector3d std_dev_position, std_dev_orientation;
    RandomVectorGenerator random_vector_generator_;
    RandomQuaternionGenerator random_quaternion_generator_;


public:
    MotionModel()
      /*  : random_vector_generator_(),
          random_quaternion_generator_()*/
    {
    }


    bool update(std::vector<Particle>& particles, const Eigen::Isometry3d& movement)
    {
        for (int i = 0; i < particles.size(); i++)
        {
            Eigen::Isometry3d noise;
            noise.rotate(random_quaternion_generator_.get_quaternion());
            noise.translate(random_vector_generator_.get_vector());

            particles[i].move(noise * movement);
        }

        return true;
    }
};


#endif
