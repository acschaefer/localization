#ifndef MOTION_MODEL_4D_H_
#define MOTION_MODEL_H_4D_ MOTION_MODEL_4D_H_

#include "motion_model_3d.h"


class MotionModel4d : public MotionModel3d
{
protected:
    double var_z_;


public:
    MotionModel4d()
    {
        alpha_.push_back(0.1);
        var_z_ = 0.1;
    }


    void set_alpha(const std::vector<double>& alpha)
    {
        alpha_ = alpha;
    }


    void set_start_pose(const tf::Transform& start_pose, double var_xy,
                        double var_z, double var_yaw)
    {
        start_pose_ = start_pose;
        var_xy_     = var_xy;
        var_z_      = var_z;
        var_yaw_    = var_yaw;
    }


    void init(std::vector<Particle>& particles)
    {
        MotionModel3d::init(particles);

        GaussNumberGenerator z_generator(0.0, var_z_);

        for (int p = 0; p < particles.size(); p++)
        {
            tf::Vector3 translation(particles[p].get_pose().getOrigin());
            translation.setZ(z_generator());
            particles[p].set_pose(tf::Transform(particles[p].get_pose().getRotation(),
                                                translation));
        }
    }


    tf::Transform sample_pose(const tf::Transform& last_pose,
                         const tf::Transform& movement)
    {
        tf::Transform new_pose = MotionModel3d::sample_pose(last_pose, movement);

        GaussNumberGenerator z_generator(0.0, alpha_[4]*movement.getOrigin().length());
        tf::Vector3 translation(new_pose.getOrigin());
        translation.setZ(translation.getZ() + z_generator());
        new_pose.setOrigin(translation);

        return new_pose;
    }
};


#endif
