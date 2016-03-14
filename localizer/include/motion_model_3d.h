#ifndef MOTION_MODEL_3D_H_
#define MOTION_MODEL_H_3D_ MOTION_MODEL_3D_H_

#include "motion_model.h"
#include "random_generators.h"


class MotionModel3d : public MotionModel
{
protected:
    std::vector<double> alpha_;


public:
    MotionModel3d(const std::vector<double> alpha = std::vector<double>(4, 0.1))
        : alpha_(alpha)
    {
    }


    void init(const tf::Transform& start_pose, std::vector<Particle>& particles)
    {
        init(start_pose, tf::Vector3(5.0, 5.0, 0.5), particles);
    }


    void init(const tf::Transform& start_pose, const tf::Vector3& max_movement,
              std::vector<Particle>& particles)
    {
        UniformVectorGenerator generator(tf::Vector3(), max_movement);

        for (int p = 0; p < particles.size(); p++)
            particles[p].set_pose(
                        tf::Transform(tf::Matrix3x3(),
                                      start_pose.getOrigin() + generator()));
    }


    tf::Transform sample(const tf::Transform& last_pose,
                         const tf::Transform& movement)
    {
        tfScalar last_roll, last_pitch, last_yaw;
        tf::Matrix3x3 last_rotation(last_pose.getRotation());
        last_rotation.getRPY(last_roll, last_pitch, last_yaw);
        double last_theta   = last_yaw;

        double d_x          = movement.getOrigin().x();
        double d_y          = movement.getOrigin().y();
        tfScalar d_roll, d_pitch, d_yaw;
        tf::Matrix3x3 d_rotation(movement.getRotation());
        d_rotation.getRPY(d_roll, d_pitch, d_yaw);
        double d_theta      = d_yaw;

        double rot1         = std::atan2(d_y, d_x) - last_theta;
        double trans        = std::sqrt(d_x*d_x + d_y*d_y);
        double rot2         = d_theta - rot1;

        double var_rot1     = alpha_[0]*std::abs(rot1) + alpha_[1]*trans;
        double var_trans    = alpha_[2]*trans + alpha_[3]*(std::abs(rot1)+std::abs(rot2));
        double var_rot2     = alpha_[0]*rot2 + alpha_[1]*trans;

        GaussNumberGenerator rot1_generator(0.0, var_rot1);
        GaussNumberGenerator trans_generator(0.0, var_trans);
        GaussNumberGenerator rot2_generator(0.0, var_rot2);

        double rot1_noise   = rot1  - rot1_generator();
        double trans_noise  = trans - trans_generator();
        double rot2_noise   = rot2  - rot2_generator();

        tf::Vector3 position(last_pose.getOrigin());
        position.setX(last_pose.getOrigin().x() + trans_noise * cos(last_theta+rot1_noise));
        position.setY(last_pose.getOrigin().y() + trans_noise * sin(last_theta+rot1_noise));

        tf::Matrix3x3 orientation;
        orientation.setRPY(last_roll, last_pitch, last_theta + rot1_noise + rot2_noise);

        tf::Transform pose(orientation, position);
        return pose;
    }
};


#endif
