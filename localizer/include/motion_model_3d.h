#ifndef MOTION_MODEL_3D_H_
#define MOTION_MODEL_H_3D_ MOTION_MODEL_3D_H_

#include "motion_model.h"
#include "random_generators.h"


class MotionModel3d : public MotionModel
{
protected:
    std::vector<double> alpha_;
    tf::Transform start_pose_;
    double var_xy_, var_yaw_;


public:
    MotionModel3d()
        : alpha_(std::vector<double>(4, 1.0)),
          start_pose_(tf::Transform::getIdentity()),
          var_xy_(1.0), var_yaw_(0.1)
    {
    }


    void set_alpha(const std::vector<double>& alpha)
    {
        alpha_ = alpha;
    }


    void set_start_pose(const tf::Transform& start_pose,
                        double var_xy, double var_yaw)
    {
        start_pose_ = start_pose;
        var_xy_     = var_xy;
        var_yaw_    = var_yaw;
    }


    void init(std::vector<Particle>& particles)
    {
        tf::Matrix3x3 rotation(start_pose_.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        GaussNumberGenerator yaw_generator(yaw, var_yaw_);
        VectorPolarGenerator vector_generator(
                    0.0, var_xy_, 0.0, 2.0*M_PI, 1.0);

        for (int p = 0; p < particles.size(); p++)
        {
            rotation.setRPY(0.0, 0.0, yaw_generator());
            tf::Vector3 translation(start_pose_.getOrigin() + vector_generator());
            translation.setZ(0.0);
            particles[p].set_pose(tf::Transform(rotation, translation));
        }
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

        double rot1_noisy   = rot1;
        double trans_noisy  = trans;
        double rot2_noisy   = rot2;

        if (var_rot1 > 0.0)
        {
            GaussNumberGenerator rot1_generator(0.0, var_rot1);
            rot1_noisy -= rot1_generator();
        }
        if (var_trans > 0.0)
        {
            GaussNumberGenerator trans_generator(0.0, var_trans);
            trans_noisy -= trans_generator();
        }
        if (var_rot2 > 0.0)
        {
            GaussNumberGenerator rot2_generator(0.0, var_rot2);
            rot2_noisy -= rot2_generator();
        }

        tf::Vector3 position(last_pose.getOrigin());
        position.setX(last_pose.getOrigin().x() + trans_noisy * cos(last_theta+rot1_noisy));
        position.setY(last_pose.getOrigin().y() + trans_noisy * sin(last_theta+rot1_noisy));

        tf::Matrix3x3 orientation;
        orientation.setRPY(last_roll, last_pitch, last_theta + rot1_noisy + rot2_noisy);

        tf::Transform new_pose(orientation, position);
        return new_pose;
    }
};


#endif
