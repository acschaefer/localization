#ifndef MOTION_MODEL_3D_H_
#define MOTION_MODEL_H_3D_ MOTION_MODEL_3D_H_

#include <Eigen/Geometry>

#include "motion_model.h"
#include "random_generators.h"


class MotionModel3d : public MotionModel<Eigen::Isometry2d>
{
public:
    typedef Eigen::Isometry2d RobotState;


protected:
    std::vector<double> alpha_;


public:
    MotionModel3d(const std::vector<double> alpha = std::vector<double>(4, 0.1))
        : alpha_(alpha)
    {
    }


    Eigen::Isometry2d sample(const Eigen::Isometry2d& last_pose,
                             const Eigen::Isometry2d& movement)
    {
        Eigen::Rotation2D<double> rotation(0.0);

        double last_theta   = rotation.fromRotationMatrix(last_pose.rotation()).angle();

        double d_x          = movement.translation().x();
        double d_y          = movement.translation().y();
        double d_theta      = rotation.fromRotationMatrix(movement.rotation()).angle();

        double rot1         = std::atan2(d_y, d_x) - last_theta;
        double trans        = std::sqrt(d_x*d_x + d_y*d_y);
        double rot2         = d_theta - rot1;

        double var_rot1     = alpha_[0]*std::abs(rot1) + alpha_[1]*trans;
        double var_trans    = alpha_[2]*trans + alpha_[3]*(std::abs(rot1)+std::abs(rot2));
        double var_rot2     = alpha_[0]*rot2 + alpha_[1]*trans;

        double rot1_noise   = rot1  - Random::sample_gauss<double>(0.0, var_rot1);
        double trans_noise  = trans - Random::sample_gauss<double>(0.0, var_trans);
        double rot2_noise   = rot2  - Random::sample_gauss<double>(0.0, var_rot2);

        Eigen::Isometry2d pose(last_pose);
        pose.translation().x()  += trans_noise * cos(last_theta+rot1_noise);
        pose.translation().y()  += trans_noise * sin(last_theta+rot1_noise);
        pose.linear() = Eigen::Rotation2D<double>(last_theta + rot1_noise + rot2_noise).matrix();

        return pose;
    }
};


#endif
