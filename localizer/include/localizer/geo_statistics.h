#ifndef GEO_STATISTICS_H
#define GEO_STATISTICS_H

// Standard libraries.
#include <vector>
#include <numeric>

// ROS coordinate transformations and logging.
#include <tf/tf.h>
#include <ros/console.h>

// Eigen.
#include <Eigen/Eigenvalues>


/// Computes the mean of all given vectors.
tf::Vector3 vectormean(const std::vector<tf::Vector3>& v)
{
    return std::accumulate(v.begin(), v.end(), tf::Vector3(0.0, 0.0, 0.0)) / v.size();
}


/// Scales the given vectors by the given scaling factor.
tf::Vector3 vectorscale(const tf::Vector3& v, const double& s)
{
    return v * s;
}


/// Computes the weighted average of all given vectors using the given weight factors.
tf::Vector3 vectormean(std::vector<tf::Vector3> v, const std::vector<double>& w)
{
    ROS_ERROR_COND(v.size() != w.size(), "Numbers of vectors and weights do not match.");

    std::transform(v.begin(), v.end(), w.begin(), v.begin(), vectorscale);

    return vectormean(v);
}


/// Compute the weighted average of all given quaternions using the given weight factors.
/// This method uses the quaternion averaging approach proposed by Markley:
/// F. Landis Markley, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman.
/// Averaging Quaternions.
/// Journal of Guidance, Control, and Dynamics, 30(4):1193-1197, 2007.
tf::Quaternion quatmean(const std::vector<tf::Quaternion>& q, const std::vector<double>& w)
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    Eigen::Vector4d qi;
    for (size_t i = 0u; i < q.size(); ++i)
    {
        qi << q[i][0], q[i][1], q[i][2], q[i][3];
        M += w[i] * qi * qi.transpose();
    }

    Eigen::EigenSolver<Eigen::Matrix4d> solver(M, true);
    int imax;
    solver.eigenvalues().real().maxCoeff(&imax);
    Eigen::Vector4d qm;
    qm << solver.eigenvectors().col(imax).real();

    return tf::Quaternion(qm(0), qm(1), qm(2), qm(3));
}


/// Compute the mean of all given quaternions.
tf::Quaternion quatmean(const std::vector<tf::Quaternion>& q)
{
    return quatmean(q, std::vector<double>(q.size(), 1.0/q.size()));
}


/// Compute the weighted average of all given transformations using the given weight factors.
tf::Transform tfmean(const std::vector<tf::Transform>& tf, const std::vector<double>& w)
{
    std::vector<tf::Vector3> v;
    std::vector<tf::Quaternion> q;
    for (size_t i = 0u; i < tf.size(); ++i)
    {
        v.push_back(tf[i].getOrigin());
        q.push_back(tf[i].getRotation());
    }

    return tf::Transform(quatmean(q, w), vectormean(v, w));
}


/// Compute the average of all given transformations.
tf::Transform tfmean(const std::vector<tf::Transform>& tf)
{
    return tfmean(tf, std::vector<double>(tf.size(), 1.0/tf.size()));
}


#endif
