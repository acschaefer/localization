#ifndef GEO_STATISTICS_H
#define GEO_STATISTICS_H

#include <vector>
#include <numeric>
#include <tf/tf.h>
#include <Eigen/Eigenvalues>
#include <ros/console.h>


tf::Vector3 vectormean(const std::vector<tf::Vector3>& v)
{
    return std::accumulate(v.begin(), v.end(), tf::Vector3(0.0, 0.0, 0.0)) / v.size();
}


tf::Vector3 vectorscale(const tf::Vector3& v, const double& s)
{
    return v * s;
}


tf::Vector3 vectormean(std::vector<tf::Vector3> v, const std::vector<double>& w)
{
    ROS_ERROR_COND(v.size() != w.size(), "Numbers of vectors and weights do not match.");

    std::transform(v.begin(), v.end(), w.begin(), v.begin(), vectorscale);

    return vectormean(v);
}


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


tf::Quaternion quatmean(const std::vector<tf::Quaternion>& q)
{
    return quatmean(q, std::vector<double>(q.size(), 1.0/q.size()));
}


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


tf::Transform tfmean(const std::vector<tf::Transform>& tf)
{
    return tfmean(tf, std::vector<double>(tf.size(), 1.0/tf.size()));
}


#endif
