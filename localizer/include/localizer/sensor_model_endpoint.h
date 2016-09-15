#ifndef SENSOR_MODEL_PCD_H_
#define SENSOR_MODEL_PCD_H_ SENSOR_MODEL_PCD_H_

// Enable or disable multithreading.
#define MULTITHREADING false

// Standard template libraries.
#include <vector>

// Boost multi-threading.
#include <boost/thread.hpp>

// Eigen.
#include <Eigen/Core>

// Point Cloud Library.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

// ROS.
#include <pcl_ros/transforms.h>
#include <ros/console.h>

// Particle filter.
#include "localizer/particle.h"
#include "localizer/sensor_model.h"


/// Determines the weight of a particle by comparing a given point cloud to a point cloud map using the
/// endpoint model.
class SensorModelEndpoint : public SensorModel<pcl::PointCloud<pcl::PointXYZI>::Ptr>
{
protected:
    typedef pcl::PointCloud<pcl::PointXYZI> Pc;


protected:
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;

    /// Resolution used for sparsifying the incoming point clouds.
    double res_;

    /// Lower bound of the resolution used to sparsify point clouds.
    static const double min_resolution;


public:
    /// Constructor.
    /// \param[in] PCD file used as a map when weighting the particles.
    SensorModelEndpoint(pcl::PointCloud<pcl::PointXYZI>::ConstPtr map)
        : res_(0.1)
    {
        kdtree_.setInputCloud(map);
    }


    /// Set the resolution used for sparsifying incoming lidar scans.
    void set_sparsification_resolution(double resolution)
    {
        // Make sure the resolution is not higher than allowed.
        if (resolution < min_resolution)
            ROS_WARN_STREAM("Sparsification resolution must not be less than "
                            << min_resolution << ".");

        res_ = std::max(min_resolution, resolution);
    }


    /// Computes the weights of all particles based on the given scan.
    /// \param[in] pc_robot point cloud in the robot frame of reference.
    /// \param[in,out] particles set of particles.
    virtual void compute_particle_weights(const Pc::Ptr& pc_robot, std::vector<Particle>& particles)
    {
        // If no particles or no map are given, abort.
        if (particles.size() < 1)
            return;

        // Downsample the point cloud provided by the robot.
        Pc::Ptr pc_robot_sparse = boost::make_shared<Pc>();
        sparsify_measurement(pc_robot, pc_robot_sparse);

        // Compute the particle weights.
        if (MULTITHREADING)
        {
            // Perform the following steps in parallel.
            boost::thread_group threads;

            // Compute the weights of the individual particles in parallel.
            // Use as many threads as cores are available.
            int n_threads = boost::thread::hardware_concurrency();
            for (int t = 0; t < n_threads; t++)
            {
                threads.create_thread(boost::bind(
                        &SensorModelEndpoint::compute_particle_weights_thread,
                        this,
                        pc_robot_sparse, boost::ref(particles), t));
            }

            // Wait for the threads to return.
            threads.join_all();
        }
        else
        {
            // Compute the respective weights of the particles.
            for (size_t i = 0; i < particles.size(); ++i)
                compute_particle_weight(pc_robot_sparse, particles[i]);
        }

        // Find the maximum particle weight.
        double max_weight = std::numeric_limits<double>::min();
        for (size_t i = 0; i < particles.size(); ++i)
            max_weight = std::max(max_weight, particles[i].weight);

        // Subtract the maximum particle weight from all weights so the maximum
        // weight is 0.
        for (size_t i = 0; i < particles.size(); ++i)
            particles[i].weight -= max_weight;
    }


protected:
    /// Computes the weights of a subset of particles when
    /// using multiple threads.
    /// \param[in,out] particles vector of all particles.
    /// \param[in] pc_robot lidar point cloud in the robot frame of reference.
    /// \param[in] thread number of this thread.
    void compute_particle_weights_thread(Pc::ConstPtr pc_robot, std::vector<Particle>& particles, int thread)
    {
        // Compute the weights of the individual particles.
        const int n_threads = boost::thread::hardware_concurrency();
        const int particles_per_thread = std::ceil(particles.size() / float(n_threads));
        const int start_index = thread * particles_per_thread;
        const int stop_index = std::min((int)particles.size(), (thread + 1) * particles_per_thread);
        for (size_t i = start_index; i < stop_index; ++i)
            compute_particle_weight(pc_robot, particles[i]);
    }


    /// Applies a spatial sparsification heuristic.
    /// After this operation, the points of the output point cloud are at least
    /// \c resolution_ apart from each other.
    /// \param[in] point_cloud point cloud to sparsify.
    /// \param[out] point_cloud_sparse sparsified point cloud.
    virtual void sparsify_measurement(Pc::ConstPtr point_cloud, Pc::Ptr point_cloud_sparse)
    {
        pcl::VoxelGrid<pcl::PointXYZI> filter;
        filter.setInputCloud(point_cloud);
        filter.setLeafSize(res_, res_, res_);
        filter.filter(*point_cloud_sparse);
    }


    /// Computes the weight of the particle according to the map.
    /// \param[in] pc_robot lidar point cloud w.r.t. the robot frame.
    /// \param[in,out] particle particle to compute the weight of.
    virtual void compute_particle_weight(Pc::ConstPtr pc_robot, Particle& particle)
    {
        // Set parameters.
        const double p_min = 1e-9;
        const float d_max = 0.5f;

        // Transform the sensor point cloud from the particle frame to the map frame.
        pcl::PointCloud<pcl::PointXYZI> pc_map;
        pcl_ros::transformPointCloud(*pc_robot, pc_map, particle.pose);

        // If the given point cloud is empty, return the minimum weight.
        if (pc_map.size() < 1)
        {
            ROS_WARN_THROTTLE(1.0, "Cannot compute particle weight given empty point cloud.");
            particle.weight = p_min;
            return;
        }

        // Compute how well the measurement matches the map by using the point-to-point distances.
        std::vector<int> k_indices(1, 0);
        std::vector<float> d(1, 0.0f);
        float d_tot = 0.0f;
        for (size_t i = 0; i < pc_map.size(); ++i)
        {
            // Determine the squared distance to the nearest point.
            kdtree_.nearestKSearch(pc_map[i], 1, k_indices, d);

            // Sum up the capped distances.
            d_tot += std::min(d_max, std::sqrt(d.front()));
        }

        // Compute the mean distance.
        float d_mean = d_tot / pc_map.size();

        // Convert the error to a likelihood.
        particle.weight = std::max(p_min, 1.0 - d_mean/d_max);
    }
};


const double SensorModelEndpoint::min_resolution = 1.0e-9;


#endif
