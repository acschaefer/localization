#ifndef SENSOR_MODEL_ELEVATION_H_
#define SENSOR_MODEL_ELEVATION_H_ SENSOR_MODEL_ELEVATION_H_

// Enable/disable multithreading.
#define MULTITHREADING true

// Standard library.
#include <vector>

// Boost.
#include <boost/thread.hpp>

// Point Cloud Library.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

// ROS.
#include <ros/console.h>
#include <pcl_ros/transforms.h>

// Elevation map.
#include "elevation_map.h"

// Particle filter.
#include "localizer/particle.h"
#include "localizer/sensor_model.h"


/// Determines the weight of a particle by comparing two elevation maps: one generated from the global map and one
/// generated from the robot's measurements.
class SensorModelElevation : public SensorModel<pcl::PointCloud<pcl::PointXYZI> >
{
protected:
    /// Lower bound of the resolution used to sparsify point clouds.
    static const double min_res;

    /// Elevation map corresponding to global map.
    ElevationMap<pcl::PointXYZI> map_;


public:
    /// Constructor.
    /// \param[in] PCD file that is converted to the global elevation map.
    SensorModelElevation(const pcl::PointCloud<pcl::PointXYZI>& map, double res = min_res)
        : map_(map, res)
    {
    }


    /// Computes the weights of all particles based on the map and the measured point cloud.
    /// \param[in] pc_robot measured point cloud in the robot frame of reference.
    /// \param[in,out] particles set of particles.
    virtual void compute_particle_weights(const pcl::PointCloud<pcl::PointXYZI>& pc_robot,
                                          std::vector<Particle>& particles)
    {
        // Downsample the point cloud provided by the robot.
        pcl::PointCloud<pcl::PointXYZI> pc_sparse;
        sparsify(pc_robot, pc_sparse);

        // Compute the particle weights.
        if (MULTITHREADING)
        {
            // Compute the weights of the individual particles in parallel. Use as many threads as cores are available.
            boost::thread_group threads;
            int n_threads = boost::thread::hardware_concurrency();
            for (int t = 0; t < n_threads; t++)
            {
                threads.create_thread(boost::bind(
                                          &SensorModelElevation::compute_particle_weights_thread,
                                          this,
                                          boost::cref(pc_sparse), boost::ref(particles), t));
            }

            // Wait for the threads to return.
            threads.join_all();
        }
        else
        {
            // Compute the weights of all particles.
            for (size_t i = 0; i < particles.size(); ++i)
                compute_particle_weight(pc_sparse, particles[i]);
        }
    }


protected:
    /// Computes the weights of a subset of particles when using multiple threads.
    /// \param[in,out] particles vector of all particles.
    /// \param[in] pc_robot lidar point cloud in the robot frame of reference.
    /// \param[in] thread number of this thread.
    void compute_particle_weights_thread(const pcl::PointCloud<pcl::PointXYZI>& pc_robot,
                                         std::vector<Particle>& particles, int thread)
    {
        // Compute the weights of the individual particles.
        // Equally distribute the particles over the available threads.
        const int n_threads = boost::thread::hardware_concurrency();
        const int particles_per_thread = std::ceil(particles.size() / double(n_threads));
        const int start_index = thread * particles_per_thread;
        const int stop_index = std::min((int)particles.size(), (thread+1) * particles_per_thread);
        for (int i = start_index; i < stop_index; ++i)
            compute_particle_weight(pc_robot, particles[i]);
    }


    /// Downsamples the point cloud using a voxel grid.
    /// \param[in] pc point cloud to sparsify.
    /// \param[out] pc_sparse sparsified point cloud.
    virtual void sparsify(const pcl::PointCloud<pcl::PointXYZI>& pc, pcl::PointCloud<pcl::PointXYZI>& pc_sparse)
    {
        pcl::VoxelGrid<pcl::PointXYZI> filter;
        filter.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(pc));
        double res = map_.resolution();
        filter.setLeafSize(res, res, res);
        filter.filter(pc_sparse);
    }


    /// Computes the weight of the particle.
    /// \param[in] pc_robot measured point cloud in the robot frame.
    /// \param[in,out] particle particle whose weight is computed.
    virtual void compute_particle_weight(const pcl::PointCloud<pcl::PointXYZI>& pc_robot, Particle& particle)
    {
        // Set the maximum distance between two points used for weighting the particles.
        const double d_max = 0.5;

        // Transform the sensor point cloud from the particle frame to the map frame.
        pcl::PointCloud<pcl::PointXYZI> pc_map;
        pcl_ros::transformPointCloud(pc_robot, pc_map, particle.pose);

        // Convert the point cloud to an elevation map.
        ElevationMap<pcl::PointXYZI> map_robot(pc_robot, map_.resolution());

        // Compute how well the measurements match the map by computing the mean distance between the tiles
        // of the elevation maps.
        particle.weight = d_max - map_robot.diff(map_);
    }
};


const double SensorModelElevation::min_res = 1.0e-3;


#endif
