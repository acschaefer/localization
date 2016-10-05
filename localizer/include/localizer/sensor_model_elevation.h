#ifndef SENSOR_MODEL_ELEVATION_H_
#define SENSOR_MODEL_ELEVATION_H_ SENSOR_MODEL_ELEVATION_H_

// Enable/disable multithreading.
#define MULTITHREADING true

// Enable/disable saving debug files.
#define SAVE_TO_FILE false

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
        // Save the elevation map to file.
        if (SAVE_TO_FILE)
            map_.save("map.csv");
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


    /// Computes the distance in z-direction between the map and the point cloud in the frames of all particles.
    std::vector<double> get_dz(const pcl::PointCloud<pcl::PointXYZI>& pc_robot, std::vector<Particle>& particles)
    {
        std::vector<double> dz(particles.size(), std::numeric_limits<double>::quiet_NaN());
        pcl::PointCloud<pcl::PointXYZI> pc_map;
        for (size_t i = 0; i < particles.size(); ++i)
        {
            // Transform the sensor point cloud from the particle frame to the map frame.
            pcl_ros::transformPointCloud(pc_robot, pc_map, particles[i].pose);
            dz[i] = map_.diff(pc_map);
        }

        return dz;
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


    /// Computes the weight of the particle and optimizes its z-coordinate.
    /// \param[in] pc_robot measured point cloud in the robot frame.
    /// \param[in,out] particle particle whose weight is computed.
    virtual void compute_particle_weight(const pcl::PointCloud<pcl::PointXYZI>& pc_robot, Particle& particle)
    {
        // Transform the sensor point cloud from the particle frame to the map frame.
        pcl::PointCloud<pcl::PointXYZI> pc_map;
        pcl_ros::transformPointCloud(pc_robot, pc_map, particle.pose);

        // Adjust the z-coordinate of the particle to minimize the mean distance between the point cloud
        // and the elevation map.
        tf::Vector3 v = particle.pose.getOrigin();
        v.setZ(v.getZ() - map_.diff(pc_map));
        particle.pose.setOrigin(v);

        // Transform the sensor point cloud from the optimized particle frame to the map frame.
        pcl_ros::transformPointCloud(pc_robot, pc_map, particle.pose);

        // Compute how well the measurements match the map by computing the mean distance between
        // the point cloud and the tiles of the elevation map.
        particle.weight = map_.match(pc_map);
    }
};


const double SensorModelElevation::min_res = 0.001;


#endif
