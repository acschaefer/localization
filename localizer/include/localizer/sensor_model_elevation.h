#ifndef SENSOR_MODEL_ELEVATION_H_
#define SENSOR_MODEL_ELEVATION_H_ SENSOR_MODEL_ELEVATION_H_

// Enable/disable multithreading.
#define MULTITHREADING true

// Enable/disable saving debug files.
#define SAVE_FILES false

// Standard libraries.
#include <vector>

// Boost.
#include <boost/thread.hpp>

// Point Cloud Library.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS.
#include <ros/console.h>
#include <pcl_ros/transforms.h>

// Elevation map.
#include "elevation_map.h"

// Particle filter.
#include "localizer/particle.h"
#include "localizer/sensor_model.h"


/// Determines the weight of a particle by comparing the point cloud provided by the sensor to a
/// given elevation map.
class SensorModelElevation : public SensorModel<pcl::PointCloud<pcl::PointXYZI> >
{
protected:
    /// Given elevation map.
    ElevationMap<pcl::PointXYZI> map_;


public:
    /// Constructor.
    /// \param[in] map global elevation map.
    SensorModelElevation(const ElevationMap<pcl::PointXYZI>& map)
        : map_(map)
    {
        // Save the elevation map to file.
        if (SAVE_FILES)
            map_.save("map.csv");
    }


    /// Computes the weights of all particles based on the elevation map and the measured point cloud.
    /// \param[in] pc measured point cloud in the robot frame of reference.
    /// \param[in,out] particles set of particles.
    virtual void compute_particle_errors(const pcl::PointCloud<pcl::PointXYZI>& pc,
                                         std::vector<Particle>& particles)
    {
        // Compute the particle weights.
        if (MULTITHREADING)
        {
            // Compute the errors of the individual particles in parallel. Use as many threads as cores are available.
            boost::thread_group threads;
            int n_threads = boost::thread::hardware_concurrency();
            for (int t = 0; t < n_threads; t++)
                threads.create_thread(boost::bind(
                                          &SensorModelElevation::compute_particle_errors_thread,
                                          this,
                                          boost::cref(pc), boost::ref(particles), t));

            // Wait for the threads to return.
            threads.join_all();
        }
        else
        {
            // Compute the errors of all particles using one thread.
            for (size_t i = 0u; i < particles.size(); ++i)
                compute_particle_error(pc, particles[i]);
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
    /// \param[in] pc lidar point cloud in the robot frame of reference.
    /// \param[in] thread number of this thread.
    void compute_particle_errors_thread(const pcl::PointCloud<pcl::PointXYZI>& pc,
                                        std::vector<Particle>& particles, int thread)
    {
        // Compute the weights of the individual particles.
        // Distribute the particles equally over all available threads.
        const int n_threads = boost::thread::hardware_concurrency();
        const int particles_per_thread = std::ceil(particles.size() / double(n_threads));
        const int start_index = thread * particles_per_thread;
        const int stop_index = std::min((int)particles.size(), (thread+1) * particles_per_thread);
        for (int i = start_index; i < stop_index; ++i)
            compute_particle_error(pc, particles[i]);
    }


    /// Compute the error between the given point cloud and the map.
    /// \param[in] pc point cloud provided by the sensor in the robot frame.
    /// \param[in,out] particle robot position for which the error is computed.
    virtual void compute_particle_error(const pcl::PointCloud<pcl::PointXYZI>& pc, Particle& particle)
    {
        // Transform the sensor point cloud from the particle frame to the map frame.
        pcl::PointCloud<pcl::PointXYZI> pc_map;
        pcl_ros::transformPointCloud(pc, pc_map, particle.pose);

        // Compute the z-coordinate of the ground plane of the map.
        double z_map = map_.z_ground(pc_map, 0.2);

        // Compute the z-coordinate of the ground plane of the sensor point cloud.
        std::vector<double> z;
        for (size_t i = 0u; i < pc_map.size(); ++i)
            if (std::isfinite(pc_map[i].z))
                z.push_back(pc_map[i].z);
        std::sort(z.begin(), z.end());
        int n = std::max(1, std::min<int>(z.size(), (int)(0.2*z.size()+0.5)));
        double z_pc = std::accumulate(z.begin(), z.begin()+n, 0.0) / n;

        // Adjust the z-coordinate of the particle.
        particle.pose.getOrigin().setZ(particle.pose.getOrigin().getZ() + z_map - z_pc);
        pcl_ros::transformPointCloud(pc, pc_map, particle.pose);

        // Compute how well the measurements match the map by computing the mean distance between
        // the point cloud and the tiles of the elevation map.
        particle.error = map_.match(pc_map);
    }
};


#endif
