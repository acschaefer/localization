#ifndef SENSOR_MODEL_ENDPOINT_H_
#define SENSOR_MODEL_ENDPOINT_H_ SENSOR_MODEL_ENDPOINT_H_

// Enable/disable multithreading.
#define MULTITHREADING true

// Enable/disable saving PCD files for debugging.
#define SAVE_PCD true

// Standard library.
#include <vector>

// Boost.
#include <boost/thread.hpp>

// Point Cloud Library.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

// ROS.
#include <ros/console.h>
#include <pcl_ros/transforms.h>

// Particle filter.
#include "localizer/particle.h"
#include "localizer/sensor_model.h"


/// Determines the weight of a particle by comparing a given point cloud to a point cloud map using the
/// endpoint model.
class SensorModelEndpoint : public SensorModel<pcl::PointCloud<pcl::PointXYZI> >
{
protected:
    /// 3D tree for computing the distances between points.
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;

    /// Voxel edge length used for sparsifying the incoming point clouds.
    double res_;

    /// Lower bound of the resolution used to sparsify point clouds.
    static const double min_res;


public:
    /// Constructor.
    /// \param[in] PCD file used as a map when weighting the particles.
    SensorModelEndpoint(pcl::PointCloud<pcl::PointXYZI>::ConstPtr map, double res = min_res)
        : res_(min_res)
    {
        set_sparsification_resolution(res);

        // Downsample map and pass it to nearest-neighbor algorithm.
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_sparse(new pcl::PointCloud<pcl::PointXYZI>());
        sparsify(*map, *map_sparse);
        kdtree_.setInputCloud(map_sparse);

        if (SAVE_PCD)
        {
            pcl::io::savePCDFileASCII("map.pcd", *map);
            ROS_DEBUG("Saved \"map.pcd\".");
        }
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
                                          &SensorModelEndpoint::compute_particle_weights_thread,
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
    /// Set the resolution used for sparsifying the map and incoming lidar scans.
    void set_sparsification_resolution(double res)
    {
        // Make sure the voxel edge length used for sparsification is not smaller than allowed.
        if (res < min_res)
            ROS_WARN_STREAM("Sparsification resolution set to minimum admissible resolution " << min_res << ".");

        res_ = std::max(min_res, res);
    }


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
        filter.setLeafSize(res_, res_, res_);
        filter.filter(pc_sparse);
    }


    /// Computes the weight of the particle.
    /// \param[in] pc_robot measured point cloud in the robot frame.
    /// \param[in,out] particle particle whose weight is computed.
    virtual void compute_particle_weight(const pcl::PointCloud<pcl::PointXYZI>& pc_robot, Particle& particle)
    {
        // Set the maximum distance between two points used for weighting the particles.
        const float d_max = 0.5f;

        // Transform the sensor point cloud from the particle frame to the map frame.
        pcl::PointCloud<pcl::PointXYZI> pc_map;
        pcl_ros::transformPointCloud(pc_robot, pc_map, particle.pose);

        // Compute how well the measurements match the map by computing the point-to-point distances.
        std::vector<int> k_indices(1, -1);
        std::vector<float> d(1, 0.0f);
        float d_tot = 0.0f;
        int n_tot = 0;
        for (size_t i = 0; i < pc_map.size(); ++i)
        {
            // Determine the squared distance to the nearest point.
            if (std::isfinite(pc_map[i].x) && std::isfinite(pc_map[i].y) && std::isfinite(pc_map[i].z))
                kdtree_.nearestKSearch(pc_map[i], 1, k_indices, d);
            else
                continue;

            // Sum up the capped distances.
            d_tot += std::min(d_max, std::sqrt(d[0]));

            // Sum up the number of finite points.
            n_tot++;
        }

        // Set the particle weight to the mean distance.
        particle.weight = d_tot / n_tot;

        // Save the point clouds for debugging reasons.
        if (SAVE_PCD)
        {
            std::stringstream filename;
            ros::Time now(ros::Time::now());
            filename << now.sec << now.nsec << ".pcd";
            pcl::io::savePCDFileASCII(filename.str(), pc_map);
            ROS_DEBUG_STREAM("Saved \"" << filename.str() << "\".");
        }
    }
};


const double SensorModelEndpoint::min_res = 1.0e-3;


#endif
