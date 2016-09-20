#ifndef ELEVATION_MAP_H_
#define ELEVATION_MAP_H_ ELEVATION_MAP_H_

// Standard library.
#include <array>
#include <cmath>

// Point Cloud Library.
#include <pcl/point_cloud.h>

// ROS.
#include <ros/console.h>


/// Converts a PCL point cloud into a 2D elevation map.
template<typename PointType>
class ElevationMap
{
protected:
    /// Map data.
    std::array<std::array<double> > map_;
    
    /// Resolution -- edge length of the map tiles.
    double resolution_;
    
    /// Minimum x coordinate covered by the map.
    double x_min_;
    
    /// Minimum y coordinate covered by the map.
    double y_min_;
    

public:
    /// Constructor.
    ElevationMap(pcl::PointCloud<PointType>::ConstPtr point_cloud, double resolution = 0.1f)
        : resolution_(resolution)
    {
        // Compute the limits of the point cloud in x and y direction.
        double x_min = std::numeric_limits<double>::max();
        double y_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::min();
        double y_max = std::numeric_limits<double>::min();
        for (size_t i = 0; i < point_cloud->size(); ++i)
        {
            x_min = std::min<double>(x_min, point_cloud->at(i)->x);
            y_min = std::min<double>(y_min, point_cloud->at(i)->y);
            x_max = std::max<double>(x_max, point_cloud->at(i)->x);
            y_max = std::max<double>(y_max, point_cloud->at(i)->y);
        }
        
        // Compute the corner of the map where the x and y coordinates reach their minimum.
        x_min_ = std::floor(x_min/resolution) * resolution_;
        y_min_ = std::floor(y_min/resolution) * resolution_;
        
        // Compute the size of the map.
        size_t x_size = std::ceil((x_max-x_min_) / resolution_);
        size_t y_size = std::ceil((y_max-y_min_) / resolution_);
        
        // Allocate the map and set all values to NaN.
        map_.resize(x_size);
        for (size_t i = 0; i < map_.size(); ++i)
            map_[i].resize(y_size, std::numeric_limits<double>::quiet_NaN());
        
        // Compute the elevation values.
        for (size_t i = 0; i < point_cloud->size(); ++i)
        {
            size_t ix, iy;
            get_tile(point_cloud->at(i), ix, iy);
            if (std::isnan(map_[ix][iy]))
                map_[ix][iy] = std::numeric_limits<double>::min();
            
            map_[x][y] = std::max(map_[ix][iy], p->z);
        }
    }
    
    
protected:
    /// Returns the index of the tile where the given point resides.
    /// If the point lies outside the map, this method returns \c false.
    bool get_tile(const PointT& point, size_t& ix, size_t& iy)
    {
        int ix_tmp = std::floor((p.x - x_min_) / resolution_);
        int iy_tmp = std::floor((p.y - y_min_) / resolution_);
        
        if (0 <= ix_tmp && ix_tmp < map_.size() && 0 <= iy_tmp && iy_tmp < map_[0].size())
        {
            ix = ix_tmp;
            iy = iy_tmp;
            return true;
        }
        else
            return false;
    }
};


#endif
