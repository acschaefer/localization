#ifndef ELEVATION_MAP_H_
#define ELEVATION_MAP_H_ ELEVATION_MAP_H_

// Standard library.
#include <array>

// Point Cloud Library.
#include <pcl/point_cloud.h>

// ROS.
#include <ros/console.h>


/// Converts a PCL point cloud into a 2D elevation map.
template<typename PointType>
class ElevationMap
{
protected:
    std::array<std::array<float> > map_;
    

public:
    ElevationMap(pcl::PointCloud<PointType>::ConstPtr cloud, float res = 0.1f)
    {
    }
};


#endif
