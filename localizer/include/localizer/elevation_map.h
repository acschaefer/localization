#ifndef ELEVATION_MAP_H_
#define ELEVATION_MAP_H_ ELEVATION_MAP_H_

// Standard library.
#include <array>
#include <cmath>

// Point Cloud Library.
#include <pcl/point_cloud.h>

// ROS logging.
#include <ros/console.h>


/// Converts a PCL point cloud into a 2D elevation map.
template<typename PointType>
class ElevationMap
{
protected:
    /// Map data.
    std::array<std::array<double> > map_;

    /// Edge length of the map tiles.
    double resolution_;

    /// Minimum admissible resolution.
    const double resolution_min_;

    /// Minimum x coordinate covered by the map.
    double x_min_;

    /// Minimum y coordinate covered by the map.
    double y_min_;


public:
    /// Constructor.
    ElevationMap(const pcl::PointCloud<PointType>& point_cloud, double resolution = 0.1f)
        : resolution_min_(1.0e-3)
    {
        // Set the resolution.
        resolution_ = std::max(resolution_min_, resolution);

        // Compute the limits of the point cloud in x and y direction.
        double x_min = std::numeric_limits<double>::max();
        double y_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::min();
        double y_max = std::numeric_limits<double>::min();
        for (size_t i = 0; i < point_cloud->size(); ++i)
        {
            x_min = std::min<double>(x_min, point_cloud[i].x);
            y_min = std::min<double>(y_min, point_cloud[i].y);
            x_max = std::max<double>(x_max, point_cloud[i].x);
            y_max = std::max<double>(y_max, point_cloud[i].y);
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
        for (size_t i = 0; i < point_cloud.size(); ++i)
        {
            size_t ix, iy;
            get_tile(point_cloud[i], ix, iy);
            if (std::isnan(map_[ix][iy]))
                map_[ix][iy] = std::numeric_limits<double>::min();

            map_[x][y] = std::max(map_[ix][iy], p->z);
        }
    }


    /// Returns the map value correspoding to the given point.
    double elevation(const PointType& point)
    {
        size_t ix, iy;
        if (get_tile(point, ix, iy))
            return elevation(ix, iy);
        else
            return std::numeric_limits<double>::quiet_NaN();
    }


    /// Returns the value of the map tile with the given index.
    double elevation(size_t ix, size_t iy)
    {
        if (check(ix, iy))
            return map_[ix][iy];
        else
            return std::numeric_limits<double>::quiet_NaN();
    }


    /// Computes the mean distance between two elevation maps.
    double diff(const ElevationMap& map, double d_max = std::numeric_limits<double>::max())
    {
        // Check if both maps have the same resolution.
        if (resolution_ != map.resolution_)
            ROS_ERROR("ElevationMap objects must have the same resolution to allow for comparison.");

        // Compute the total height distance between the maps.
        double d = 0.0;
        size_t n = 0;
        for (size_t ix = 0; ix < map_.size(); ++ix)
            for (size_t iy = 0; iy < map_[0].size(); ++iy)
            {
                // Compute the center of the map tile.
                PointXYZ point(x_min_ + (ix+0.5)*resolution_, y_min_ + (iy+0.5)*resolution_, 0.0);

                // Check if both maps define an elevation value.
                if (std::isnan(elevation(point) || std::isnan(map.elevation(point)))))
                    continue;

                // Add the height difference to the total difference.
                d += std::min(std::abs(elevation(point) - map.elevation(point)), d_max);
                n++;
            }

        // Return the mean of the distances.
        if (n < 1)
            return 0.0;
        else
            return d / n;
    }


    /// Returns the resolution of the map.
    double resolution()
    {
        return resolution_;
    }


protected:
    /// Checks if the given map tile indices are valid.
    bool check(size_t ix, size_t, iy)
    {
        return 0 <= ix_tmp && ix_tmp < map_.size()
            && 0 <= iy_tmp && iy_tmp < map_[0].size();
    }

    /// Returns the index of the tile where the given point resides.
    /// If the point lies outside the map, this method returns \c false.
    bool get_tile(const PointT& point, size_t& ix, size_t& iy)
    {
        int ix_tmp = std::floor((p.x - x_min_) / resolution_);
        int iy_tmp = std::floor((p.y - y_min_) / resolution_);

        if (check(ix_tmp, iy_tmp))
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
