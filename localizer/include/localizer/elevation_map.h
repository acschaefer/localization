#ifndef ELEVATION_MAP_H_
#define ELEVATION_MAP_H_ ELEVATION_MAP_H_

// Standard libraries.
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

// Point Cloud Library.
#include <pcl/point_cloud.h>

// ROS logging.
#include <ros/console.h>


/// Converts a PCL point cloud to an elevation map.
template<typename PointType>
class ElevationMap
{
protected:
    /// Map data.
    std::vector<std::vector<double> > map_;

    /// Edge length of the map tiles.
    double resolution_;

    /// Minimum admissible resolution.
    static const double resolution_min;

    /// Minimum x coordinate covered by the map.
    double x_min_;

    /// Minimum y coordinate covered by the map.
    double y_min_;


public:
    /// Constructor.
    ElevationMap(const pcl::PointCloud<PointType>& point_cloud, double resolution = 0.1)
    {
        // Set the resolution.
        resolution_ = std::max(resolution_min, resolution);

        // Compute the limits of the point cloud in x and y direction.
        double x_min = std::numeric_limits<double>::max();
        double y_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::min();
        double y_max = std::numeric_limits<double>::min();
        for (size_t i = 0u; i < point_cloud.size(); ++i)
        {
            if (std::isfinite(point_cloud[i].x) && std::isfinite(point_cloud[i].y))
            {
                x_min = std::min<double>(x_min, point_cloud[i].x);
                y_min = std::min<double>(y_min, point_cloud[i].y);
                x_max = std::max<double>(x_max, point_cloud[i].x);
                y_max = std::max<double>(y_max, point_cloud[i].y);
            }
        }

        // Compute the corner of the map where the x and y coordinates reach their minimum.
        x_min_ = std::floor(x_min/resolution_) * resolution_;
        y_min_ = std::floor(y_min/resolution_) * resolution_;

        // Compute the size of the map.
        size_t x_size = std::max<size_t>(std::ceil((x_max-x_min_) / resolution_), 1);
        size_t y_size = std::max<size_t>(std::ceil((y_max-y_min_) / resolution_), 1);

        // Allocate the map and set all values to NaN.
        map_.resize(x_size);
        for (size_t ix = 0u; ix < map_.size(); ++ix)
            map_[ix].resize(y_size, std::numeric_limits<double>::quiet_NaN());

        // Compute the elevation values.
        for (size_t i = 0u; i < point_cloud.size(); ++i)
        {
            size_t ix, iy;
            if (tile(point_cloud[i], ix, iy))
            {
                if (std::isfinite(map_[ix][iy]))
                    map_[ix][iy] = std::max<double>(map_[ix][iy], (double)point_cloud[i].z);
                else
                    map_[ix][iy] = (double)point_cloud[i].z;

            }
        }
    }


    unsigned int fill_nan(unsigned int window_size = 3u)
    {
        // If the map is empty, return immediately.
        if (map_.empty())
            return 0u;

        // Compute half the window size.
        int d_window = std::floor(window_size / 2u);

        // Loop over all tiles of the elevation map and set the NaN tiles to the median of the values of all tiles
        // in the window.
        unsigned int n = 0u;
        std::vector<std::vector<double> > map(map_);
        for (int x = 0u; x < (int)map_.size(); ++x)
            for (int y = 0u; y < (int)map_[0].size(); ++y)
                if (std::isnan(map_[x][y]))
                {
                    // Collect the values of all map tiles in the window.
                    std::vector<double> e_window;

                    // Loop over all tiles in the window.
                    for (int wx = -d_window; wx <= +d_window; ++wx)
                        for (int wy = -d_window; wy <= +d_window; ++wy)
                            // Check if the index is valid.
                            if (x+wx >= 0 && x+wx < map_.size() && y+wy >= 0 && y+wy < map_[0].size())
                                if (!std::isnan(map_[x+wx][y+wy]))
                                    e_window.push_back(map_[x+wx][y+wy]);

                    // Compute the median of the values in the window.
                    if (!e_window.empty())
                    {
                        // Make sure the median value is located in the middle of the vector.
                        std::nth_element(e_window.begin(), e_window.begin() + e_window.size()/2, e_window.end());

                        // Assign the median to the current map tile.
                        map[x][y] = e_window[e_window.size() / 2];

                        // Increment the counter of filled NaN value.
                        ++n;
                    }
                }

        // Store the filtered map.
        map_ = map;

        return n;
    }


    /// Returns the map value correspoding to the given point.
    double elevation(const PointType& point) const
    {
        size_t ix, iy;
        if (tile(point, ix, iy))
            return elevation(ix, iy);
        else
            return std::numeric_limits<double>::quiet_NaN();
    }


    /// Returns the map value corresponding to the given coordinates.
    double elevation(double x, double y) const
    {
        size_t ix, iy;
        if (tile(x, y, ix, iy))
            return elevation(ix, iy);
        else
            return std::numeric_limits<double>::quiet_NaN();
    }


    /// Returns the value of the map tile with the given index.
    double elevation(size_t ix, size_t iy) const
    {
        if (check(ix, iy))
            return map_[ix][iy];
        else
            return std::numeric_limits<double>::quiet_NaN();
    }


    /// Returns the mean z-coordinate of the lowest map tiles above or below which the given points are located.
    double z_ground(const pcl::PointCloud<pcl::PointXYZI>& pc, double fraction) const
    {
        // Create a mask that tells which tiles of the elevation map are located below or above a point of the given
        // point cloud.
        std::vector<std::vector<bool> > mask(map_.size(), std::vector<bool>(map_[0].size(), false));
        size_t ix, iy;
        for (size_t i = 0u; i < pc.size(); ++i)
            if (tile(pc[i], ix, iy))
                if (std::isfinite(map_[ix][iy]))
                    mask[ix][iy] = true;

        // Create a vector that contains the z-coordinates of all tiles onto which points are projected.
        std::vector<double> tile_z;
        for (ix = 0u; ix < mask.size(); ++ix)
            for (iy = 0u; iy < mask[0].size(); ++iy)
                if (mask[ix][iy])
                    tile_z.push_back(map_[ix][iy]);

        // Sort the vector and compute the mean of the lowest fraction of the coordinates.
        std::sort(tile_z.begin(), tile_z.end());
        int n = std::max(1, std::min<int>(tile_z.size(), (int)(fraction*tile_z.size()+0.5)));
        return std::accumulate(tile_z.begin(), tile_z.begin()+n, 0.0) / n;
    }


    /// Computes a measure of how well the given map matches this map by computing the mean distance
    /// between the two elevation maps.
    double diff(const ElevationMap& map, double d_max = 1.0) const
    {
        d_max = std::abs(d_max);

        // Check if both maps have the same resolution.
        if (resolution_ != map.resolution_)
            ROS_ERROR("Elevation maps must have the same resolution to be comparable.");

        // Compute the total height distance between the maps.
        double d_total = 0.0;
        size_t n = 0u;
        for (size_t ix = 0u; ix < map_.size(); ++ix)
            for (size_t iy = 0u; iy < map_[0].size(); ++iy)
            {
                // Compute the coordinates of the center of the map tile.
                double x_center = x_min_ + (ix+0.5)*resolution_;
                double y_center = y_min_ + (iy+0.5)*resolution_;

                // Compute the height difference between the two map tiles.
                double d = elevation(x_center, y_center) - map.elevation(x_center, y_center);

                // If the height difference is defined, add it to the total difference.
                if (std::isfinite(d))
                {
                    d_total += std::min(std::abs(d), d_max);
                    n++;
                }
            }

        // Return the mean of the distances.
        if (n < 1)
            return 0.0;
        else
            return d_max - d_total / n;
    }


    /// Computes the mean distance in z direction between the elevation map and a given point cloud.
    double diff(const pcl::PointCloud<PointType>& pc) const
    {
        // Compute the total distance in z direction between the point cloud and the map.
        double d_total = 0.0;
        unsigned int n = 0u;
        size_t ix, iy;
        for (size_t i = 0u; i < pc.size(); ++i)
            if (tile(pc[i], ix, iy))
            {
                double dz = pc[i].z - map_[ix][iy];
                if (std::isfinite(dz))
                {
                    d_total += dz;
                    n++;
                }
            }

        return d_total / std::max(1u, n);
    }


    /// Computes the error between the given point cloud and the elevation map.
    double match(const pcl::PointCloud<PointType>& pc) const
    {
        // Compute the total distance in z-direction between the point cloud and the map.
        double d_total = 0.0;
        double dz;
        unsigned int n = 0u;
        for (size_t i = 0u; i < pc.size(); ++i)
        {
            // Determine distance between the current point and the map.
            size_t ix, iy;
            if (tile(pc[i], ix, iy))
            {
                if (std::isfinite(map_[ix][iy]))
                    dz = pc[i].z - map_[ix][iy];
                else
                    dz = pc[i].z;
            }
            else
                dz = pc[i].z;

            // If the distance is finite, add it to the total distance.
            if (std::isfinite(dz))
            {
                d_total += std::max(0.0, dz);
                n++;
            }
        }

        // Compute the mean distance.
        return d_total / n;
    }


    /// Computes a measure of how well the given map matches this map by computing the exponentials
    /// of the distance between the two elevation maps.
    double expdiff(const ElevationMap& map, double d_max = 1.0) const
    {
        // Check if both maps have the save resolution.
        if (resolution_ != map.resolution())
            ROS_ERROR("Elevation maps must have the same resolution to be compatible.");

        // Compute the sum of the exponentials of the height difference between both maps.
        double exp_d_total = 0.0;
        const double exp_d_max = std::exp(std::abs(d_max));
        size_t n = 0u;
        for (size_t ix = 0u; ix < map_.size(); ++ix)
            for (size_t iy = 0u; iy < map_[0].size(); ++iy)
            {
                // Compute the coordinates of the center of the map tile.
                double x_center = x_min_ + (ix+0.5)*resolution_;
                double y_center = y_min_ + (iy+0.5)*resolution_;

                // Compute the height difference between the two map tiles.
                double d = elevation(x_center, y_center) - map.elevation(x_center, y_center);

                // If the height difference is defined, compute the exponential and add it to the total difference.
                if (std::isfinite(d))
                {
                    exp_d_total += std::min(std::exp(std::abs(d)), exp_d_max);
                    n++;
                }
            }

        // Return the mean of the exponentials.
        if (n < 1)
            return 0.0;
        else
            return exp_d_max - exp_d_total / n;
    }


    /// Returns the resolution of the map.
    double resolution() const
    {
        return resolution_;
    }


    /// Saves the elevation map to a CSV file.
    void save(std::string filename = std::string()) const
    {
        // Define the filename.
        if (filename.empty())
        {
            ros::Time now(ros::Time::now());
            std::stringstream filename_stream;
            filename_stream << now.sec << now.nsec << ".csv";
            filename = filename_stream.str();
        }

        // Write the map to a comma-separated file.
        std::ofstream file;
        file.open(filename.c_str());
        for (size_t ix = 0; ix < map_.size(); ++ix)
            for (size_t iy = 0; iy < map_[0].size(); ++iy)
            {
                file << map_[ix][iy];
                if (iy < map_[ix].size()-1)
                    file << ",";
                else
                    file << std::endl;
            }
        file.close();

        ROS_DEBUG_STREAM("Saved \"" << filename << "\".");
    }



protected:
    /// Checks if the given map tile indices are valid.
    bool check(size_t ix, size_t iy) const
    {
        return 0 <= ix && ix < map_.size()
            && 0 <= iy && iy < map_[0].size();
    }

    /// Returns the index of the tile where the given point resides.
    /// If the point lies outside the map, this method returns \c false.
    bool tile(const PointType& point, size_t& ix, size_t& iy) const
    {
        return tile(point.x, point.y, ix, iy);
    }


    /// Returns the index of the tile where the point with the given coordinates resides.
    /// If the point lies outside the map, this method returns \c false.
    bool tile(double x, double y, size_t& ix, size_t& iy) const
    {
        // If the coordinates are infinite, exit immediately.
        if (!std::isfinite(x) || !std::isfinite(y))
            return false;

        // Compute the tile index that corresponds to the given coordinates.
        int ix_tmp = std::floor((x - x_min_) / resolution_);
        int iy_tmp = std::floor((y - y_min_) / resolution_);

        // If the coordinates lie within the map, return the corresponding map tile indices.
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


template<typename PointType> const double ElevationMap<PointType>::resolution_min = 0.001;


#endif
