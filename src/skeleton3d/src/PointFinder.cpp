#include "PointFinder.hpp"
#include <boost/algorithm/clamp.hpp>
#include <stdexcept>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Point3d = geometry_msgs::Point;

boost::optional<Point3d> PointFinder::find_best_point_around_coordinates(
    float relative_x, float relative_y)
{
    if ( image_max_x_ == 0 || image_max_y_ == 0 || !point_cloud_ )
    {
        throw std::runtime_error("Pointcloud not completly initialised!");
    }
   
    Point2d center_point_2d = absolute_coordinates(relative_x, relative_y);

    // Create group of possible points by scattering around center point.
    std::vector<Point3d> possible_points = create_possible_points(center_point_2d);
    if (possible_points.size() == 0 )
    {
        // Center point is included in possible points, so we can't return that either.
        return boost::none;
    }
    boost::optional<Point3d> center_point_3d = create_point3d(center_point_2d);
    if (!center_point_3d)
    {
        // Pick any point. The first one is the only one to be guaranteed to exist at this point
        center_point_3d = possible_points.at(0);
    }
    return find_best_point(possible_points, *center_point_3d);
}

std::vector<Point3d> PointFinder::create_possible_points(Point2d &center_point)
{
    std::vector<Point3d> scattered_points;
    // Create 9 points, 8 of which are arranged in a square around the center point, and center
    // point itself being the 9th.
    for (int i = 0; i < 9; i++)
    {
        /* Scatter according to this pattern:
         *
         * i=0 | i=3 | i=6
         * ----|-----|----
         * i=1 | i=4 | i=7
         * ----|-----|----
         * i=2 | i=5 | i=8
         */
        unsigned int x = center_point.x + ((int)i/3) * 2 * SCATTER_DISTANCE_ - SCATTER_DISTANCE_;
        unsigned int y = center_point.y + i % 3 * 2 * SCATTER_DISTANCE_ - SCATTER_DISTANCE_;
        // Make sure we don't go out of image bounds
        x = boost::algorithm::clamp(x, 0, image_max_x_);
        y = boost::algorithm::clamp(y, 0, image_max_y_);
        Point2d point2d{
            .x = x,
            .y = y
        };

        boost::optional<Point3d> point3d = create_point3d(point2d);
        if(point3d)
        {
            scattered_points.push_back(*point3d);
        }
    }
    return scattered_points;
}

Point3d PointFinder::find_best_point(const std::vector<Point3d> &possible_points, const Point3d &center_point)
{
    // Find the point with minimum z coordinate
    Point3d best_depth_point = *std::min_element(possible_points.begin(), possible_points.end(),
                     [] (Point3d const& point1, Point3d const& point2)
                     {
                         return point1.z < point2.z;
                     });
    
    Point3d best_point;
    if( best_depth_point.z + DEPTH_TOLERANCE_ < center_point.z )
    {
        best_point = Point3d(best_depth_point);
    } else
    {
        best_point = Point3d(center_point);
    }
    return best_point;
}

boost::optional<Point3d> PointFinder::create_point3d(Point2d &point2d)
{
    if(point2d.x > image_max_x_ || point2d.y > image_max_y_)
    {
        throw std::out_of_range("Point Cloud illegal coordinates queried");
    }
    pcl::PointXYZ point_in_pointcloud = point_cloud_->at(point2d.x, point2d.y);
    
    Point3d point3d;
    if (any_coordinate_invalid(point_in_pointcloud.x,
                               point_in_pointcloud.y,
                               point_in_pointcloud.z))
    {
        return boost::none;
    }
    point3d.x = point_in_pointcloud.x;
    point3d.y = point_in_pointcloud.y;
    point3d.z = point_in_pointcloud.z;
    return point3d;
}

PointFinder::Point2d PointFinder::absolute_coordinates(float &relative_x, float &relative_y)
{
    unsigned int x = relative_x * image_max_x_ + frame_offset_;
    unsigned int y = relative_y * image_max_y_;
    return {.x = x, .y = y};
}

inline bool PointFinder::any_coordinate_invalid(float x, float y, float z)
{
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
}

void PointFinder::set_window_boundaries(unsigned int image_max_x, unsigned int image_max_y)
{
    image_max_x_ = image_max_x-1;
    image_max_y_ = image_max_y-1;
}

void PointFinder::set_point_cloud(const PointCloud::ConstPtr &point_cloud)
{
    point_cloud_ = point_cloud;
}
