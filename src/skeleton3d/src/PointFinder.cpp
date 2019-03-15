#include "PointFinder.h"
#include <boost/algorithm/clamp.hpp>
#include <stdexcept>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

boost::optional<geometry_msgs::Point> PointFinder::find_best_point_around_coordinates(
    float relative_x, float relative_y)
{
    if ( image_max_x_ == 0 || image_max_y_ == 0 || !point_cloud_ )
    {
        throw std::runtime_error("Pointcloud not completly initialised!");
    }
    
    unsigned int x = relative_x * image_max_x_ + frame_offset_;
    unsigned int y = relative_y * image_max_y_;
    Point2d center_point_2d{.x = x, .y = y};
    
    std::vector<geometry_msgs::Point> scattered_points = create_scattered_points(center_point_2d);
    if (scattered_points.size() == 0 )
    {
        //ROS_WARN("Could not find points around designated coordinates: x=%i; y=%i", x,y);
        return boost::none;
    }
    boost::optional<geometry_msgs::Point> center_point_3d = create_Point3d(center_point_2d);
    if (!center_point_3d)
    {
        center_point_3d = scattered_points.at(0);
    }
    return find_best_point(scattered_points, *center_point_3d);
}

std::vector<geometry_msgs::Point> PointFinder::create_scattered_points(Point2d &center_point)
{
    std::vector<geometry_msgs::Point> scattered_points;
    for (int i = 0; i < 9; i++)
    {
        unsigned int x = center_point.x + ((int)i/3) * 2 * SCATTER_DISTANCE_ - SCATTER_DISTANCE_;
        unsigned int y = center_point.y + i % 3 * 2 * SCATTER_DISTANCE_ - SCATTER_DISTANCE_;
        x = boost::algorithm::clamp(x, 0, image_max_x_);
        y = boost::algorithm::clamp(y, 0, image_max_y_);
        Point2d point2d{
            .x = x,
            .y = y
        };

        boost::optional<geometry_msgs::Point> point3d = create_Point3d(point2d);
        if(point3d)
        {
            scattered_points.push_back(*point3d);
        }
    }

    return scattered_points;
}

geometry_msgs::Point PointFinder::find_best_point(const std::vector<geometry_msgs::Point> &possible_points, const geometry_msgs::Point &center_point)
{
    geometry_msgs::Point best_depth_point = *std::min_element(possible_points.begin(), possible_points.end(),
                     [] (geometry_msgs::Point const& point1, geometry_msgs::Point const& point2)
                     {
                         return point1.z < point2.z;
                     });
    
    geometry_msgs::Point best_point;
    if( best_depth_point.z + DEPTH_TOLERANCE_ < center_point.z )
    {
        best_point = geometry_msgs::Point(best_depth_point);
    } else
    {
        best_point = geometry_msgs::Point(center_point);
    }
    return best_point;
}

boost::optional<geometry_msgs::Point> PointFinder::create_Point3d(Point2d &point2d)
{
    if(point2d.x > image_max_x_ || point2d.y > image_max_y_)
    {
        throw std::out_of_range("Point Cloud illegal coordinates queried");
    }
    pcl::PointXYZ point_in_pointcloud = point_cloud_->at(point2d.x, point2d.y);
    
    geometry_msgs::Point point3d;
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
