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
    unsigned int x_min, x_max, y_min, y_max;
    setup_scatter_bounds(center_point, x_min, x_max, y_min, y_max);
    for (unsigned int x = x_min; x <= x_max; x += SCATTER_STEP_DISTANCE_)
    {
        for(unsigned int y = y_min; y <= y_max; y += SCATTER_STEP_DISTANCE_)
        {
            Point2d point2d{
                .x = x,
                .y = y,
            };

            boost::optional<Point3d> point3d = create_point3d(point2d);
            if(point3d)
            {
                scattered_points.push_back(*point3d);
            }
        }
    }
    return scattered_points;
}

void PointFinder::setup_scatter_bounds(const Point2d &center_point,
                                       unsigned int &x_min, unsigned int &x_max,
                                       unsigned int &y_min, unsigned int &y_max)
{
    unsigned int total_distance = SCATTER_STEPS_ * SCATTER_STEP_DISTANCE_;
    x_min = center_point.x <= total_distance ? 0 : center_point.x - total_distance;
    x_max = center_point.x + total_distance > image_max_x_ ?
        image_max_x_ : center_point.x + total_distance;
    y_min = center_point.y <= total_distance ? 0 : center_point.y - total_distance;
    y_max = center_point.y + total_distance > image_max_y_ ?
        image_max_y_ : center_point.y + total_distance;
}

Point3d PointFinder::find_best_point(const std::vector<Point3d> &possible_points, const Point3d &center_point)
{
    using PointValuePair = std::pair<const Point3d&, double>;
    double median_z = find_median_z(possible_points);
    std::vector<PointValuePair> point_heuristical_values;
    for (const Point3d &point : possible_points)
    {
        PointValuePair pair(point, point_value(point, median_z, center_point));
        point_heuristical_values.push_back(pair);
    }
    PointValuePair best_depth_point =
        *std::min_element(point_heuristical_values.begin(), point_heuristical_values.end(),
                          [] (PointValuePair const& pair1, PointValuePair const& pair2)
                          {
                              return pair1.second < pair2.second;
                          });
    return best_depth_point.first;
}

double PointFinder::point_value(const Point3d &point,
                                const double &median_z_value,
                                const Point3d &center_point)
{
    return distance_from_center_point(point, center_point) +
        distance_from_median_z(point, median_z_value);
}

double PointFinder::distance_from_median_z(const Point3d &point, const double &median_z_value)
{
    return std::fabs(point.z - median_z_value);
}

double PointFinder::distance_from_center_point(const Point3d &point, const Point3d &center_point)
{
    return std::fabs(point.x-center_point.x) + std::fabs(point.y - center_point.y);
}

double PointFinder::find_median_z(const std::vector<Point3d> &points)
{
    size_t median_index = points.size() / 2;
    std::vector<double> z_values(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        z_values.at(i) = points.at(i).z;
    }
    std::nth_element(z_values.begin(), z_values.begin()+median_index, z_values.end());
    return z_values.at(median_index);
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

void PointFinder::set_point_cloud(const PointCloud &point_cloud)
{
    point_cloud_ = &point_cloud;
}
