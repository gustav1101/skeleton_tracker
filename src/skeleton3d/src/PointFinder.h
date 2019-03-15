#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>

class PointFinder
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

public:
    PointFinder(
        int scatter_distance, double frame_offset) :
        SCATTER_DISTANCE_(scatter_distance), frame_offset_(frame_offset){ };
    ~PointFinder() { };

    void set_window_boundaries(unsigned int image_max_x, unsigned int image_max_y);
    void set_point_cloud(const PointCloud::ConstPtr &point_cloud);
    boost::optional<geometry_msgs::Point> find_best_point_around_coordinates(
        float relative_x, float relative_y);

private:
    struct Point2d {
        unsigned int x;
        unsigned int y;
    };

    unsigned int image_max_x_;
    unsigned int image_max_y_;
    PointCloud::ConstPtr point_cloud_;
    const int SCATTER_DISTANCE_;
    const double frame_offset_;

    Point2d absolute_coordinates(float &relative_x, float &relative_y);
    std::vector<geometry_msgs::Point> create_scattered_points(
        Point2d &center_point);
    boost::optional<geometry_msgs::Point> create_Point3d(
        Point2d &point2d);
    geometry_msgs::Point find_best_point(const std::vector<geometry_msgs::Point> &possible_points, const geometry_msgs::Point &center_point);
    inline bool any_coordinate_invalid(float x, float y, float z);
};
