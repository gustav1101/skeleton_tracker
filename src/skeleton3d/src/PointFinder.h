#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointFinder
{
private:
    struct Point2d {
        unsigned int x;
        unsigned int y;
    };

    unsigned int image_max_x_;
    unsigned int image_max_y_;
    const PointCloud::ConstPtr point_cloud_;
    const int SCATTER_DISTANCE_;

    Point2d absolute_coordinates(float &relative_x, float &relative_y);
    std::vector<geometry_msgs::Point> create_scattered_points(
        Point2d &center_point);
    boost::optional<geometry_msgs::Point> create_Point3d(
        Point2d &point2d);
    geometry_msgs::Point find_best_point(const std::vector<geometry_msgs::Point> &possible_points, const geometry_msgs::Point &center_point);
    inline bool any_coordinate_invalid(float x, float y, float z);
public:
PointFinder(
    unsigned int image_max_x, unsigned int image_max_y,
    const PointCloud::ConstPtr &point_cloud,
    int scatter_distance) : image_max_x_(image_max_x), image_max_y_(image_max_y), point_cloud_(point_cloud), SCATTER_DISTANCE_(scatter_distance) { };
    ~PointFinder() { };

    boost::optional<geometry_msgs::Point> find_best_point_around_coordinates(
        float relative_x, float relative_y);
};
