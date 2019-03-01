#include <geometry_msgs/Point.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/Persons.h>
#include <skeleton3d/Skeleton3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class SkeletonCreator
{
private:
    const int point_finder_scatter_distance_ = 8;
    unsigned int image_height_;
    unsigned int image_width_;

    void save_skeletons(const tfpose_ros::Persons& persons_msg);
    boost::optional<skeleton3d::Skeleton3d> transform_skeleton_to_3d(
        const tfpose_ros::Person &person,
        const PointCloud::ConstPtr &point_cloud);
    bool any_coordinate_invalid(float x, float y, float z);
    
public:
SkeletonCreator(int scatter_distance) : point_finder_scatter_distance_(scatter_distance) {}
    ~SkeletonCreator();
    std::vector<skeleton3d::Skeleton3d> generate_skeleton(
        const std::vector<tfpose_ros::Person> &persons,
        const PointCloud::ConstPtr &point_cloud);
    void set_image_size(unsigned int width, unsigned int height);
};
