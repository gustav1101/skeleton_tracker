#ifndef SKELETONCREATOR_HPP
#define SKELETONCREATOR_HPP

#include <geometry_msgs/Point.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/Persons.h>
#include <skeleton3d/Skeleton3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>
#include "PointFinder.hpp"

class SkeletonCreator
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

public:
    SkeletonCreator(int scatter_distance, double frame_offset) : point_finder_(scatter_distance, frame_offset) {}
    ~SkeletonCreator();
    std::vector<skeleton3d::Skeleton3d> generate_skeleton(
        const std::vector<tfpose_ros::Person> &persons,
        const PointCloud::ConstPtr &point_cloud);
    void set_image_size(unsigned int width, unsigned int height);

private:
    PointFinder point_finder_;

    boost::optional<skeleton3d::Skeleton3d> transform_skeleton_to_3d(
        const tfpose_ros::Person &person);
    bool any_coordinate_invalid(float x, float y, float z);
    geometry_msgs::Point get_skeleton_center(skeleton3d::Skeleton3d &skeleton);
};

#endif
