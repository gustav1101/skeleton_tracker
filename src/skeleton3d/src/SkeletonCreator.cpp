#include "SkeletonCreator.hpp"
#include <tfpose_ros/BodyPartElm.h>
#include <skeleton3d/BodyPart3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

SkeletonCreator::~SkeletonCreator()
{

}

std::vector<skeleton3d::Skeleton3d> SkeletonCreator::generate_skeletons(const std::vector<tfpose_ros::Person> &persons, const PointCloud::ConstPtr &point_cloud)
{
    point_finder_.set_point_cloud(point_cloud);
    // For each skeleton created by openpose: transform into 3d skeleton
    std::vector<skeleton3d::Skeleton3d> skeletons_3d;
    for(const tfpose_ros::Person &person : persons)
    {
        boost::optional<skeleton3d::Skeleton3d> skeleton = transform_skeleton_to_3d(
            person);
        if (skeleton)
        {
            skeletons_3d.push_back(*skeleton);
        }
    }
    return skeletons_3d;
}

void SkeletonCreator::set_image_size(unsigned int width, unsigned int height)
{
    point_finder_.set_window_boundaries(width, height);
}

boost::optional<skeleton3d::Skeleton3d> SkeletonCreator::transform_skeleton_to_3d(
    const tfpose_ros::Person &person)
{
    skeleton3d::Skeleton3d skeleton;
    // All skeletons have 18 body parts
    skeleton.body_parts = std::vector<skeleton3d::BodyPart3d>(18);

    // First set all body parts to invalid (meaning not found), then overwrite with
    // data from the actual tfpose body parts
    for(skeleton3d::BodyPart3d &body_part : skeleton.body_parts)
    {
        body_part.part_is_valid = false;
    }

    bool any_body_part_valid = false;
    for(const tfpose_ros::BodyPartElm &body_part_2d : person.body_part)
    {
        skeleton3d::BodyPart3d &body_part_3d = skeleton.body_parts.at(body_part_2d.part_id);
        body_part_3d.part_id = body_part_2d.part_id;

        boost::optional<geometry_msgs::Point> body_part_point =
            point_finder_.find_best_point_around_coordinates(body_part_2d.x, body_part_2d.y);
        if( !body_part_point )
        {
            continue;
        }
        body_part_3d.part_is_valid = true;
        body_part_3d.point = *body_part_point;
        body_part_3d.confidence = body_part_2d.confidence;
        any_body_part_valid = true;
    }
    
    if (any_body_part_valid)
    {
        return skeleton;
    } else
    {
        return boost::none;
    }
}
