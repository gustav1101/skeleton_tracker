#include "SkeletonCreator.h"
#include <tfpose_ros/BodyPartElm.h>
#include <skeleton3d/BodyPart3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include "PointFinder.h"

SkeletonCreator::~SkeletonCreator()
{

}

std::vector<skeleton3d::Skeleton3d> SkeletonCreator::generate_skeleton(const std::vector<tfpose_ros::Person> &persons, const PointCloud::ConstPtr &point_cloud)
{
    std::vector<skeleton3d::Skeleton3d> skeletons_3d;
    // For each skeleton created by openpose: transform into 3d skeleton
    for(const tfpose_ros::Person &person : persons)
    {
        boost::optional<skeleton3d::Skeleton3d> skeleton = transform_skeleton_to_3d(
            person,
            point_cloud);
        if (skeleton)
        {
            skeletons_3d.push_back(*skeleton);
        }
    }
    return skeletons_3d;
}

boost::optional<skeleton3d::Skeleton3d> SkeletonCreator::transform_skeleton_to_3d(
    const tfpose_ros::Person &person,
    const PointCloud::ConstPtr &point_cloud)
{
    skeleton3d::Skeleton3d skeleton;
    skeleton.body_parts = std::vector<skeleton3d::BodyPart3d>(18);

    // First set all body parts to invalid (meaning not found), then overwrite with
    // data from the actual tfpose body parts
    for(skeleton3d::BodyPart3d &body_part : skeleton.body_parts)
    {
        body_part.part_is_valid = false;
    }

    PointFinder point_finder(image_width_, image_height_, point_cloud, point_finder_scatter_distance_, frame_offset_);
    for(const tfpose_ros::BodyPartElm &body_part_2d : person.body_part)
    {
        skeleton3d::BodyPart3d &body_part_3d = skeleton.body_parts.at(body_part_2d.part_id);
        body_part_3d.part_is_valid = true;
        body_part_3d.part_id = body_part_2d.part_id;

        boost::optional<geometry_msgs::Point> body_part_point =
            point_finder.find_best_point_around_coordinates(body_part_2d.x, body_part_2d.y);
        if( !body_part_point )
        {
            continue;
        }
        body_part_3d.point = *body_part_point;
        body_part_3d.confidence = body_part_2d.confidence;
    }
    return skeleton;
}

void SkeletonCreator::set_image_size(unsigned int width, unsigned int height)
{
    image_height_ = height;
    image_width_ = width;
}
