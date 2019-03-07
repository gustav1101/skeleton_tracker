#include <ros/ros.h>
#include "SkeletonRepository.h"
#include <math.h>
#include <boost/range/combine.hpp>

using Skeleton = skeleton3d::Skeleton3d;
using Point = geometry_msgs::Point;
template<class T> using optional = boost::optional<T>;


SkeletonRepository::~SkeletonRepository() { }

void SkeletonRepository::update_skeletons(const std::vector<Skeleton> &skeletons)
{
    ROS_INFO("AM ALIVE");
    for (const Skeleton &skeleton : skeletons)
    {
        optional<Skeleton&> existing_skeleton = find_skeleton_in_list(skeleton);
        if(existing_skeleton)
        {
            merge_skeleton(skeleton, *existing_skeleton);
            ROS_INFO("MERGED");
        } else
        {
            insert_skeleton(skeleton);
            ROS_INFO("INSERTED");
        }
    }
}

std::vector<Skeleton> SkeletonRepository::get_skeleton_masterlist()
{
    ROS_INFO("RETRIEVING MASTERLIST");
    return skeletons_masterlist_;
}

optional<Skeleton&> SkeletonRepository::find_skeleton_in_list(const Skeleton &new_skeleton)
{
    auto existing_skeleton_iterator = std::find_if(
        skeletons_masterlist_.begin(),
        skeletons_masterlist_.end(),
        [&new_skeleton, this](const Skeleton &skeleton_in_list)
        {
            return is_same_skeleton(new_skeleton, skeleton_in_list); 
        });
    if (existing_skeleton_iterator != skeletons_masterlist_.end())
    {
        return *existing_skeleton_iterator;
    } else
    {
        return boost::none;
    }
}

bool SkeletonRepository::is_same_skeleton(const Skeleton &skel1, const Skeleton &skel2)
{
    Point skel1_center = get_skeleton_center_position(skel1.body_parts);
    Point skel2_center = get_skeleton_center_position(skel2.body_parts);

    if (distance_between_points(skel1_center, skel2_center) > POSITION_TOLERANCE_)
    {
        return false;
    }
    else {
        return true;
    }
}

inline double SkeletonRepository::distance_between_points(const Point &point1, const Point &point2)
{
	// Taxicap metric is sufficient here since we'll be comparing the result
	// with an arbitrary number anyway.
    return abs(point1.x-point2.x) + abs(point1.y-point2.y) + abs(point1.z - point2.z); //sqrt(pow(point1.x-point2.x, 2) + pow(point1.y-point2.y, 2) + pow(point1.z - point2.z, 2));
}

Point SkeletonRepository::get_skeleton_center_position(const std::vector<BodyPart> &body_parts)
{
    Point center;
    center.x = 0;
    center.y = 0;
    center.z = 0;
    for(const BodyPart &body_part : body_parts)
    {
        center.x += body_part.point.x;
        center.y += body_part.point.y;
        center.z += body_part.point.z;
    }
    center.x /= body_parts.size();
    center.y /= body_parts.size();
    center.z /= body_parts.size();
    return center;
}

void SkeletonRepository::merge_skeleton(const Skeleton &new_skeleton, Skeleton &existing_skeleton)
{
    if (new_skeleton.body_parts.size() != existing_skeleton.body_parts.size())
    {
        // Should this ever happen something is seriously wrong with the skeleton creator
        // implementation, as all body part lists should always have same size
        throw std::runtime_error("Skeleton Body Part Lists have unequal size.");
    }
    for (auto both_body_parts : boost::combine(new_skeleton.body_parts, existing_skeleton.body_parts))
    {
        BodyPart body_part_new;
        BodyPart body_part_existing;
        boost::tie(body_part_new, body_part_existing) = both_body_parts;
        if (should_update(body_part_new, body_part_existing))
        {
            body_part_existing = body_part_new;
        }
    }
}

inline bool SkeletonRepository::should_update(const BodyPart &body_part_new, const BodyPart &body_part_existing)
{
    return body_part_new.confidence > body_part_existing.confidence + 0.05;
}


void SkeletonRepository::insert_skeleton(const Skeleton &new_skeleton)
{
    skeletons_masterlist_.push_back(new_skeleton);
}

