#include "SkeletonRepository.h"
#include <math.h>
#include <boost/range/combine.hpp>
#include <boost/algorithm/clamp.hpp>

using Skeleton = skeleton3d::Skeleton3d;
using BodyPart = skeleton3d::BodyPart3d;
using Point = geometry_msgs::Point;
template<class T> using optional = boost::optional<T>;


SkeletonRepository::~SkeletonRepository() { }

void SkeletonRepository::update_skeletons(const skeleton3d::Skeletons3d::ConstPtr &skeletons_msg)
{
    const std::vector<Skeleton> &skeletons = skeletons_msg->skeletons;
    const ros::Time &timestamp = skeletons_msg->header.stamp;
    for (const Skeleton &skeleton : skeletons)
    {
        optional<SkeletonInformation&> existing_skeleton = find_skeleton_in_list(skeleton);
        if(existing_skeleton)
        {
            merge_skeleton(skeleton, *existing_skeleton, timestamp);
        } else
        {
            insert_skeleton(skeleton, timestamp);
        }
    }
}

std::vector<Skeleton> SkeletonRepository::get_skeleton_masterlist()
{
    decay_masterlist();
    std::vector<Skeleton> skeletons;
    for (const SkeletonInformation &skeleton_info : skeletons_masterlist_)
    {
        skeletons.push_back(simple_skeleton_from_skeletoninfo(skeleton_info));
    }
    return skeletons;
}

Skeleton SkeletonRepository::simple_skeleton_from_skeletoninfo(const SkeletonInformation &skeleton_info)
{
    Skeleton skeleton;
    for (const BodyPartInformation &body_part_info : skeleton_info.body_part_information)
    {
        skeleton.body_parts.push_back(body_part_info.body_part);
    }
    return skeleton;
}

optional<SkeletonRepository::SkeletonInformation&> SkeletonRepository::find_skeleton_in_list(const Skeleton &new_skeleton)
{
    auto existing_skeleton_iterator = std::find_if(
        skeletons_masterlist_.begin(),
        skeletons_masterlist_.end(),
        [&new_skeleton, this](const SkeletonInformation &skeleton_in_list)
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

bool SkeletonRepository::is_same_skeleton(const Skeleton &skel1, const SkeletonInformation &skel2)
{
    Point skel1_center = get_skeleton_center_position(skel1.body_parts);
    Point skel2_center = get_skeleton_center_position(skel2.body_part_information);

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
    return fabs(point1.x-point2.x) + fabs(point1.y-point2.y) + fabs(point1.z - point2.z); //sqrt(pow(point1.x-point2.x, 2) + pow(point1.y-point2.y, 2) + pow(point1.z - point2.z, 2));
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

Point SkeletonRepository::get_skeleton_center_position(const std::vector<BodyPartInformation> &body_parts)
{
    Point center;
    center.x = 0;
    center.y = 0;
    center.z = 0;
    for(const BodyPartInformation &body_part_info : body_parts)
    {
        center.x += body_part_info.body_part.point.x;
        center.y += body_part_info.body_part.point.y;
        center.z += body_part_info.body_part.point.z;
    }
    center.x /= body_parts.size();
    center.y /= body_parts.size();
    center.z /= body_parts.size();
    return center;
}

void SkeletonRepository::merge_skeleton(const Skeleton &new_skeleton, SkeletonInformation &existing_skeleton, const ros::Time &timestamp)
{
    if (new_skeleton.body_parts.size() != existing_skeleton.body_part_information.size())
    {
        // Should this ever happen something is seriously wrong with the skeleton creator
        // implementation, as all body part lists should always have same size
        throw std::runtime_error("Skeleton Body Part Lists have unequal size.");
    }
    for (const BodyPart &new_skeleton_body_part : new_skeleton.body_parts)
    {
        BodyPartInformation new_body_part_info {.body_part = new_skeleton_body_part, .timestamp = timestamp};
        const unsigned int body_part_id = new_skeleton_body_part.part_id;
        BodyPartInformation &old_body_part_info = existing_skeleton.body_part_information.at(body_part_id);
        if (should_update(new_body_part_info, old_body_part_info))
        {
            old_body_part_info = new_body_part_info;
        }
    }
}

inline bool SkeletonRepository::should_update(const BodyPartInformation &body_part_new_info, const BodyPartInformation &body_part_existing_info)
{
    const BodyPart &body_part_new = body_part_new_info.body_part;
    const BodyPart &body_part_old = body_part_existing_info.body_part;
    // Update the body part only if the new part is valid AND the old part is either invalid
    // or has a lower confidence.
    return body_part_new.part_is_valid && ( !body_part_old.part_is_valid) ||
        (body_part_new.confidence > body_part_old.confidence);
}

void SkeletonRepository::insert_skeleton(const Skeleton &new_skeleton, const ros::Time &timestamp)
{
    SkeletonInformation skeleton_info;
    for(const BodyPart &body_part : new_skeleton.body_parts)
    {
        BodyPartInformation body_part_info{ .body_part = body_part, .timestamp = timestamp };
        skeleton_info.body_part_information.push_back(body_part_info);
    }
    skeletons_masterlist_.push_back(skeleton_info);
}

void SkeletonRepository::decay_masterlist()
{
    if (DECAY_STRENGTH_ == 0.0)
    {
        return;
    }
    for (auto masterlist_iter = skeletons_masterlist_.begin(); masterlist_iter != skeletons_masterlist_.end(); masterlist_iter++) //SkeletonInformation &skeleton_info : skeletons_masterlist_)
    {
        if (decay_skeleton(*masterlist_iter))
        {
            auto to_delete_iter = masterlist_iter;
            masterlist_iter--;
            skeletons_masterlist_.erase(to_delete_iter);
        }
    }
}

// Return true iff the Skeleton has only invalid bodyparts and thus should be removed
bool SkeletonRepository::decay_skeleton(SkeletonInformation &skeleton_information)
{
    bool remove_skeleton = true;
    for (BodyPartInformation &body_part_info : skeleton_information.body_part_information)
    {
        remove_skeleton &= decay_bodypart(body_part_info);
    }
    return remove_skeleton;
}

// Return True iff part has decayed to the point of being invalid (confidence <= 0 )
bool SkeletonRepository::decay_bodypart(BodyPartInformation &body_part_info)
{
    BodyPart &body_part = body_part_info.body_part;
    
    double time_since_message = boost::algorithm::clamp( (ros::Time::now() - body_part_info.timestamp ).toSec(), 0.0, DBL_MAX );
    body_part.confidence -= time_since_message * DECAY_STRENGTH_;

    // Check if body part has decayed so much that it should be removed
    if(body_part.confidence <= 0.0)
    {
        body_part_info.body_part.part_is_valid = false;
        return true;
    } else
    {
        return false;
    }
}
