#include "SkeletonRepository.hpp"
#include <math.h>
#include <boost/range/combine.hpp>
#include <boost/algorithm/clamp.hpp>
#include <tf/transform_listener.h>

using Skeleton = skeleton3d::Skeleton3d;
using BodyPart = skeleton3d::BodyPart3d;
using Point = geometry_msgs::Point;
template<class T> using optional = boost::optional<T>;
using TimedBodyPart = repository_data_structures::TimedBodyPart;
using TimedSkeleton = repository_data_structures::TimedSkeleton;

void SkeletonRepository::update_skeletons(const std::vector<TimedSkeleton> &timed_skeletons)
{
    exclude_list_.clear();
    for (const TimedSkeleton &new_skeleton : timed_skeletons)
    {
        optional<TimedSkeleton&> existing_skeleton;
        try {
             existing_skeleton = find_skeleton_in_list(new_skeleton);
        } catch (const NoCenterpointFoundError&)
        {
            ROS_WARN("Found skeleton without valid center parts, skipping");
            continue;
        }
        if(existing_skeleton)
        {
            merge_skeleton(new_skeleton, *existing_skeleton);
        } else
        {
            add_to_masterlist(new_skeleton);
        }
    }
}

std::vector<Skeleton> SkeletonRepository::get_skeleton_masterlist()
{
    decay_masterlist();

    std::vector<Skeleton> simple_skeletons;
    for (const TimedSkeleton &timed_skeleton : skeletons_masterlist_)
    {
        simple_skeletons.push_back(simple_skeleton_from(timed_skeleton));
    }
    return simple_skeletons;
}

optional<SkeletonRepository::TimedSkeleton&> SkeletonRepository::find_skeleton_in_list(const TimedSkeleton &new_skeleton)
{
    auto existing_skeleton_iterator = std::find_if(
        skeletons_masterlist_.begin(),
        skeletons_masterlist_.end(),
        [&new_skeleton, this](const TimedSkeleton &skeleton_in_list)
        {
            return !from_same_message(skeleton_in_list) && is_same_skeleton(new_skeleton, skeleton_in_list);
        });
    if (existing_skeleton_iterator != skeletons_masterlist_.end())
    {
        return *existing_skeleton_iterator;
    } else
    {
        return boost::none;
    }
}

bool SkeletonRepository::is_same_skeleton(const TimedSkeleton &skel1, const TimedSkeleton &skel2)
{
    Point skel1_center = get_skeleton_center_position(skel1.timed_body_parts);
    Point skel2_center = get_skeleton_center_position(skel2.timed_body_parts);

    if (distance_between_points(skel1_center, skel2_center) > POSITION_TOLERANCE_)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool SkeletonRepository::from_same_message(const TimedSkeleton &skeleton)
{
    auto it = find(exclude_list_.begin(), exclude_list_.end(), &skeleton);
    return it != exclude_list_.end();
}

Point SkeletonRepository::get_skeleton_center_position(const std::vector<TimedBodyPart> &body_parts)
{
    // Use only center head and center torso points for calculation. More robust when Skeleton
    // is leaving the frame.
    std::vector<const TimedBodyPart*> interesting_parts;
    interesting_parts.push_back(&body_parts.at(0));
    interesting_parts.push_back(&body_parts.at(1));
    
    return calculate_mean_position(interesting_parts);
}

Point SkeletonRepository::calculate_mean_position(const std::vector<const TimedBodyPart*> body_parts)
{
    Point centerpoint;
    centerpoint.x = 0;
    centerpoint.y = 0;
    centerpoint.z = 0;
    int valid_point_counter = 0;
    for(const TimedBodyPart * const timed_body_part : body_parts)
    {
        const BodyPart &body_part = timed_body_part->body_part;
        if ( body_part.part_is_valid )
        {
            centerpoint.x += body_part.point.x;
            centerpoint.y += body_part.point.y;
            centerpoint.z += body_part.point.z;
            valid_point_counter++;
        }
    }
    if (valid_point_counter == 0)
    {
        throw NoCenterpointFoundError();
    }
    centerpoint.x /= valid_point_counter;
    centerpoint.y /= valid_point_counter;
    centerpoint.z /= valid_point_counter;
    return centerpoint;
}

double SkeletonRepository::distance_between_points(const Point &point1, const Point &point2)
{
	// Taxicap metric is sufficient here since we'll be comparing the result
	// with an arbitrary number anyway.
    return fabs(point1.x-point2.x) + fabs(point1.y-point2.y) + fabs(point1.z - point2.z);
    // Should you ever want the euclidian metric:
    //sqrt(pow(point1.x-point2.x, 2) + pow(point1.y-point2.y, 2) + pow(point1.z - point2.z, 2));
}

void SkeletonRepository::merge_skeleton(const TimedSkeleton &new_skeleton, TimedSkeleton &existing_skeleton)
{
    for (int i = 0; i < 18; i++)
    {
        const TimedBodyPart &new_body_part = new_skeleton.timed_body_parts.at(i);
        TimedBodyPart &existing_body_part = existing_skeleton.timed_body_parts.at(i);
        if (should_update(new_body_part, existing_body_part))
        {
            existing_body_part = new_body_part;
        }
    }
    exclude_list_.push_back(&existing_skeleton);
}

inline bool SkeletonRepository::should_update(const TimedBodyPart &new_timed_body_part, const TimedBodyPart &existing_timed_body_part)
{
    const BodyPart &new_body_part = new_timed_body_part.body_part;
    const BodyPart &existing_body_part = existing_timed_body_part.body_part;
    // Update the body part only if the new part is valid AND the old part is either invalid
    // or has a lower confidence.
    return (new_body_part.part_is_valid && !existing_body_part.part_is_valid) ||
        (new_body_part.confidence > existing_body_part.confidence);
}

void SkeletonRepository::add_to_masterlist(const TimedSkeleton &new_skeleton)
{
    skeletons_masterlist_.push_back(new_skeleton);
    skeletons_masterlist_.back().id = skeleton_id_++;
}

Skeleton SkeletonRepository::simple_skeleton_from(const TimedSkeleton &timed_skeleton)
{
    Skeleton skeleton;
    for (const TimedBodyPart &timed_body_part : timed_skeleton.timed_body_parts)
    {
        skeleton.body_parts.push_back(timed_body_part.body_part);
    }
    return skeleton;
}

void SkeletonRepository::decay_masterlist()
{
    if (DECAY_STRENGTH_ == 0.0)
    {
        return;
    }
    for (auto masterlist_iter = skeletons_masterlist_.begin(); masterlist_iter != skeletons_masterlist_.end(); masterlist_iter++)
    {
        if (decay_skeleton(*masterlist_iter))
        {
            auto to_delete_iter = masterlist_iter;
            bool all_deleted = false;
            if (masterlist_iter != skeletons_masterlist_.begin() )
            {
                masterlist_iter--;
            } else
            {
                all_deleted = true;
            }
            skeletons_masterlist_.erase(to_delete_iter);
            if (all_deleted)
            {
                return;
            }
        }
    }
}

bool SkeletonRepository::decay_skeleton(TimedSkeleton &timed_skeleton)
{
    bool remove_skeleton = true;
    for (TimedBodyPart &timed_body_part : timed_skeleton.timed_body_parts)
    {
        remove_skeleton &= decay_bodypart(timed_body_part);
    }
    return remove_skeleton;
}

bool SkeletonRepository::decay_bodypart(TimedBodyPart &timed_body_part)
{
    BodyPart &body_part = timed_body_part.body_part;

    if (!body_part.part_is_valid)
    {
        return true;
    }
    
    double time_since_message = boost::algorithm::clamp(
        (ros::Time::now() - timed_body_part.timestamp ).toSec(), 0.0, DBL_MAX );
    body_part.confidence -= time_since_message * DECAY_STRENGTH_;

    // Check if body part has decayed so much that it should be removed
    if(body_part.confidence <= 0.0)
    {
        timed_body_part.body_part.part_is_valid = false;
        return true;
    } else
    {
        return false;
    }
}
