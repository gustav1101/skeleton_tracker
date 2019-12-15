#include "SkeletonRepository.hpp"
#include <boost/algorithm/clamp.hpp>

using Skeleton = skeleton3d::Skeleton3d;
using BodyPart = skeleton3d::BodyPart3d;
using Point = geometry_msgs::Point;
template<class T> using vector = std::vector<T>;
using TimedBodyPart = repository_data_structures::TimedBodyPart;
using TimedSkeleton = repository_data_structures::TimedSkeleton;

void SkeletonRepository::update_skeletons(std::vector<TimedSkeleton> &observed_skeletons)
{
    vector<TimedSkeleton*> new_tracks = skeleton_matcher_.update_tracks_and_return_unmatched_observations(skeletons_masterlist_, observed_skeletons);
    add_to_masterlist(new_tracks);
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

void SkeletonRepository::add_to_masterlist(const vector<TimedSkeleton*>& new_tracks)
{
    for(const TimedSkeleton* new_track : new_tracks)
    {
        add_to_masterlist(*new_track);
    }
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
