#include "SkeletonMerger.hpp"

#include "RepositoryDataStructures.hpp"
#include <skeleton3d/BodyPart3d.h>

using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;

namespace SkeletonMerger {
    void merge_body_part(const TimedBodyPart& observed_bodypart,
                         TimedBodyPart& tracked_bodypart);
    bool should_merge(const TimedBodyPart& old_part, const TimedBodyPart& new_part);
    bool part_is_valid(const TimedBodyPart& body_part);
    bool new_part_has_higher_confidence(const TimedBodyPart& old_part,
                                        const TimedBodyPart& new_part);
}

void SkeletonMerger::merge_skeleton(
    const TimedSkeleton *const observed_skeleton,
    TimedSkeleton &tracked_skeleton)
{
    for(int i = 0; i < 18; i++)
    {
        merge_body_part(
            observed_skeleton->timed_body_parts.at(i),
            tracked_skeleton.timed_body_parts.at(i));
    }
}

void SkeletonMerger::merge_body_part(
    const TimedBodyPart &observed_bodypart,
    TimedBodyPart &tracked_bodypart)
{
    if( should_merge(tracked_bodypart, observed_bodypart) )
    {
        tracked_bodypart = observed_bodypart;
    }
}

bool SkeletonMerger::should_merge(const TimedBodyPart &old_part, const TimedBodyPart &new_part)
{
    // Yes, this function could be reduced to one line. No, that wouldn't help readability.
    if( part_is_valid(old_part) )
    {
        if( new_part_has_higher_confidence(old_part, new_part) ||
            !part_is_valid(old_part))
        {
            return true;
        }
    }
    return false;
}

bool SkeletonMerger::part_is_valid(const TimedBodyPart &body_part)
{
    return body_part.body_part.part_is_valid;
}

bool SkeletonMerger::new_part_has_higher_confidence(const TimedBodyPart &old_part,
                                                    const TimedBodyPart &new_part)
{
    return new_part.body_part.confidence > old_part.body_part.confidence;
}
