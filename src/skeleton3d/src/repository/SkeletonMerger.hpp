#ifndef SKELETONMERGER_HPP
#define SKELETONMERGER_HPP

#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include "RepositoryDataStructures.hpp"

class SkeletonMerger {
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;
    template<class T> using vector = std::vector<T>;

public:
    static void merge_skeleton(const TimedSkeleton * const observed_skeleton,
                        TimedSkeleton& tracked_skeleton);
private:
    static void merge_body_part(const TimedBodyPart& observed_bodypart,
                                TimedBodyPart& tracked_bodypart);
    static bool part_is_valid(const TimedBodyPart& body_part);
};

#endif
