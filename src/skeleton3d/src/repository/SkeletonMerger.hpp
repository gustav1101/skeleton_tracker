#ifndef SKELETONMERGER_HPP
#define SKELETONMERGER_HPP

#include "RepositoryDataStructures.hpp"

namespace SkeletonMerger
{
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    template<class T> using vector = std::vector<T>;

    void merge_skeleton(const TimedSkeleton * const observed_skeleton,
                        TimedSkeleton& tracked_skeleton);
};

#endif
