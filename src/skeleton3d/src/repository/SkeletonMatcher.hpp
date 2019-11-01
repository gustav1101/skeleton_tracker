#ifndef SKELETONMATCHER_HPP
#define SKELETONMATCHER_HPP

// #include <geometry_msgs/Point.h>
// #include <skeleton3d/BodyPart3d.h>
// #include <skeleton3d/Skeleton3d.h>
// #include <skeleton3d/Skeletons3d.h>
#include "RepositoryDataStructures.hpp"
#include <boost/optional.hpp>
#include "SkeletonRepository.hpp"

class SkeletonMatcher {
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    template<class T> using vector = std::vector<T>;
    template<class T> using optional = boost::optional<T>;
    
public:
    SkeletonMatcher(float distance_threshold, SkeletonRepository& repository) :
        DISTANCE_THRESHOLD_(distance_threshold),
        repository_(repository)
        {}
    std::vector<const TimedSkeleton * const> update_tracks_and_return_unmatched_observations(
        std::vector<TimedSkeleton>& tracks,
        const std::vector<TimedSkeleton>& observation);

private:
    struct observation_track_match {
        const TimedSkeleton * const observation;
        TimedSkeleton& track;
    };
    
    vector<const TimedSkeleton * const> create_pointer_vector(const vector<TimedSkeleton>& original);
    vector<const TimedSkeleton * const> filter_too_isolated_observations(
        vector<const TimedSkeleton * const>& observations,
        vector<vector<float>>& distance_observation_to_track);
    bool observation_is_far_from_tracks(const vector<float>& distances);
    vector<const TimedSkeleton * const> update_or_return_new(
        const vector<const TimedSkeleton * const>& observations,
        vector<TimedSkeleton>& tracks,
        const vector<vector<bool>>& assignment_matrix);
    optional<TimedSkeleton&> find_corresponding_track(
        vector<TimedSkeleton>& tracks,
        const vector<bool>& assignment_matrix_row);
    
    const float DISTANCE_THRESHOLD_;
    SkeletonRepository& repository_;
};

#endif
