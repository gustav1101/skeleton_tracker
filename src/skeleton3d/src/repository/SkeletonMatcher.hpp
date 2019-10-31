#ifndef SKELETONMATCHER_HPP
#define SKELETONMATCHER_HPP

#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include "RepositoryDataStructures.hpp"
#include <boost/optional.hpp>
#include "SkeletonRepository.hpp"

class SkeletonMatcher {
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;
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
        TimedSkeleton * const track;
    };
    
    vector<const TimedSkeleton * const> create_pointer_vector(const vector<TimedSkeleton>& original);
    vector<vector<float>> create_distance_observation_to_track_matrix(
        const vector<const TimedSkeleton * const>& observations,
        const vector<TimedSkeleton>& tracks);
    vector<vector<float>> prepare_empty_distance_matrix(
        int number_of_tracks,
        int number_of_observations);
    void fill_matrix(
        const vector<const TimedSkeleton * const>& observation,
        const vector<TimedSkeleton>& tracks,
        vector<vector<float> > &distance_matrix);
    float skeleton_distance(const TimedSkeleton& skeleton1,
                            const TimedSkeleton& skeleton2);
    float body_part_distance(const TimedBodyPart& bodypart1,
                             const TimedBodyPart& bodypart2);
    vector<const TimedSkeleton * const> filter_too_isolated_observations(
        vector<const TimedSkeleton * const>& observations,
        const vector<vector<float>>& distance_observation_to_track);
    bool observation_is_far_from_tracks(const vector<float>& distances);
    void find_match_over_matrix(vector<vector<float>>& distance_observation_to_track); // TODO: Impl
    vector<const TimedSkeleton * const> update_matched_tracks_and_return_others(
        vector<const TimedSkeleton * const>& observations,
        vector<TimedSkeleton>& tracks,
        vector<vector<float>>& distance_matrix);
    vector<optional<observation_track_match>> assign_tracks_to_observations(
        vector<const TimedSkeleton * const>& observations,
        vector<TimedSkeleton>& tracks,
        vector<vector<float>>& distance_matrix);
    optional<observation_track_match> find_observation_track_pair(
        const TimedSkeleton * const& observation,
        vector<TimedSkeleton>& tracks,
        vector<float>& assignment_matrix_row);
    void update_existing_track(const TimedSkeleton * const observation, TimedSkeleton& track);
    
    const float DISTANCE_THRESHOLD_;
    SkeletonRepository& repository_;
};

#endif
