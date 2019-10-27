#ifndef SKELETONMATCHER_HPP
#define SKELETONMATCHER_HPP

#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include "RepositoryDataStructures.hpp"
#include <boost/optional.hpp>

class SkeletonMatcher {
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;
    template<class T> using vector = std::vector<T>;
    template<class T> using optional = boost::optional<T>;
    
public:
    SkeletonMatcher(float distance_threshold) :
        DISTANCE_THRESHOLD_(distance_threshold),
        tracks_(nullptr)
        {}
    void update_tracks(std::vector<TimedSkeleton>& tracks,
                       const std::vector<TimedSkeleton>& observation);

private:
    vector<const TimedSkeleton * const> create_pointer_vector(const vector<TimedSkeleton>& original);
    vector<vector<float>> create_distance_observation_to_track_matrix(
        const vector<const TimedSkeleton*const>& observation
        );
    vector<vector<float>> prepare_empty_distance_matrix(
        int number_of_tracks,
        int number_of_observations);
    void fill_matrix(
        vector<vector<float>>& distance_matrix,
        const vector<const TimedSkeleton * const>& observation);
    float skeleton_distance(const TimedSkeleton& skeleton1,
                            const TimedSkeleton& skeleton2);
    float body_part_distance(const TimedBodyPart& bodypart1,
                             const TimedBodyPart& bodypart2);
    vector<const TimedSkeleton * const> filter_for_unassoiciated_observations(
        vector<const TimedSkeleton * const>& observations,
        const vector<vector<float>>& distance_observation_to_track);
    bool observation_is_far_from_tracks(const vector<float>& distances);
    void match_tracks_against_observations(vector<vector<float>>& distance_observation_to_track); // TODO: Impl
    void process_observations_close_to_tracks(
        const vector<const TimedSkeleton * const>& observations,
        const vector<vector<float>>& assignment_matrix);
    void process_observation_close_to_track(
        const TimedSkeleton * const observation, const vector<float>& assignment_matrix_row);
    optional<TimedSkeleton&> matched_skeleton(const vector<float>& assignment_matrix);
    void update_existing_track(const TimedSkeleton * const observation, TimedSkeleton& track);
    void add_new_tracks(const vector<const TimedSkeleton * const>& new_tracks);
    void add_new_track(const TimedSkeleton * const new_track);
    
    const float DISTANCE_THRESHOLD_;
    vector<TimedSkeleton>* tracks_;
};

#endif
