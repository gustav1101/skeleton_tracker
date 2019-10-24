#ifndef SKELETONREPOSITORY_HPP
#define SKELETONREPOSITORY_HPP

#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include "RepositoryDataStructures.hpp"

class SkeletonMatcher {
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;
    template<class T> using vector = std::vector<T>;
    
public:
    SkeletonMatcher(float distance_threshold) :
        DISTANCE_THRESHOLD_(distance_threshold),
        tracks_(nullptr)
        {}
    void update_tracks(std::vector<TimedSkeleton>& tracks,
                       std::vector<TimedSkeleton>& observation);

private:
    vector<vector<float>> create_distance_observation_to_track_matrix(
        const vector<TimedSkeleton>& observation
        );
    vector<vector<float>> prepare_empty_distance_matrix(
        int number_of_tracks,
        int number_of_observations);
    void fill_matrix(
        vector<vector<float>>& distance_matrix,
        const vector<TimedSkeleton>& observation);
    float skeleton_distance(const TimedSkeleton& skeleton1,
                            const TimedSkeleton& skeleton2);
    float body_part_distance(const TimedBodyPart& bodypart1,
                             const TimedBodyPart& bodypart2);
    vector<TimedSkeleton> filter_for_unassoiciated_observations(
        vector<TimedSkeleton> observations,
        vector<vector<float>> distance_observation_to_track);
    bool observation_is_far_from_tracks(TimedSkeleton observation);
    void find_match(vector<vector<float>> distance_observation_to_track);
    void update_existing_tracks(
        vector<TimedSkeleton> observations,
        vector<vector<float>> assignment_matrix);
    void add_new_tracks(vector<TimedSkeleton> new_tracks);
    
    const float DISTANCE_THRESHOLD_;
    vector<TimedSkeleton>* tracks_;
};

#endif
