#include "SkeletonMatcher.hpp"
#include <math.h>

using BodyPart = skeleton3d::BodyPart3d;
using Skeleton = skeleton3d::Skeleton3d;
using Point = geometry_msgs::Point;
using TimedSkeleton = repository_data_structures::TimedSkeleton;
template<class T> using vector = std::vector<T>;


void SkeletonMatcher::update_tracks(
    std::vector<TimedSkeleton> &tracks,
    std::vector<TimedSkeleton> &observation)
{
    vector<vector<float>> distance_matrix =
        create_distance_observation_to_track_matrix(observation);

    vector<TimedSkeleton> new_tracks = filter_for_unassoiciated_observations(
        observation,
        distance_matrix);

    update_existing_tracks(observation, distance_matrix);
    add_new_tracks(new_tracks);
}

vector<vector<float>> SkeletonMatcher::create_distance_observation_to_track_matrix(
    const vector<TimedSkeleton>& observation)
{
    vector<vector<float>> distance_matrix =
        prepare_empty_distance_matrix(tracks_->size(), observation.size());

    fill_matrix(distance_matrix, observation);
    
    return distance_matrix;
}

void SkeletonMatcher::fill_matrix(
    vector<vector<float> > &distance_matrix,
    const vector<TimedSkeleton> &observation)
{
    for(int row=0; row<tracks_->size(); row++)
    {
        for (int col=0; col<observation.size(); col++)
        {
            distance_matrix.at(row).at(col) = skeleton_distance(
                tracks_->at(row),
                observation.at(col));
        }
    }
}

float SkeletonMatcher::skeleton_distance(const TimedSkeleton& skeleton1,
                                         const TimedSkeleton& skeleton2)
{
    float total_distance = 0.0;
    for(int i = 0; i<18; i++)
    {
        total_distance+=body_part_distance(skeleton1.timed_body_parts.at(i),
                                           skeleton2.timed_body_parts.at(i));
    }
    return total_distance;
}

float SkeletonMatcher::body_part_distance(const TimedBodyPart &bodypart1,
                                          const TimedBodyPart &bodypart2)
{
    return sqrt(pow(bodypart1.body_part.point.x - bodypart2.body_part.point.x, 2) +
                pow(bodypart1.body_part.point.y - bodypart2.body_part.point.y, 2) +
                pow(bodypart1.body_part.point.z - bodypart2.body_part.point.z, 2));
}

vector<TimedSkeleton> SkeletonMatcher::filter_for_unassoiciated_observations(
    vector<TimedSkeleton> observations,
    vector<vector<float> > distance_observation_to_track)
{
    vector<TimedSkeleton> new_tracks;
    for (int i = 0; i < observations.size(); ++i) {
        if(observation_is_far_from_tracks(observations.at(i)))
        {
            new_tracks.push_back(observations.at(i));
            observations.erase(observations.begin() + i);
        }
    }
    return new_tracks;
}
