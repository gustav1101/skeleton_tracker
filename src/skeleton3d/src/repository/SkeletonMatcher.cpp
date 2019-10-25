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

vector<vector<float>> SkeletonMatcher::prepare_empty_distance_matrix(
    int number_of_tracks,
    int number_of_observations)
{
    vector<vector<float>> empty_matrix(number_of_observations);
    for (int row = 0; row < number_of_observations; row++) {
        empty_matrix.at(row) = vector<float>(number_of_tracks);
    }
    return empty_matrix;
}

void SkeletonMatcher::fill_matrix(
    vector<vector<float> > &distance_matrix,
    const vector<TimedSkeleton> &observation)
{
    for(int row=0; row < observation.size(); row++)
    {
        for (int col=0; col<tracks_->size(); col++)
        {
            distance_matrix.at(row).at(col) = skeleton_distance(
                tracks_->at(col),
                observation.at(row));
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
        if( observation_is_far_from_tracks(distance_observation_to_track.at(i)) )
        {
            new_tracks.push_back(observations.at(i));
            observations.erase(observations.begin() + i);
        }
    }
    return new_tracks;
}

bool SkeletonMatcher::observation_is_far_from_tracks(const vector<float>& distances)
{
    bool is_far = true;
    for(auto& distance : distances)
    {
        is_far = is_far && ( distance > DISTANCE_THRESHOLD_ );
    }
    return is_far;
}

void SkeletonMatcher::find_match(vector<vector<float>>& distance_observation_to_track)
{
    // TODO: Do hungarian here
}

void SkeletonMatcher::update_existing_tracks(
    const vector<TimedSkeleton>& observations,
    const vector<vector<float>>& assignment_matrix)
{
    for (int i = 0; i < observations.size(); i++) {
        update_skeleton(
            observations.at(i),
            find_matched_skeleton(assignment_matrix)
            );
    }
}

TimedSkeleton& SkeletonMatcher::find_matched_skeleton(const vector<float> &assignment_matrix)
{
    
}
