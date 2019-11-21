#include "SkeletonMatcher.hpp"

#include "SkeletonMerger.hpp"
#include "DistanceMatrixOperations.hpp"
using TimedSkeleton = repository_data_structures::TimedSkeleton;
template<class T> using vector = std::vector<T>;
template<class T> using optional = boost::optional<T>;


std::vector<TimedSkeleton *> SkeletonMatcher::update_tracks_and_return_unmatched_observations(
    std::vector<TimedSkeleton> &tracks,
    std::vector<TimedSkeleton> &_observation)
{
    // TODO: Early abort with error if observations is empty

    vector<TimedSkeleton *> observation = create_pointer_vector(_observation);
    vector<vector<float>> distance_matrix =
        DistanceMatrixOperations::create_distance_observation_to_track_matrix(
            observation,
            tracks);

    vector<TimedSkeleton *> new_tracks = filter_too_isolated_observations(
        observation,
        distance_matrix);

    vector<vector<bool>> assignment_matrix =
        DistanceMatrixOperations::find_match_over_matrix(distance_matrix);

    vector<TimedSkeleton *> unmatched_tracks =
        update_or_return_new(observation, tracks, assignment_matrix);

    new_tracks.insert(new_tracks.end(), unmatched_tracks.begin(), unmatched_tracks.end());
    
    return new_tracks;
}

vector<TimedSkeleton *> SkeletonMatcher::create_pointer_vector(
    vector<TimedSkeleton> &original)
{
    vector<TimedSkeleton*> pointer_vector;
    pointer_vector.reserve(original.size());
    for(auto& skeleton : original)
    {
        pointer_vector.push_back(&skeleton);
    }
    return pointer_vector;
}

vector<TimedSkeleton *> SkeletonMatcher::filter_too_isolated_observations(
    vector<TimedSkeleton *>& observations,
    vector<vector<float>>& distance_observation_to_track)
{
    vector<TimedSkeleton *> new_tracks;
    for (int i = 0; i < observations.size(); ++i) {
        if( observation_is_far_from_tracks(distance_observation_to_track.at(i)) )
        {
            new_tracks.push_back(observations.at(i));
            observations.erase(observations.begin() + i);
            distance_observation_to_track.erase(distance_observation_to_track.begin() + i);
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

vector<TimedSkeleton *> SkeletonMatcher::update_or_return_new(
    const vector<TimedSkeleton *> &observations,
    vector<TimedSkeleton> &tracks,
    const vector<vector<bool> > &assignment_matrix)
{
    vector<TimedSkeleton *> new_tracks;
    new_tracks.reserve(observations.size());

    for(int i = 0; i < observations.size(); i++)
    {
        optional<TimedSkeleton&> corresponding_track =
            find_corresponding_track(tracks, assignment_matrix.at(i));
        if(corresponding_track)
        {
            SkeletonMerger::merge_skeleton(observations.at(i), *corresponding_track);
        }
        else {
            new_tracks.push_back(observations.at(i));
        }
    }
    return new_tracks;
}

optional<TimedSkeleton&> SkeletonMatcher::find_corresponding_track(
    vector<TimedSkeleton> &tracks,
    const vector<bool> &assignment_matrix_row)
{
    auto elem = std::find(assignment_matrix_row.begin(), assignment_matrix_row.end(), true);
    if(elem != assignment_matrix_row.end())
    {
        return tracks.at(elem - assignment_matrix_row.begin());
    }
    else {
        return boost::none;
    }
}
