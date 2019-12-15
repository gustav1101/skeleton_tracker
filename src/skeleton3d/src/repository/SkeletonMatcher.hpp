#ifndef SKELETONMATCHER_HPP
#define SKELETONMATCHER_HPP

#include "RepositoryDataStructures.hpp"
#include <boost/optional.hpp>

class SkeletonMatcher
{
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    template<class T> using vector = std::vector<T>;
    template<class T> using optional = boost::optional<T>;
    
public:
    SkeletonMatcher(double distance_threshold) :
        DISTANCE_THRESHOLD_(distance_threshold)
        {}

    // Actually the observations are const as well, just can't denote that because:
    // I'm working on pointers with the observation so I don't have to copy everything.
    // I want to return the vector of observations that are not associated with any tracks.
    // Since you can't create a vector of references I really need to work with pointers.
    // Trouble is: You can't put anything const into a vector.
    std::vector<TimedSkeleton *> update_tracks_and_return_unmatched_observations(
        std::vector<TimedSkeleton>& tracks,
        std::vector<TimedSkeleton>& observation);

private:
    vector<TimedSkeleton *> create_pointer_vector(vector<TimedSkeleton>& original);
    vector<TimedSkeleton *> filter_too_isolated_observations(
        vector<TimedSkeleton *>& observations,
        vector<vector<double>>& distance_observation_to_track);
    bool observation_is_far_from_tracks(const vector<double>& distances);
    vector<TimedSkeleton *> update_or_return_new(
        const vector<TimedSkeleton *>& observations,
        vector<TimedSkeleton>& tracks,
        const vector<vector<bool>>& assignment_matrix);
    optional<TimedSkeleton&> find_corresponding_track(
        vector<TimedSkeleton>& tracks,
        const vector<bool>& assignment_matrix_row);
    
    const double DISTANCE_THRESHOLD_;
};

#endif
