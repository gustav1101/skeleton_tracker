#ifndef DISTANCEMATRIXOPERATIONS_HPP
#define DISTANCEMATRIXOPERATIONS_HPP

#include "RepositoryDataStructures.hpp"

namespace DistanceMatrixOperations
{
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    template<class T> using vector = std::vector<T>;

    vector<vector<float>> create_distance_observation_to_track_matrix(
        const vector<const TimedSkeleton * const>& observations,
        const vector<TimedSkeleton>& tracks);
    vector<vector<bool>> find_match_over_matrix(vector<vector<float>> distance_matrix); // TODO: Impl
}

#endif
