#include "DistanceMatrixOperations.hpp"

#include <math.h>
#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>

using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;
template<class T> using vector = std::vector<T>;

namespace DistanceMatrixOperations {
    vector<vector<double>> prepare_empty_distance_matrix(
        int number_of_tracks,
        int number_of_observations);
    void fill_matrix(
        const vector<TimedSkeleton *>& observation,
        const vector<TimedSkeleton>& tracks,
        vector<vector<double> > &distance_matrix);
    double skeleton_distance(const TimedSkeleton& skeleton1,
                            const TimedSkeleton& skeleton2);
    double body_part_distance(const TimedBodyPart& bodypart1,
                             const TimedBodyPart& bodypart2);
    vector<vector<bool>> create_assignment_matrix(const vector<vector<double>>& distance_matrix);
    bool any_part_invalid(const TimedBodyPart& part1, const TimedBodyPart& part2);
}
    
vector<vector<double>> DistanceMatrixOperations::create_distance_observation_to_track_matrix(
    const vector<TimedSkeleton *>& observations,
    const vector<TimedSkeleton>& tracks)
{
    vector<vector<double>> distance_matrix = prepare_empty_distance_matrix(tracks.size(), observations.size());

    fill_matrix(observations, tracks, distance_matrix);
    
    return distance_matrix;
}

vector<vector<double>> DistanceMatrixOperations::prepare_empty_distance_matrix(
    int number_of_tracks,
    int number_of_observations)
{
    vector<vector<double>> empty_matrix(
        number_of_observations,vector<double>(number_of_tracks,0.0));
    return empty_matrix;
}

void DistanceMatrixOperations::fill_matrix(
    const vector<TimedSkeleton *>& observation,
    const vector<TimedSkeleton>& tracks,
    vector<vector<double> > &distance_matrix)
{
    for(int row=0; row < observation.size(); row++)
    {
        for (int col=0; col<tracks.size(); col++)
        {
            distance_matrix.at(row).at(col) = skeleton_distance(
                tracks.at(col),
                *observation.at(row));
        }
    }
}

double DistanceMatrixOperations::skeleton_distance(const TimedSkeleton& skeleton1,
                                                  const TimedSkeleton& skeleton2)
{
    double total_distance = 0.0;
    for(int i = 0; i<18; i++)
    {
        total_distance+=body_part_distance(skeleton1.timed_body_parts.at(i),
                                           skeleton2.timed_body_parts.at(i));
    }
    return total_distance;
}

double DistanceMatrixOperations::body_part_distance(const TimedBodyPart &bodypart1,
                                          const TimedBodyPart &bodypart2)
{
    if (any_part_invalid(bodypart1, bodypart2))
    {
        return 0.0;
    }
    else
    {
        return sqrt(pow(bodypart1.body_part.point.x - bodypart2.body_part.point.x, 2) +
                    pow(bodypart1.body_part.point.y - bodypart2.body_part.point.y, 2) +
                    pow(bodypart1.body_part.point.z - bodypart2.body_part.point.z, 2));    
    }
}

bool DistanceMatrixOperations::any_part_invalid(const TimedBodyPart &part1, const TimedBodyPart &part2)
{
    return (!part1.body_part.part_is_valid || !part2.body_part.part_is_valid);
}

vector<vector<bool>> DistanceMatrixOperations::find_match_over_matrix(vector<vector<double>> distance_matrix)
{
    // TODO: Do hungarian here

    return create_assignment_matrix(distance_matrix);
}

vector<vector<bool>> DistanceMatrixOperations::create_assignment_matrix(const vector<vector<double>>& distance_matrix)
{
    vector<vector<bool>> assignment_matrix(distance_matrix.size());
    for (auto& row : assignment_matrix) {
        row.resize(distance_matrix.at(0).size());
    }

    for(int row = 0; row < assignment_matrix.size(); row++)
    {
        for(int col = 0; col < assignment_matrix.at(0).size(); col++)
        {
            assignment_matrix.at(row).at(col) = (distance_matrix.at(row).at(col) == 0);
        }
    }
    return assignment_matrix;
}
