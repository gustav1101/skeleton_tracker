#include <catch_ros/catch.hpp>
#include "../src/repository/DistanceMatrixOperations.hpp"
#include "../src/repository/RepositoryDataStructures.hpp"
#include <ros/ros.h>
#include <skeleton3d/BodyPart3d.h>
#include "DummyDataCreator.hpp"

template<class T> using vector = std::vector<T>;
using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;


SCENARIO("distances between multiple skeletons can be calculated in a matrix", "[metric]") {
    GIVEN("Two skeletons")
    {
        vector<TimedBodyPart> bodypart_observation = DummyDataCreator::create_body_part_list(
            0.0,
            0.0,
            0.0,
            18);
        vector<TimedBodyPart> bodypart_track = DummyDataCreator::create_body_part_list(
            0.0,
            5.0,
            16.5,
            17);
        TimedSkeleton observed_skeleton =
            {.timed_body_parts = bodypart_observation,
             .id = 0};
        vector<TimedSkeleton*> observation = {&observed_skeleton};
        vector<TimedSkeleton> track = {
            {.timed_body_parts = bodypart_track,
             .id = 0}};

        WHEN("the distance is calculated")
        {
            vector<vector<double>> distance_matrix =
                DistanceMatrixOperations::create_distance_observation_to_track_matrix(observation, track);

            THEN("The size of the matrix is 1x1")
            {
                REQUIRE(distance_matrix.size() == 1); 
                REQUIRE_NOTHROW(distance_matrix.at(0).size(), 1);
                REQUIRE(distance_matrix.at(0).at(0) == Approx(293.09597404263334575551419520639812636241613094368806));
            }
        }
        
        
    }
}
