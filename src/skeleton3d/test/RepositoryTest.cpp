#include <catch_ros/catch.hpp>
#include "../src/repository/DistanceMatrixOperations.hpp"
#include "../src/repository/RepositoryDataStructures.hpp"
#include <ros/ros.h>
#include <skeleton3d/BodyPart3d.h>
#include "DummyDataCreator.hpp"
#include "../src/repository/MunkresSolver.hpp"

template<class T> using vector = std::vector<T>;
using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;


SCENARIO("distances between multiple skeletons can be calculated in a matrix", "[metric]") {
    GIVEN("Two skeletons")
    {
        vector<TimedBodyPart> bodypart_observation = DummyDataCreator::create_body_part_list(
            0.0, 0.0, 0.0, 18);
        vector<TimedBodyPart> bodypart_track = DummyDataCreator::create_body_part_list(
            0.0, 5.0, 16.5, 17);
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

            THEN("The size of the matrix is 1x1 and correct")
            {
                REQUIRE(distance_matrix.size() == 1); 
                REQUIRE_NOTHROW(distance_matrix.at(0).size(), 1);
                REQUIRE(distance_matrix.at(0).at(0) == Approx(293.09597404263334575551419520639812636241613094368806));
            }
        }

        WHEN("A third skeleton is added")
        {
            vector<TimedBodyPart> second_observation_parts = DummyDataCreator::create_body_part_list(
                -2.0, 0.0, 0.9, 14);
            TimedSkeleton second_observed_skeleton =
                {.timed_body_parts = second_observation_parts,
                 .id = 0};
            observation.push_back(&second_observed_skeleton);

            vector<vector<double>> distance_matrix =
                DistanceMatrixOperations::create_distance_observation_to_track_matrix(observation, track);

            THEN("The resulting distance matrix is correct in 2x1")
            {
                REQUIRE(distance_matrix.size() == 2); 
                REQUIRE_NOTHROW(distance_matrix.at(0).size(), 1);
                REQUIRE_NOTHROW(distance_matrix.at(1).size(), 1);
                REQUIRE(distance_matrix.at(0).at(0) == Approx(293.09597404263334575551419520639812636241613094368806));
                REQUIRE(distance_matrix.at(1).at(0) == Approx(231.04666195381399623481003386277413146812025999217664));
            }
        }
    }

}
