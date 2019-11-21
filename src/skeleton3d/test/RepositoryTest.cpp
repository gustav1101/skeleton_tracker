#include <gtest/gtest.h>
#include "../src/repository/DistanceMatrixOperations.hpp"
#include "../src/repository/RepositoryDataStructures.hpp"
#include <ros/ros.h>
#include <skeleton3d/BodyPart3d.h>
#include "DummyDataCreator.hpp"

template<class T> using vector = std::vector<T>;
using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;


TEST(REPOSITORY, DISTANCE_CALCULATIONS)
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
    
    vector<vector<double>> distance_matrix =
        DistanceMatrixOperations::create_distance_observation_to_track_matrix(observation, track);
    ASSERT_FALSE(bodypart_track.at(17).body_part.part_is_valid);
    ASSERT_EQ(distance_matrix.size(), 1); 
    ASSERT_EQ(distance_matrix.at(0).size(), 1);

    ASSERT_DOUBLE_EQ(distance_matrix.at(0).at(0), 293.09597404263334575551419520639812636241613094368806);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
