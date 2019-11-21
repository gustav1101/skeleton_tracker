#ifndef DUMMYDATACREATOR
#define DUMMYDATACREATOR

#include "../src/repository/RepositoryDataStructures.hpp"
#include <ros/ros.h>
#include <skeleton3d/BodyPart3d.h>

namespace DummyDataCreator
{
    template<class T> using vector = std::vector<T>;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;

    vector<TimedBodyPart> create_body_part_list(float x_offset,
                                        float y_offset,
                                        float z_offset,
                                        int number_of_valid_parts);
}

#endif
