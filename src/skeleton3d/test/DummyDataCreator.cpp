#include "DummyDataCreator.hpp"
#include <skeleton3d/BodyPart3d.h>
#include <geometry_msgs/Point.h>

template<class T> using vector = std::vector<T>;
using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;
using BodyPart3d = skeleton3d::BodyPart3d;

namespace DummyDataCreator
{
    BodyPart3d create_dummy_body_part(
        float x_offset,
        float y_offset,
        float z_offset,
        bool part_is_valid,
        int part_id
        );
    BodyPart3d create_dummy_body_part(
        float x_offset,
        float y_offset,
        float z_offset,
        bool part_is_valid,
        int part_id,
        float confidence
        );
    TimedBodyPart create_timed_body_part(
        float x_offset,
        float y_offset,
        float z_offset,
        bool part_is_valid,
        int part_id,
        ros::Time creation_time
        );
    TimedBodyPart to_timed_bodypart(BodyPart3d body_part, ros::Time creation_time);
}


vector<TimedBodyPart> DummyDataCreator::create_body_part_list(float x_offset,
                                                              float y_offset,
                                                              float z_offset,
                                                              int number_of_valid_parts)
{
    ros::Time now = ros::Time::now();
    vector<TimedBodyPart> all_timed_body_parts;

    for (int i = 0; i < 18; i++)
    {
        all_timed_body_parts.push_back(
            create_timed_body_part(x_offset + i, y_offset + i/3, z_offset + i*2, number_of_valid_parts > i, i, now));
    }

    return all_timed_body_parts;
}

BodyPart3d DummyDataCreator::create_dummy_body_part(
    float x_offset, float y_offset, float z_offset, bool part_is_valid, int part_id)
{
    return create_dummy_body_part(x_offset, y_offset, z_offset, part_is_valid, part_id, 0.8);
}

BodyPart3d DummyDataCreator::create_dummy_body_part(
    float x_offset, float y_offset, float z_offset, bool part_is_valid, int part_id, float confidence)
{
    BodyPart3d body_part;
    body_part.part_is_valid = part_is_valid;
    body_part.part_id = part_id;
    geometry_msgs::Point point;
    point.x = x_offset;
    point.y = y_offset;
    point.z = z_offset;
    body_part.point = point;
    body_part.confidence = confidence;

    return body_part;
}

TimedBodyPart DummyDataCreator::to_timed_bodypart(BodyPart3d body_part, ros::Time creation_time)
{
    return {.body_part = body_part,
            .timestamp = creation_time};
}

TimedBodyPart DummyDataCreator::create_timed_body_part(float x_offset, float y_offset, float z_offset, bool part_is_valid, int part_id, ros::Time creation_time)
{
    return to_timed_bodypart(
        create_dummy_body_part(x_offset,
                               y_offset,
                               z_offset,
                               part_is_valid,
                               part_id),
        creation_time);
}
