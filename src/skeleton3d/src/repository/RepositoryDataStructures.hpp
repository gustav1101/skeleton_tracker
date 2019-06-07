#ifndef REPOSITORYDATASTRUCTURES_HPP
#define REPOSITORYDATASTRUCTURES_HPP

#include <ros/ros.h>
#include <skeleton3d/BodyPart3d.h>

namespace repository_data_structures {
    struct TimedBodyPart
    {
        skeleton3d::BodyPart3d body_part;
        ros::Time timestamp;
    };
    
    struct TimedSkeleton
    {
        std::vector<TimedBodyPart> timed_body_parts;
        unsigned int id;
    };
}

#endif
