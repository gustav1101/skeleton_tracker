#ifndef REPOSITORYTFTRANSFORMER_HPP
#define REPOSITORYTFTRANSFORMER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include <skeleton3d/BodyPart3d.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "RepositoryDataStructures.hpp"

class FrameTransformer {
    using Skeletons = skeleton3d::Skeletons3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using BodyPart = skeleton3d::BodyPart3d;
    using Point = geometry_msgs::Point;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;

public:
    FrameTransformer(const tf::TransformListener &tf_listener,
                            std::string global_frame) : tf_listener_(tf_listener),
                                                        global_frame_(global_frame) {}
    
    std::vector<TimedSkeleton> transform_to_global_frame(const Skeletons::ConstPtr &skeletons_msg);
    
private:
    const tf::TransformListener &tf_listener_;
    const std::string global_frame_;

    TimedSkeleton transform_to_global_frame(const Skeleton &skeleton,
                                            const std::string &source_frame,
                                            const ros::Time &time_stamp);
    TimedBodyPart transform_to_global_frame(const BodyPart &body_part,
                                            const std::string &source_frame,
                                            const ros::Time &time_stamp);
    Point transform_to_global_frame(const Point &old_point,
                                    const std::string &source_frame,
                                    const ros::Time &time_stamp);
};

#endif
