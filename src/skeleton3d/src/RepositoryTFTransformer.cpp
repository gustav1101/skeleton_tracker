#include "RepositoryTFTransformer.hpp"

using Skeleton = skeleton3d::Skeleton3d;
using BodyPart = skeleton3d::BodyPart3d;
using Point = geometry_msgs::Point;
using TimedSkeleton = repository_data_structures::TimedSkeleton;
using TimedBodyPart = repository_data_structures::TimedBodyPart;

std::vector<TimedSkeleton> RepositoryTFTransformer::transform_to_global_frame(const Skeletons::ConstPtr &skeletons_msg)
{
    ROS_INFO("Trying to transform from %s to %s",
             skeletons_msg->header.frame_id.c_str(),
             global_frame_.c_str());
    std::vector<TimedSkeleton> transformed_skeletons;
    for (const Skeleton &simple_skeleton : skeletons_msg->skeletons)
    {
        transformed_skeletons.push_back(transform_to_global_frame(simple_skeleton,
                                                                  skeletons_msg->header.frame_id,
                                                                  skeletons_msg->header.stamp));
    }
    return transformed_skeletons;
}

TimedSkeleton RepositoryTFTransformer::transform_to_global_frame(const Skeleton &skeleton,
                                                                 const std::string &source_frame,
                                                                 const ros::Time &time_stamp)
{
    TimedSkeleton timed_skeleton;
    for (const BodyPart &body_part : skeleton.body_parts)
    {
        timed_skeleton.timed_body_parts.push_back(transform_to_global_frame(body_part,
                                                                            source_frame,
                                                                            time_stamp));
    }
    return timed_skeleton;
}

TimedBodyPart RepositoryTFTransformer::transform_to_global_frame(const BodyPart &body_part,
                                                                 const std::string &source_frame,
                                                                 const ros::Time &time_stamp)
{
    TimedBodyPart transformed_body_part;
    transformed_body_part.body_part = body_part;
    transformed_body_part.timestamp = time_stamp;
    transformed_body_part.body_part.point = transform_to_global_frame(body_part.point,
                                                                      source_frame,
                                                                      time_stamp);
    return transformed_body_part;
}

Point RepositoryTFTransformer::transform_to_global_frame(const Point &old_point,
                                                         const std::string &source_frame,
                                                         const ros::Time &time_stamp)
{
    tf::Point tf_old_point(old_point.x, old_point.y, old_point.z);
    tf::Stamped<tf::Point> stamped_old_point(tf_old_point, time_stamp, source_frame);
    tf::Stamped<tf::Point> targetpoint;
    tf_listener_.transformPoint(global_frame_, stamped_old_point, targetpoint);
    
    Point return_point;
    return_point.x=targetpoint.getX();
    return_point.y=targetpoint.getY();
    return_point.z=targetpoint.getZ();
    return return_point;
}
