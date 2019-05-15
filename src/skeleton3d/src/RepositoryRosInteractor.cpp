#include "RepositoryRosInteractor.hpp"
#include "RepositoryDataStructures.hpp"
#include <skeleton3d/Skeleton3d.h>

using TimedSkeleton = repository_data_structures::TimedSkeleton;
using Skeleton = skeleton3d::Skeleton3d;

RepositoryRosInteractor::Params RepositoryRosInteractor::read_params()
{
    double position_tolerance;
    ros::param::param<double>("~position_tolerance", position_tolerance, 0.3);
    double publish_interval;
    ros::param::param<double>("~publish_interval", publish_interval, 0.15);
    double decay_strength;
    // Decay strength of 1.6 will cause skeleton to disapper after 0.5 seconds with initial
    // confidence of 0.8
    ros::param::param<double>("~decay_strength", decay_strength, 1.6);
    std::string input;
    if (!ros::param::get("~skeleton_input", input))
    {
        throw skeleton_exceptions::LackingRosParameter("skeleton_input");
    };
    std::string output;
    if (!ros::param::get("~masterlist_output", output))
    {
        throw skeleton_exceptions::LackingRosParameter("masterlist_output");
    };
    std::string global_frame_id;
    ros::param::param<std::string>("~global_frame_id", global_frame_id, "world");
    return RepositoryRosInteractor::Params{
        .position_tolerance = position_tolerance,
            .publish_interval = publish_interval,
            .decay_strength = decay_strength,
            .subscriber_topic = input,
            .publisher_topic = output,
            .global_frame_id = global_frame_id,
            };
}

void RepositoryRosInteractor::setup_topic_names(const std::string &publisher_topic_name,
                                                const double &publish_interval)
{
    repository_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>(publisher_topic_name, 5);
    publish_timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &RepositoryRosInteractor::publish_masterlist, this);
    ROS_INFO("Publishing with interval %f", publish_interval);
    tf_message_filter_.registerCallback( boost::bind(&RepositoryRosInteractor::update_masterlist, this, _1) );
}

void RepositoryRosInteractor::publish_masterlist(const ros::TimerEvent&)
{
    const std::vector<Skeleton> skeleton_masterlist = repository_.get_skeleton_masterlist();
    if (skeleton_masterlist.size() == 0)
    {
        return;
    }

    skeleton3d::Skeletons3d skeletons_msg;
    skeletons_msg.header.stamp = ros::Time::now();
    skeletons_msg.skeletons = skeleton_masterlist;
    repository_publisher_.publish(skeletons_msg);
}

void RepositoryRosInteractor::update_masterlist(const skeleton3d::Skeletons3d::ConstPtr &msg)
{
    std::vector<TimedSkeleton> transformed_skeletons =
        skeleton_transformer_.transform_to_global_frame(msg);
    repository_.update_skeletons(transformed_skeletons);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "skeleton3d_repository");
    RepositoryRosInteractor interactor(RepositoryRosInteractor::read_params());

    ros::spin();
}
