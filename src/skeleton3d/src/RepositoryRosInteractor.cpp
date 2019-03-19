#include "RepositoryRosInteractor.hpp"

RepositoryRosInteractor RepositoryRosInteractor::get_repository_interactor()
{
    RepositoryRosInteractor::Params params = read_params();
    return RepositoryRosInteractor(params);
}

RepositoryRosInteractor::Params RepositoryRosInteractor::read_params()
{
    double position_tolerance;
    ros::param::param<double>("~position_tolerance", position_tolerance, 0.3);
    double publish_rate;
    ros::param::param<double>("~publish_rate", publish_rate, 0.15);
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
    return RepositoryRosInteractor::Params{
        .position_tolerance = position_tolerance,
            .publish_rate = publish_rate,
            .decay_strength = decay_strength,
            .subscriber_topic = input,
            .publisher_topic = output
            };
}

void RepositoryRosInteractor::setup_topics(const std::string &subscriber_topic, const std::string &publisher_topic, const double &publish_rate)
{
    repository_subscriber_ = node_handle_.subscribe(
        subscriber_topic,
        10,
        &RepositoryRosInteractor::update_masterlist,
        this);
    repository_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>(publisher_topic, 5);
    ROS_INFO("Publishing with rate %f", publish_rate);
    publish_timer_ = node_handle_.createTimer(ros::Duration(publish_rate), &RepositoryRosInteractor::publish_masterlist, this);
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
    repository_.update_skeletons(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "skeleton3d_repository");
    RepositoryRosInteractor interactor = RepositoryRosInteractor::get_repository_interactor();

    ros::spin();
}
