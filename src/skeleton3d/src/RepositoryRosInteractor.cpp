#include "RepositoryRosInteractor.h"

RepositoryRosInteractor RepositoryRosInteractor::get_repository_interactor()
{
    RepositoryRosInteractor::Params params = read_params();
    return RepositoryRosInteractor(params);
}

RepositoryRosInteractor::Params RepositoryRosInteractor::read_params()
{
    double position_tolerance;
    ros::param::param<double>("~position_tolerance", position_tolerance, 0.3);
    return RepositoryRosInteractor::Params{ .position_tolerance = position_tolerance };
}

void RepositoryRosInteractor::setup_topics(const std::string &subscriber_topic, const std::string &publisher_topic)
{
    repository_subscriber_ = node_handle_.subscribe(
        subscriber_topic,
        10,
        &RepositoryRosInteractor::update_masterlist,
        this);
    repository_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>(publisher_topic, 5);

    publish_timer_ = node_handle_.createTimer(ros::Duration(0.15), &RepositoryRosInteractor::publish_masterlist, this);
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
    repository_.update_skeletons(msg->skeletons);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "skeleton3d_repository");
    RepositoryRosInteractor interactor = RepositoryRosInteractor::get_repository_interactor();

    ros::spin();
}
