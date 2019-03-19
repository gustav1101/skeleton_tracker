#include <ros/ros.h>
#include "SkeletonRepository.h"
#include <skeleton3d/Skeletons3d.h>
#include "exceptions.h"
class RepositoryRosInteractor
{
    using Skeleton = skeleton3d::Skeleton3d;

public:
    ~RepositoryRosInteractor() {};
    static RepositoryRosInteractor get_repository_interactor();

private:
    struct Params {
        double position_tolerance;
        double publish_rate;
        double decay_strength;
        std::string subscriber_topic;
        std::string publisher_topic;
    };

    ros::NodeHandle node_handle_;
    ros::Publisher repository_publisher_;
    ros::Subscriber repository_subscriber_;
    SkeletonRepository repository_;
    ros::Timer publish_timer_;
    
    static Params read_params();
    void setup_topics(const std::string &subscriber_topic, const std::string &publisher_topic, const double &publish_rate);
    RepositoryRosInteractor(const Params params) : repository_(params.position_tolerance, params.decay_strength)
    {
        setup_topics(params.subscriber_topic, params.publisher_topic, params.publish_rate);
    }
    void publish_masterlist(const ros::TimerEvent&);
    void update_masterlist(const skeleton3d::Skeletons3d::ConstPtr &msg);
};
