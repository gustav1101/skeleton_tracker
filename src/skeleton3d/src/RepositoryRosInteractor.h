#include <ros/ros.h>
#include "SkeletonRepository.h"
#include <skeleton3d/Skeletons3d.h>
#include "exceptions.h"
class RepositoryRosInteractor
{
    using Skeleton = skeleton3d::Skeleton3d;

private:
    struct Params {
        double position_tolerance;
        std::string subscriber_topic;
        std::string publisher_topic;
    };

    ros::NodeHandle node_handle_;
    ros::Publisher repository_publisher_;
    ros::Subscriber repository_subscriber_;
    SkeletonRepository repository_;
    ros::Timer publish_timer_;
    
    static Params read_params();
    void setup_topics(const std::string &subscriber_topic, const std::string &publisher_topic);
    RepositoryRosInteractor(const Params params) : repository_(params.position_tolerance)
    {
        setup_topics(params.subscriber_topic, params.publisher_topic);
    }
    void publish_masterlist(const ros::TimerEvent&);
    void update_masterlist(const skeleton3d::Skeletons3d::ConstPtr &msg);

public:

    ~RepositoryRosInteractor() {};
    static RepositoryRosInteractor get_repository_interactor();
    
};
