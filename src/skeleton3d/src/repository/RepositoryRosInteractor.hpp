#ifndef REPOSITORYROSINTERACTOR_HPP
#define REPOSITORYROSINTERACTOR_HPP

#include <ros/ros.h>
#include "SkeletonRepository.hpp"
#include <skeleton3d/Skeletons3d.h>

/**
 * Ros Interactor for the Skeleton Repository.
 *
 * This class is responsible for forwarding all skeleton messages to the repository and
 * for publishing the skeleton master list.
 *
 * Parameters for this node are as follows:
 *
 * Name               | Type   | defaults | Description
 * ------------------ | ------ | -------- | -------
 * position_tolerance | double | 0.3      | Minimum distance between two different Persons before overlap (not euclidian distance!)
 * publish_interval   | double | 0.15     | Time period in s between each publish of masterlist
 * decay_strength     | double | 1.6      | Greater decay strength ages skeleton information faster
 * skeleton_input     | String | required | Topic on which to listen for new skeletons
 * masterlist_ouput   | String | required | Topic on which to publish the skeleton masterlist
 * global_frame_id    | String | world    | TF Global Frame Name
 */
class RepositoryRosInteractor
{
    using Skeleton = skeleton3d::Skeleton3d;

public:
    /**
     * Holds all parameters set for this node.
     */
    struct Params {
        double position_tolerance;
        double publish_interval;
        double decay_strength;
        std::string subscriber_topic;
        std::string publisher_topic;
        std::string global_frame_id;
    };

    RepositoryRosInteractor(const Params params) : repository_(params.position_tolerance,
                                                               params.decay_strength),
                                                   skeleton_subscriber_(node_handle_,
                                                                        params.subscriber_topic,
                                                                        MESSAGE_QUEUE_SIZE_),
                                                   tf_message_filter_(skeleton_subscriber_,
                                                                      tf_listener_,
                                                                      params.global_frame_id,
                                                                      MESSAGE_QUEUE_SIZE_),
                                                   skeleton_transformer_(tf_listener_,
                                                                         params.global_frame_id),
                                                   global_frame_(params.global_frame_id)
    {
        setup_topic_names(params.publisher_topic, params.publish_interval);
    }

    ~RepositoryRosInteractor() {};

    /**
     * Read all parameters set for this node.
     *
     * @return Params object, all fields have valid values.
     */
    static Params read_params();

private:
    
    const int MESSAGE_QUEUE_SIZE_ = 10;
    ros::NodeHandle node_handle_;
    ros::Publisher repository_publisher_;
    SkeletonRepository repository_;
    ros::Timer publish_timer_;
    message_filters::Subscriber<skeleton3d::Skeletons3d> skeleton_subscriber_;
    tf::TransformListener tf_listener_;
    tf::MessageFilter<skeleton3d::Skeletons3d> tf_message_filter_;
    FrameTransformer skeleton_transformer_;
    const std::string global_frame_;
    
    /**
     * Setup subscriber and publisher. Also sets timer to make sure skeletons are published.
     *
     * @param subscriber_topic_name Topic name on which to listen for skeleton msgs.
     * @param publisher_topic_name Topic name on which to publish skeleton masterlist.
     * @param publish_interval Time between each publication of the skeleton masterlist.
     */
    void setup_topic_names(const std::string &publisher_topic_name,
                           const double &publish_interval);

    /**
     * Retrieve skeleton masterlist from repository and publish.
     */
    void publish_masterlist(const ros::TimerEvent&);

    /**
     * Update the masterlist according to new skeleton information.
     */
    void update_masterlist(const skeleton3d::Skeletons3d::ConstPtr &msg);
};

#endif
