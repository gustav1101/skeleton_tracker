#ifndef REPOSITORYROSINTERACTOR_HPP
#define REPOSITORYROSINTERACTOR_HPP

#include <ros/ros.h>
#include "SkeletonRepository.hpp"
#include <skeleton3d/Skeletons3d.h>
#include "exceptions.hpp"

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
 * publish_rate       | double | 0.15     | Time period in s between each publish of masterlist
 * decay_strength     | double | 1.6      | Greater decay strength ages skeleton information faster
 * skeleton_input     | String | required | Topic on which to listen for new skeletons
 * masterlist_ouput   | String | required | Topic on which to publish the skeleton masterlist
 */
class RepositoryRosInteractor
{
    using Skeleton = skeleton3d::Skeleton3d;

public:
    ~RepositoryRosInteractor() {};
    /**
     * Factory method to get a repository ros interactor.
     *
     * Reads ros node parameters to create the repository.
     */
    static RepositoryRosInteractor get_repository_interactor();

private:
    /**
     * Holds all parameters set for this node.
     */
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

    RepositoryRosInteractor(const Params params) : repository_(params.position_tolerance,
                                                               params.decay_strength)
    {
        setup_topics(params.subscriber_topic, params.publisher_topic, params.publish_rate);
    }
    
    /**
     * Read all parameters set for this node.
     *
     * @return Params object, all fields have valid values.
     */
    static Params read_params();

    /**
     * Setup subscriber and publisher. Also sets timer to make sure skeletons are published.
     *
     * @param subscriber_topic_name Topic name on which to listen for skeleton msgs.
     * @param publisher_topic_name Topic name on which to publish skeleton masterlist.
     * @param publish_interval Time between each publication of the skeleton masterlist.
     */
    void setup_topics(const std::string &subscriber_topic_name,
                      const std::string &publisher_topic_name,
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
