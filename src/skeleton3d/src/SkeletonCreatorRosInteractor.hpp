#ifndef SKELETONCREATORROSINTERACTOR_HPP
#define SKELETONCREATORROSINTERACTOR_HPP

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include "SkeletonCreator.hpp"
#include "StaticCloudFilter.hpp"

/**
 * Ros Interactor for the Skeleton Creator class.
 *
 * This module handles all ros interaction (publishing, subscribing, ...) for the Skeleton Creator. It listens for the tf pose and the pointcloud and calls the SkeletonCreator to generate the skeleton. Both messages (pointcloud and pose) need to have similar timestamps, otherwise the callback will not be called.
 * SkeletonCreator then uses this module to publish the resulting skeletons on a corresponding ros topic.
 * This module also handles reading and passing on of node parameters, some of which are required. The parameters are as follows:
 * Name                            | Type   | defaults | Description
 * ------------------------------- | ------ | -------- | --------------------------------------------------
 * input_pose                      | String | required | Name of tf pose topic (for subscribing)
 * input_pointcloud                | String | required | Name of pointcloud topic (for subscribing)
 * output_skeleton                 | String | required | Name of skeleton topic (for publishing)
 * frame_id                        | String | required | Name of camera base frame id
 * x_frame_offset                  | double | 0.0      | X offset for depth image against pose coorinates
 * scatter_step_distance           | int    | 2        | Scatter Distance for PointFinder
 * scatter_steps                   | int    | 4        | Number of scatter steps in each direction (as radius, not diameter)
 * number_of_messages_to_discard   | int    | 10       | Number of messages to discard upon initialisation before calibration starts
 * number_of_calibration_messages  | int    | 5        | Number of messages that will only be used to record the static background
 */
class SkeletonCreatorRosInteractor
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<tfpose_ros::Persons, PointCloud>;
    
public:
    /**
     * Data structure that can hold all parameters for this program.
     */
    struct RosParams{
        const std::string pose_topic_name;
        const std::string pointcloud_topic_name;
        const std::string skeleton_topic_name;
        const std::string frame_id;
        const int scatter_step_distance;
        const int scatter_steps;
        const int number_of_messages_to_discard;
        const int number_of_calibration_messages;
        const double x_frame_offset;
    };

    /**
     * Constructor. Sets up subscribers and listeners.
     *
     * @param [in] params RosParams with all fields specified.
     */
    SkeletonCreatorRosInteractor(const RosParams params) :
        skeleton_creator_(
            params.scatter_step_distance,
            params.scatter_steps,
            params.x_frame_offset),
        static_cloud_filter_(
            params.number_of_messages_to_discard,
            params.number_of_calibration_messages),
        tfpose_subscriber_(
            node_handle_,
            params.pose_topic_name,
            INPUT_QUEUE_SIZE_),
        pointcloud_subscriber_(
            node_handle_,
            params.pointcloud_topic_name,
            INPUT_QUEUE_SIZE_),
        message_synchronizer_(
            ApproximateTimePolicy(INPUT_QUEUE_SIZE_),
            tfpose_subscriber_,
            pointcloud_subscriber_),
        frame_id_(params.frame_id)
    {
        message_synchronizer_.registerCallback(
            boost::bind(&SkeletonCreatorRosInteractor::generate_skeleton, this, _1, _2));
        create_publisher(params.skeleton_topic_name);
        ROS_INFO("Scatter steps: %i, Scatter distance: %i",
                 params.scatter_steps,
                 params.scatter_step_distance);
    }
    
    ~SkeletonCreatorRosInteractor() {}

    /**
     * Reads parameters given to this node.
     *
     * See SkeletonCreatorRosInteractor class description for details on parameters.
     * This method will set default values for the frame offset (0.0) and for the scatter distance (6). All opther values do not default and need to be specified.
     * @return The RosParams instance holding all parameters.
     */
    static RosParams read_params();

private:

    /** Sets size for all queues in subscribers and synchronizers. */
    static const int INPUT_QUEUE_SIZE_ = 30;
    ros::NodeHandle node_handle_;
    SkeletonCreator skeleton_creator_;
    StaticCloudFilter static_cloud_filter_;
    /** Subscriber to the tf openpose topic */
    /* Note that the subscribers and synchronizer are set as pointers because
     * they are set in their own methods to avoid cluttering (and thus they cannot be
     * initialised in the initialization list).
     */
    message_filters::Subscriber<tfpose_ros::Persons> tfpose_subscriber_;
    /** Pointcloud subscriber for retrieving depth information */
    message_filters::Subscriber<PointCloud> pointcloud_subscriber_;
    /** Synchronizer between depth and pose messages */
    message_filters::Synchronizer<ApproximateTimePolicy> message_synchronizer_;
    /** Output topic for skeletons */
    ros::Publisher skeleton_publisher_;
    
    std::string frame_id_;
    /** Boundaries need to be set before camera input can be evaluated. This happens in generate_skeletons(). */
    bool window_boundaries_set_ = false;

    /**
     * Retrieve one individual parameter from the ros node environment.
     *
     * @param [in] param_name Name of the parameter as given to the ros node.
     * @throws skeleton_exceptions::LackingRosParameter when the parameter is not found in the node environment.
     */
    static std::string get_param(const std::string &param_name);

    /**
     * Generate the skeleton(s) from the argument data.
     *
     * This method should be invoked with message arguments that correspond to the same period of time for good results.
     *
     * @param [in] persons_msg The Persons message that holds the tfpose data.
     * @param [in] point_cloud The pointcloud message for depth information, with only 3d position of points being the relevant information. (Does not need to be depth_registered.)
     */
    void generate_skeleton(const tfpose_ros::Persons::ConstPtr &persons_msg,
                           const PointCloud::ConstPtr &point_cloud);

    /**
     * Publish given list of skeletons on Ros.
     *
     * @param [in] skeletons Vector holding all skeletons to be published.
     */
    void publish_skeletons(std::vector<skeleton3d::Skeleton3d> skeletons);

    /**
     * Setup listener topics as well as synchronizer.
     *
     * For the listeners, an approximate time policy is used to call the callback with messages having similar timestamps.
     *
     * @param [in] pose_topic_name Name of the ros topic to listen on for the tf pose.
     * @param [in] poincloud_topic_name Name of the ros topic to listen on for the pointcloud.
     */
    void create_listeners(const std::string &pose_topic_name, const std::string &pointcloud_topic_name);

    /**
     * Setup the publisher to publish skeletons.
     *
     * @param [in] skeleton_topic_name Name of the topic to publish the skeletons on.
     */
    void create_publisher(const std::string &skeleton_topic_name);
    
    void make_sure_window_boundaries_set(const PointCloud::ConstPtr &point_cloud);

    bool no_pose_found(const tfpose_ros::Persons::ConstPtr &persons_msg);

    void process_persons_to_skeletons(const tfpose_ros::Persons::ConstPtr &persons_msg,
                                      PointCloud::ConstPtr point_cloud);
};

#endif
