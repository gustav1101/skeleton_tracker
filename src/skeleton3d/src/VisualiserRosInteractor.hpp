#ifndef VISUALISERROSINTERACTOR_HPP
#define VISUALISERROSINTERACTOR_HPP

#include <ros/ros.h>
#include "SkeletonVisualiser.hpp"
#include <skeleton3d/Skeletons3d.h>
#include <skeleton3d/Skeleton3d.h>

/**
 * Ros Interactor for the SkeletonVisualiser.
 *
 * Handles all publishing and subscribing. Node parameters as follows:
 * Name           | Type   | Default  | Description
 * -------------- | ------ | -------- | ----
 * frame_id       | String | Required | Name of TF frame that markers should be relative to
 * decaying_lines | Bool   | true     | Lines live for 0.5 seconds if true and do not decay at all if false
 * input_skeleton | String | Required | Name of skeleton topic
 * output_marker  | String | Required | Name of marker topic
 *
 * @note This class is designed to work with the RepositoryRosinteractor. It can also be used with
 *       only one sensor directly sending it's information to this class as long as that sensor
 *       is the only one publishing on that topic.
 */
class VisualiserRosInteractor {

public:
    /**
     * Constructor also sets up all topics and starts working immediately upon initialisation.
     */
    VisualiserRosInteractor();

private:
    using Line = SkeletonVisualiser::Line;
    
    ros::NodeHandle node_handle_;
    ros::Subscriber skeleton_subscriber_;
    ros::Publisher marker_publisher_;
    /** Required for tf frame name set in the published marker messages */
    std::string frame_id_;
    /** Used to keep track of line id for the published markers. These do not have, and can not
     * have, inherent meaning, since the number of consecutive lines can vary drastically
     * even if the number of skeletons stays constant. (When skeletons are only partially
     * recognised and normally consecutive lines need to be broken up into multiple lines.)*/
    int skeleton_consecutive_line_id_;
    /** Sets the rviz Marker Lifetime to 0.5 seconds if true */
    bool use_decaying_lines_;

    /**
     * Setup subscriber and advertise publisher
     */
    void setup_topics();

    /**
     * Callback method for constructing markers and publishing them.
     *
     * @param skeletons All skeletons to publish markers for.
     */
    void call_marker_construction(const skeleton3d::Skeletons3d::ConstPtr &skeletons);

    /**
     * Publish all skeleton markers. Does not actually publish, refers to publish_line_markers()
     * instead.
     */
    void publish_skeleton_markers(const std::vector<Line> &all_consecutive_lines);

    /**
     * Set line marker properties, such as color and scale.
     */
    void set_marker_properties(visualization_msgs::Marker &line_marker);

    /**
     * Publish one line of markers.
     *
     * @param line_markers Line markers for this line with all properties already set.
     * @param consecutive_line Points that are forming the line.
     */
    void publish_line_markers(visualization_msgs::Marker &line_markers,
                              const Line &consecutive_line);

    /**
     * Get an individual parameter.
     *
     * @param param_name Name of the parameter as set in the node.
     * @return Value of the parameter requested.
     * @throw skeleton_exceptions::LackingRosParameter If parameter has not been set.
     */
    std::string get_param(const std::string &param_name);
};

#endif
