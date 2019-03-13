#ifndef VISUALISERROSINTERACTOR_H
#define VISUALISERROSINTERACTOR_H

#include <ros/ros.h>
#include "SkeletonVisualiser.h"
#include <skeleton3d/Skeletons3d.h>
#include <skeleton3d/Skeleton3d.h>


class VisualiserRosInteractor {

private:
    //using Skeletons = skeleton3d::Skeletons3d;
    using Line = SkeletonVisualiser::Line;
    
    ros::NodeHandle node_handle_;
    ros::Subscriber skeleton_subscriber_;
    ros::Publisher marker_publisher_;
    std::string camera_name_;
    int skeleton_consecutive_line_id_;
    bool use_decaying_lines_;
    
    void setup_topics();
    void call_marker_construction(const skeleton3d::Skeletons3d::ConstPtr &skeletons);
    std::string get_param(const std::string &param_name);
    void publish_skeleton_markers(const std::vector<Line> &all_consecutive_lines);
    void publish_line_markers(visualization_msgs::Marker &line_markers, const Line &consecutive_line);
    void set_marker_properties(visualization_msgs::Marker &line_marker);
    
public:
    VisualiserRosInteractor();

};

#endif
