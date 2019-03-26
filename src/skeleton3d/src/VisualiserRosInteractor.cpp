#include "VisualiserRosInteractor.hpp"
#include "exceptions.hpp"

using Line = SkeletonVisualiser::Line;

VisualiserRosInteractor::VisualiserRosInteractor()
{
    setup_topics();
    frame_id_ = get_param("~frame_id");
    ros::param::param<bool>("~decaying_lines", use_decaying_lines_, true);
}

void VisualiserRosInteractor::call_marker_construction(const skeleton3d::Skeletons3d::ConstPtr &skeletons_msg)
{
    std::vector<Line> all_consecutive_lines = SkeletonVisualiser::generate_skeletons_lines(skeletons_msg->skeletons);
    publish_skeleton_markers(all_consecutive_lines);
}

void VisualiserRosInteractor::publish_skeleton_markers(const std::vector<Line> &all_consecutive_lines)
{
    skeleton_consecutive_line_id_ = 0;
    for (const Line &consecutive_line : all_consecutive_lines)
    {
        if (consecutive_line.points.size() < 2 )
        {
            ROS_WARN("Empty Consecutive line");
            continue;
        }
        visualization_msgs::Marker line_markers;
        set_marker_properties(line_markers);
        publish_line_markers(line_markers, consecutive_line);
        skeleton_consecutive_line_id_++;
    }
}

void VisualiserRosInteractor::publish_line_markers(visualization_msgs::Marker &line_markers,
                                                   const Line &consecutive_line)
{
    line_markers.points = consecutive_line.points;
    marker_publisher_.publish(line_markers);
}

void VisualiserRosInteractor::set_marker_properties(visualization_msgs::Marker &line_marker)
{
    line_marker.id = skeleton_consecutive_line_id_;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "skeleton_to_3d_vis";
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.scale.x = 0.02;
    line_marker.color.b = 1.0;
    line_marker.color.a = 1.0;
    if (use_decaying_lines_)
    {
        line_marker.lifetime = ros::Duration(0.5);
    }
}

void VisualiserRosInteractor::setup_topics()
{
    skeleton_subscriber_ = node_handle_.subscribe(
        get_param("~input_skeleton"),
        10,
        &VisualiserRosInteractor::call_marker_construction,
        this);
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(
        get_param("~output_marker"),
        10);
}

std::string VisualiserRosInteractor::get_param(const std::string &param_name)
{
    std::string param;
    if (!ros::param::get(param_name, param))
    {
        ROS_ERROR("Missing Node Parameter %s", param_name.c_str());
        throw skeleton_exceptions::LackingRosParameter(param_name);
    }
    return param;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d_vis");
    try
    {
        VisualiserRosInteractor interactor;
        ros::spin();
    } catch (skeleton_exceptions::LackingRosParameter &e)
    {
        ROS_ERROR("Missing Node Parameter %s", e.get_info().c_str());
        return 1;
    }
}
