#include <ros/ros.h>
#include <skeleton3d/Skeletons3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <vector>
#include <visualization_msgs/Marker.h>

class SkeletonVisualiser
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber skeleton3d_subscriber_;
    ros::Publisher skeleton3d_marker_publisher_;
    int skeleton_consecutive_line_id_;
    
    static const std::vector<int> ADJACENCY_HEAD_TORSO_;
    static const std::vector<int> ADJACENCY_ARMS_;
    static const std::vector<int> ADJACENCY_HEAD_;
    static const std::vector<int> ADJACENCY_LEGS_;
    
    
    void build_3d_skeletons(const skeleton3d::Skeletons3d &skeletons);
    static std::vector<std::vector<geometry_msgs::Point>> create_markers(const skeleton3d::Skeleton3d &skeleton);
    static std::vector<std::vector<geometry_msgs::Point>> construct_point_line(const std::vector<skeleton3d::BodyPart3d> &all_body_parts, const std::vector<int> &adjacency_list);
    void publish_skeleton_markers(const std::vector<std::vector<geometry_msgs::Point>> &all_consecutive_lines);
    std::string get_param(const std::string &param_name);
public:
    SkeletonVisualiser();
    ~SkeletonVisualiser();
};
