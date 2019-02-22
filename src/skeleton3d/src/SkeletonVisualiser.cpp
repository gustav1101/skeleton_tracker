#include "SkeletonVisualiser.h"
const std::vector<int> SkeletonVisualiser::ADJACENCY_HEAD_TORSO_ = {0, 1};
const std::vector<int> SkeletonVisualiser::ADJACENCY_ARMS_ = {4, 3, 2, 1, 5, 6, 7};
const std::vector<int> SkeletonVisualiser::ADJACENCY_HEAD_ = {16, 14, 0, 15, 17};
const std::vector<int> SkeletonVisualiser::ADJACENCY_LEGS_ = {10, 9, 8, 1, 11, 12, 13};

SkeletonVisualiser::SkeletonVisualiser()
{
    skeleton3d_subscriber_ = node_handle_.subscribe("skeleton_to_3d/skeletons", 10, &SkeletonVisualiser::build_3d_skeletons, this);
    skeleton3d_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("skeleton_to_3d_vis/skeleton_vis", 10);
}

SkeletonVisualiser::~SkeletonVisualiser()
{

}

void SkeletonVisualiser::build_3d_skeletons(const skeleton3d::Skeletons3d skeletons)
{
    skeleton_consecutive_line_id_ = 0;
    for(skeleton3d::Skeleton3d skeleton : skeletons.skeletons)
    {
        std::vector<std::vector<geometry_msgs::Point>> visualised_skeleton = create_markers(skeleton);
        publish_skeleton_markers(visualised_skeleton);
    }
}

std::vector<std::vector<geometry_msgs::Point>> SkeletonVisualiser::create_markers(const skeleton3d::Skeleton3d skeleton)
{
    // Try to iterate through list of markers.
    std::vector<std::vector<geometry_msgs::Point>> marker_lines;

    // Torso-Head:
    marker_lines = construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_HEAD_TORSO_);
    
    
    
    std::vector<std::vector<geometry_msgs::Point>> temp_lines =
        construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_ARMS_);
    marker_lines.insert(marker_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();

    temp_lines = construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_HEAD_);
    marker_lines.insert(marker_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();
    
    temp_lines = construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_LEGS_);
    marker_lines.insert(marker_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();
    
    return marker_lines;
}

/*
 * Iterate through the adjacency list. For each pair of adjacent points find the
 * corresponding body parts. Add their coordinate points to the list of markers.
 * Only add consecutive lists of valid body part coordinate points to make sure the lines
 * are drawn properly later.
 */
std::vector<std::vector<geometry_msgs::Point>> SkeletonVisualiser::construct_point_line(const std::vector<skeleton3d::BodyPart3d> &all_body_parts, std::vector<int> adjacency_list)
{
    std::vector<std::vector<geometry_msgs::Point>> all_consecutive_lines;
    std::vector<geometry_msgs::Point> consecutive_line;
    
    if ((all_body_parts.size() < 2) || (adjacency_list.size() < 2))
    {
        std::string part;
        switch (adjacency_list.at(0))
        {
        case 0: part="head-torso"; break;
        case 4: part="arms"; break;
        case 10: part="legs"; break;
        case 16: part="head"; break;
        default: ROS_WARN("Invalid Adjacency list"); return all_consecutive_lines;
        }
        ROS_WARN("Discarding message for %s!", part.c_str());
        return all_consecutive_lines;
    }

    for(auto adjacency_iterator = adjacency_list.begin(); adjacency_iterator != adjacency_list.end()-1; adjacency_iterator++)
    {
        const skeleton3d::BodyPart3d &cur_part = all_body_parts.at(*adjacency_iterator);
        const skeleton3d::BodyPart3d &next_part = all_body_parts.at(*std::next(adjacency_iterator));
        
        if(cur_part.part_is_valid && next_part.part_is_valid)
        {
            // Both this and the next point are valid, so draw a line between them.

            //If the consecutive line has no starting point yet: create one first.
            if (consecutive_line.size() == 0)
            {
                consecutive_line.push_back(geometry_msgs::Point(cur_part.point));
            }
            consecutive_line.push_back(geometry_msgs::Point(next_part.point));
        } else
        {
            // Either this point, the next point, or both are invalid.
            // If there has been points put on the consecutive line:
            // Add that line to the marker points, then reset the consecutive list.
            if (consecutive_line.size() > 0 )
            {
                all_consecutive_lines.push_back(consecutive_line);
                consecutive_line.clear();
            }
        }
    }
    // If the last two body points were valid we still have an ongoing consecutive line here.
    if (consecutive_line.size() > 0)
    {
        all_consecutive_lines.push_back(consecutive_line);
    }
    return all_consecutive_lines;
}

void SkeletonVisualiser::publish_skeleton_markers(const std::vector<std::vector<geometry_msgs::Point>> all_consecutive_lines)
{
    for (const std::vector<geometry_msgs::Point> &consecutive_line : all_consecutive_lines)
    {
        visualization_msgs::Marker skeleton_markers;
        skeleton_markers.header.frame_id = "/myxtion_depth_optical_frame";
        skeleton_markers.header.stamp = ros::Time::now();
        skeleton_markers.ns = "skeleton_to_3d_vis";
        skeleton_markers.action = visualization_msgs::Marker::ADD;
        skeleton_markers.pose.orientation.w = 1.0;
    
        skeleton_markers.id = skeleton_consecutive_line_id_++;
    
        skeleton_markers.type = visualization_msgs::Marker::LINE_STRIP;
    
        skeleton_markers.scale.x = 0.02;
    
        skeleton_markers.color.b = 1.0;
        skeleton_markers.color.a = 1.0;

        skeleton_markers.points = consecutive_line;

        skeleton3d_marker_publisher_.publish(skeleton_markers);

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d_vis");
    SkeletonVisualiser vis;
    ros::spin();
}
