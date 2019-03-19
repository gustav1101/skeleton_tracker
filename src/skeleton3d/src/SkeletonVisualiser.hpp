#ifndef SKELETONVISUALISER_HPP
#define SKELETONVISUALISER_HPP

#include <ros/ros.h>
#include <skeleton3d/Skeletons3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <vector>
#include <visualization_msgs/Marker.h>

class SkeletonVisualiser
{
public:
    struct Line {
        std::vector<geometry_msgs::Point> points;
    };//SkeletonVisualiser();
    //~SkeletonVisualiser();
    static std::vector<SkeletonVisualiser::Line> generate_skeletons_lines(const std::vector<skeleton3d::Skeleton3d> &skeletons);

    
    
private:
    static const std::vector<int> ADJACENCY_HEAD_TORSO_;
    static const std::vector<int> ADJACENCY_ARMS_;
    static const std::vector<int> ADJACENCY_HEAD_;
    static const std::vector<int> ADJACENCY_LEGS_;
    
    
    static std::vector<Line> create_skeleton_lines(const skeleton3d::Skeleton3d &skeleton);
    static std::vector<Line> construct_point_line(const std::vector<skeleton3d::BodyPart3d> &all_body_parts, const std::vector<int> &adjacency_list);
    static void print_illegal_list_error(int first_list_item_id);
    

};

#endif
