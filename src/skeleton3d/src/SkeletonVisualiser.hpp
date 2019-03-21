#ifndef SKELETONVISUALISER_HPP
#define SKELETONVISUALISER_HPP

#include <ros/ros.h>
#include <skeleton3d/Skeletons3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <vector>
#include <visualization_msgs/Marker.h>

/**
 * This class generates visible marker lines for skeletons that can be displayed in rviz.
 *
 * First, lines are generated by making sense of the individual body parts. This is done using
 * their ID since that indicates what the body part represents (0 for head, 1 for center torso
 * etc). Then consecutive lines are built, their marker properties set and sent to the publisher.
 */
class SkeletonVisualiser
{
public:
    /**
     * One Line of mulitple geometry_msgs::Point instances.
     */
    struct Line {
        std::vector<geometry_msgs::Point> points;
    };

    /**
     * Generate Line representing a part of the body (such as head, arms, ...).
     *
     * @param skeletons All skeletons that the body part lines should be generated for
     * @return Mulitple Lines that each represent a body segment.
     */
    static std::vector<SkeletonVisualiser::Line> generate_skeletons_lines(const std::vector<skeleton3d::Skeleton3d> &skeletons);
    
private:
    /* The following vectors are adjacency information for which body part IDs
     * connected to which other body parts. The numbers refer to the corresponding
     * body part IDs. Eventually, this will be used to draw lines between body
     * parts whose IDs are in one list.
     */
    static const std::vector<int> ADJACENCY_HEAD_TORSO_;
    static const std::vector<int> ADJACENCY_ARMS_;
    static const std::vector<int> ADJACENCY_HEAD_;
    static const std::vector<int> ADJACENCY_LEGS_;

    /**
     * Generate all the lines for one skeleton.
     */
    static std::vector<Line> create_skeleton_lines(const skeleton3d::Skeleton3d &skeleton);

    /**
     * Generate one body segment line.
     *
     * Iterate through one given adjacency list. For each pair of adjacent points find the
     * corresponding body parts. Add their coordinate points to the list of markers.
     * Only add consecutive lists of valid body part coordinate points to make sure the lines
     * are drawn properly later.
     */
    static std::vector<Line> construct_point_line(const std::vector<skeleton3d::BodyPart3d> &all_body_parts, const std::vector<int> &adjacency_list);

    /**
     * Identify and print the list that caused an error.
     *
     * For debugging purposes.
     */
    static void print_illegal_list_error(int first_list_item_id);
};

#endif
