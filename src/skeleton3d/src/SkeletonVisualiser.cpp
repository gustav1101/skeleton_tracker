#include "SkeletonVisualiser.hpp"
#include "exceptions.hpp"

/* The following vectors are adjacency information for which body part IDs
 * connected to which other body parts. The numbers refer to the corresponding
 * body part IDs. Eventually, this will be used to draw lines between body
 * parts whose IDs are in one list.
 */
const std::vector<int> SkeletonVisualiser::ADJACENCY_HEAD_TORSO_ = {0, 1};
const std::vector<int> SkeletonVisualiser::ADJACENCY_ARMS_ = {4, 3, 2, 1, 5, 6, 7};
const std::vector<int> SkeletonVisualiser::ADJACENCY_HEAD_ = {16, 14, 0, 15, 17};
const std::vector<int> SkeletonVisualiser::ADJACENCY_LEGS_ = {10, 9, 8, 1, 11, 12, 13};

using Line = SkeletonVisualiser::Line;

std::vector<Line> SkeletonVisualiser::generate_skeletons_lines(const std::vector<skeleton3d::Skeleton3d> &skeletons)
{
    std::vector<Line> skeleton_lines;
    for(const skeleton3d::Skeleton3d &skeleton : skeletons)
    {
        std::vector<Line> new_lines = create_skeleton_lines(skeleton);
        skeleton_lines.insert( skeleton_lines.end(), new_lines.begin(), new_lines.end() );
    }
    return skeleton_lines;
}

std::vector<Line> SkeletonVisualiser::create_skeleton_lines(const skeleton3d::Skeleton3d &skeleton)
{
    // Torso-Head connection:
    std::vector<Line> skeleton_lines = construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_HEAD_TORSO_);
    
    // Both arms, connected at torso point:
    std::vector<Line> temp_lines =
        construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_ARMS_);
    skeleton_lines.insert(skeleton_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();

    // Head:
    temp_lines = construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_HEAD_);
    skeleton_lines.insert(skeleton_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();

    // Both legs and contact point to torso (as well as hip):
    temp_lines = construct_point_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_LEGS_);
    skeleton_lines.insert(skeleton_lines.end(),temp_lines.begin(), temp_lines.end());
    
    return skeleton_lines;
}

/*
 * Iterate through one given adjacency list. For each pair of adjacent points find the
 * corresponding body parts. Add their coordinate points to the list of markers.
 * Only add consecutive lists of valid body part coordinate points to make sure the lines
 * are drawn properly later.
 */
std::vector<Line> SkeletonVisualiser::construct_point_line(const std::vector<skeleton3d::BodyPart3d> &body_parts, const std::vector<int> &adjacency_list)
{
    std::vector<Line> all_consecutive_lines;
    Line consecutive_line;

    // If the given vector of body parts is smaller than 2 we can't draw lines, so stop here.
    // The checking for the adjacency list is just to make really, really sure we're not
    // attempting to access illegal memory and should never happen since adj. lists are
    // constant.
    if ((body_parts.size() < 2) || (adjacency_list.size() < 2))
    {
        print_illegal_list_error(adjacency_list.at(0));
        return all_consecutive_lines;
    }

    /* Create consecutive lines between body parts that are adjacent (according to adj. list).
     * Body parts that have not been recognised by the network (and are marked as invalid)
     * will stop the current consecutive line. A new one will eventually be started once the next
     * valid body part is encountered.
     */
    for(auto adjacency_iterator = adjacency_list.begin(); adjacency_iterator != adjacency_list.end()-1; adjacency_iterator++)
    {
        const skeleton3d::BodyPart3d &cur_part = body_parts.at(*adjacency_iterator);
        const skeleton3d::BodyPart3d &next_part = body_parts.at(*std::next(adjacency_iterator));
        
        if(cur_part.part_is_valid && next_part.part_is_valid)
        {
            // Both this and the next point are valid, so draw a line between them.

            //If the consecutive line has no starting point yet: create one first.
            if (consecutive_line.points.size() == 0)
            {
                consecutive_line.points.push_back(geometry_msgs::Point(cur_part.point));
            }
            consecutive_line.points.push_back(geometry_msgs::Point(next_part.point));
        } else
        {
            // Either this point, the next point, or both are invalid.
            // If there has been points put on the consecutive line:
            // Add that line to the marker points, then reset the consecutive list.
            if (consecutive_line.points.size() > 0 )
            {
                all_consecutive_lines.push_back(consecutive_line);
                consecutive_line.points.clear();
            }
        }
    }
    // If the last two body points were valid we still have an ongoing consecutive line here,
    // in which case it needs to be added to the vector holding all consecutive lines
    if (consecutive_line.points.size() > 0)
    {
        all_consecutive_lines.push_back(consecutive_line);
    }
    
    return all_consecutive_lines;
}

void SkeletonVisualiser::print_illegal_list_error(int first_list_item_id)
{
    // For debugging purposes: Find out which adjacency list is discarded here
    std::string part;
    switch (first_list_item_id)
    {
    case 0: part="head-torso"; break;
    case 4: part="arms"; break;
    case 10: part="legs"; break;
    case 16: part="head"; break;
    default: ROS_WARN("illegal adjacency list");
    }
    ROS_WARN("Discarding message for %s!", part.c_str());
}

