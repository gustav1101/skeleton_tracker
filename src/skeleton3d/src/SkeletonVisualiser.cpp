#include "SkeletonVisualiser.hpp"
#include "exceptions.hpp"
#include <stdexcept>

using Line = SkeletonVisualiser::Line;
using BodyPart = skeleton3d::BodyPart3d;
using Skeleton = skeleton3d::Skeleton3d;
using Point = geometry_msgs::Point;

const std::vector<int> SkeletonVisualiser::ADJACENCY_HEAD_TORSO_ = {0, 1};
const std::vector<int> SkeletonVisualiser::ADJACENCY_ARMS_ = {4, 3, 2, 1, 5, 6, 7};
const std::vector<int> SkeletonVisualiser::ADJACENCY_HEAD_ = {16, 14, 0, 15, 17};
const std::vector<int> SkeletonVisualiser::ADJACENCY_LEGS_ = {10, 9, 8, 1, 11, 12, 13};

std::vector<Line> SkeletonVisualiser::generate_skeletons_lines(
    const std::vector<Skeleton> &skeletons)
{
    std::vector<Line> all_skeletons_lines;
    for(const Skeleton &skeleton : skeletons)
    {
        std::vector<Line> new_lines = create_skeleton_lines(skeleton);
        all_skeletons_lines.insert( all_skeletons_lines.end(),
                                    new_lines.begin(),
                                    new_lines.end() );
    }
    return all_skeletons_lines;
}

std::vector<Line> SkeletonVisualiser::create_skeleton_lines(const Skeleton &skeleton)
{
    // Torso-Head connection:
    std::vector<Line> skeleton_lines = construct_bodyparts_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_HEAD_TORSO_);
    
    // Both arms, connected at torso point:
    std::vector<Line> temp_lines =
        construct_bodyparts_line(skeleton.body_parts, SkeletonVisualiser::ADJACENCY_ARMS_);
    skeleton_lines.insert(skeleton_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();

    // Head:
    temp_lines = construct_bodyparts_line(skeleton.body_parts,
                                          SkeletonVisualiser::ADJACENCY_HEAD_);
    skeleton_lines.insert(skeleton_lines.end(),temp_lines.begin(), temp_lines.end());
    temp_lines.clear();

    // Both legs and contact point to torso (as well as hip):
    temp_lines = construct_bodyparts_line(skeleton.body_parts,
                                          SkeletonVisualiser::ADJACENCY_LEGS_);
    skeleton_lines.insert(skeleton_lines.end(),temp_lines.begin(), temp_lines.end());
    
    return skeleton_lines;
}

std::vector<Line> SkeletonVisualiser::construct_bodyparts_line(
    const std::vector<BodyPart> &body_parts,
    const std::vector<int> &adjacency_list)
{
    std::vector<Line> all_consecutive_lines;
    Line consecutive_line;

    // If the given vector of body parts is smaller than 2 we can't draw lines, so stop here.
    // The checking for the adjacency list is just to make really, really sure we're not
    // attempting to access illegal memory and should never happen since adj. lists are
    // constant.
    if ((body_parts.size() < 2) || (adjacency_list.size() < 2))
    {
        throw std::runtime_error("Cannot construct line: invalid list");
    }

    /* Create consecutive lines between body parts that are adjacent (according to adj. list).
     * Body parts that have not been recognised by the network (and are marked as invalid)
     * will stop the current consecutive line. A new one will eventually be started once the next
     * valid body part is encountered.
     */
    for(auto adjacency_iterator = adjacency_list.begin(); adjacency_iterator != adjacency_list.end()-1; adjacency_iterator++)
    {
        const BodyPart &cur_part = body_parts.at(*adjacency_iterator);
        const BodyPart &next_part = body_parts.at(*std::next(adjacency_iterator));
        
        add_bodyparts_to_line(cur_part, next_part, all_consecutive_lines, consecutive_line);
    }
    // If the last two body points were valid we still have an ongoing consecutive line here,
    // in which case it needs to be added to the vector holding all consecutive lines
    finish_line(all_consecutive_lines, consecutive_line);
    
    return all_consecutive_lines;
}

void SkeletonVisualiser::add_bodyparts_to_line(const BodyPart &cur_part,
                                               const BodyPart &next_part,
                                               std::vector<Line> &all_consecutive_lines,
                                               Line &consecutive_line)
{
    if(cur_part.part_is_valid && next_part.part_is_valid)
    {
        // Both this and the next point are valid, so draw a line between them.
        add_part_to_line(cur_part, next_part, consecutive_line);
    } else
    {
        // Either this point, the next point, or both are invalid. This ends the line.
        finish_line(all_consecutive_lines , consecutive_line);
    }
}

void SkeletonVisualiser::add_part_to_line(const BodyPart &cur_part,
                                          const BodyPart &next_part,
                                          Line &consecutive_line)
{
    //If the consecutive line has no starting point yet: create one first.
    if ( !line_has_started(consecutive_line) )
    {
        consecutive_line.points.push_back(cur_part.point);
    }
    consecutive_line.points.push_back(next_part.point);    
}

void SkeletonVisualiser::finish_line(std::vector<Line> &all_consecutive_lines,
                                     Line &consecutive_line)
{
    // If there have been points put on the consecutive line:
    // Add that line to the marker points, then reset the consecutive list.
    if ( line_is_valid(consecutive_line) )
    {
        all_consecutive_lines.push_back(consecutive_line);
        consecutive_line.points.clear();
    }
}    

bool SkeletonVisualiser::line_has_started(const Line &consecutive_line)
{
    return (consecutive_line.points.size() > 0);
}

bool SkeletonVisualiser::line_is_valid(const Line &consecutive_line)
{
    return (consecutive_line.points.size() > 1);
}
