#ifndef SKELETONREPOSITORY_HPP
#define SKELETONREPOSITORY_HPP

#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include <boost/optional.hpp>
#include <ros/ros.h> // For time

class SkeletonRepository {
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
public:
    SkeletonRepository(double position_tolerance, double decay_strength) : POSITION_TOLERANCE_(position_tolerance), DECAY_STRENGTH_(decay_strength) {}
    ~SkeletonRepository();
    void update_skeletons(const skeleton3d::Skeletons3d::ConstPtr &skeletons_msg);
    std::vector<Skeleton> get_skeleton_masterlist();


private:
    struct BodyPartInformation
    {
        BodyPart body_part;
        ros::Time timestamp;
    };
    struct SkeletonInformation
    {
        std::vector<BodyPartInformation> body_part_information;
    };
    template<class T> using optional = boost::optional<T>;

    const double POSITION_TOLERANCE_;
    const double DECAY_STRENGTH_;
    std::vector<SkeletonInformation> skeletons_masterlist_;
    
    optional<SkeletonInformation&> find_skeleton_in_list(const Skeleton &new_skeleton);
    Point get_skeleton_center_position(const std::vector<BodyPart> &body_parts);
    Point get_skeleton_center_position(const std::vector<BodyPartInformation> &body_parts);
    Point calculate_center_point(const std::vector<const BodyPart*> body_parts);
    bool is_same_skeleton(const Skeleton &skel1, const SkeletonInformation &skel2);
    void merge_skeleton(const Skeleton &new_skeleton, SkeletonInformation &exisiting_skeleton, const ros::Time &timestamp);
    void insert_skeleton(const Skeleton &new_skeleton, const ros::Time &timestamp);
    double distance_between_points(const Point &point1, const Point &point2);
    bool should_update(const BodyPartInformation &body_part_new_info, const BodyPartInformation &body_part_existing_info);
    Skeleton simple_skeleton_from_skeletoninfo(const SkeletonInformation &skeleton_info);
    void decay_masterlist();
    bool decay_skeleton(SkeletonInformation &skeleton_information);
    bool decay_bodypart(BodyPartInformation &body_part_info);
};

#endif
