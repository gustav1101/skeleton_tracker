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

private:
    std::vector<SkeletonInformation> skeletons_masterlist_;
    const double POSITION_TOLERANCE_;
    
    optional<SkeletonInformation&> find_skeleton_in_list(const Skeleton &new_skeleton);
    Point get_skeleton_center_position(const std::vector<BodyPart> &body_parts);
    Point get_skeleton_center_position(const std::vector<BodyPartInformation> &body_parts);
    bool is_same_skeleton(const Skeleton &skel1, const SkeletonInformation &skel2);
    void merge_skeleton(const Skeleton &new_skeleton, SkeletonInformation &exisiting_skeleton, const ros::Time &timestamp);
    void insert_skeleton(const Skeleton &new_skeleton, const ros::Time &timestamp);
    double distance_between_points(const Point &point1, const Point &point2);
    bool should_update(const BodyPartInformation &body_part_new, const BodyPartInformation &body_part_existing);
    Skeleton simple_skeleton_from_skeletoninfo(const SkeletonInformation &skeleton_info);
    
public:
SkeletonRepository(double position_tolerance) : POSITION_TOLERANCE_(position_tolerance) {}
    ~SkeletonRepository();
    void update_skeletons(const skeleton3d::Skeletons3d::ConstPtr &skeletons_msg);
    std::vector<Skeleton> get_skeleton_masterlist();
};
