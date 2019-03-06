#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <boost/optional.hpp>

class SkeletonRepository {
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
    template<class T> using optional = boost::optional<T>;

private:
    std::vector<Skeleton> skeletons_masterlist_;
    const double POSITION_TOLERANCE_;
    
    optional<Skeleton&> find_skeleton_in_list(const Skeleton &new_skeleton);
    Point get_skeleton_center_position(const std::vector<BodyPart> &body_parts);
    bool is_same_skeleton(const Skeleton &skel1, const Skeleton &skel2);
    void merge_skeleton(const Skeleton &new_skeleton, Skeleton &exisiting_skeleton);
    void insert_skeleton(const Skeleton &new_skeleton);
    double distance_between_points(const Point &point1, const Point &point2);
    bool should_update(const BodyPart &body_part_new, const BodyPart &body_part_existing);
public:
SkeletonRepository(double position_tolerance) : POSITION_TOLERANCE_(position_tolerance) {}
    ~SkeletonRepository();
    void update_skeletons(const std::vector<Skeleton> &skeletons);
    std::vector<Skeleton> get_skeleton_masterlist();
};
