#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/Persons.h>
#include <ros/ros.h>
#include <skeleton3d/Skeleton3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<tfpose_ros::Persons, PointCloud> MySyncPolicy;

class SkeletonCreator
{
private:
    ros::NodeHandle node_handle_;
    message_filters::Subscriber<tfpose_ros::Persons> *skeleton_subscriber_;
    message_filters::Subscriber<PointCloud> *pointcloud_subscriber_;
    message_filters::Synchronizer<MySyncPolicy> *message_synchronizer_;
    ros::Publisher skeleton3d_publisher_;

    int POINT_FINDER_SCATTER_DISTANCE_ = 10;
    static const int INPUT_QUEUE_SIZE_ = 120;

    void construct_3d_skeleton(const tfpose_ros::Persons::ConstPtr &persons_msg, const PointCloud::ConstPtr &point_cloud);
    void save_skeletons(const tfpose_ros::Persons& persons_msg);
    boost::optional<skeleton3d::Skeleton3d> transform_skeleton_to_3d(const tfpose_ros::Person &person, const PointCloud::ConstPtr &point_cloud, unsigned int image_width, unsigned int image_height);
    bool any_coordinate_invalid(float x, float y, float z);
    static std::string get_time_string(const ros::Time &timestamp);
    std::string get_param(const std::string &param_name);
    
    
public:
    SkeletonCreator();
    ~SkeletonCreator();
};
