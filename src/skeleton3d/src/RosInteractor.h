#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include "SkeletonCreator.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<tfpose_ros::Persons, PointCloud> MySyncPolicy;

class RosInteractor
{
private:
    static const int INPUT_QUEUE_SIZE_ = 30;
    SkeletonCreator skeleton_creator_;
    ros::NodeHandle node_handle_;
    message_filters::Subscriber<tfpose_ros::Persons> *skeleton_subscriber_;
    message_filters::Subscriber<PointCloud> *pointcloud_subscriber_;
    message_filters::Synchronizer<MySyncPolicy> *message_synchronizer_;
    ros::Publisher skeleton3d_publisher_;

    static std::string get_param(const std::string &param_name);
    void generate_skeleton(const tfpose_ros::Persons::ConstPtr &persons_msg, const PointCloud::ConstPtr &point_cloud);
    void create_listeners(const std::string &pose_topic_name, const std::string &pointcloud_topic_name);
    void create_publisher(const std::string &skeleton_topic_name);
    
public:
    struct RosParams{
        const std::string pose_topic_name;
        const std::string pointcloud_topic_name;
        const std::string skeleton_topic_name;
        const int scatter_distance;
        const double x_frame_offset;
    };
    
RosInteractor(const RosParams &params) : skeleton_creator_(params.scatter_distance, params.x_frame_offset)
    {
        create_listeners(params.pose_topic_name, params.pointcloud_topic_name);
        create_publisher(params.skeleton_topic_name);
    }
    ~RosInteractor();
    void publish_skeletons(std::vector<skeleton3d::Skeleton3d> skeletons);
    static RosParams read_params();
    
};
