#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/Persons.h>
#include <ros/ros.h>
#include <skeleton3d/Skeleton3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class PointProjector
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber skeleton_subscriber_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher skeleton3d_publisher_;
    tfpose_ros::Persons latest_persons_;
    
    void construct_3d_skeleton(const PointCloud::ConstPtr& point_cloud);
    void save_skeletons(const tfpose_ros::Persons& persons_msg);
    boost::optional<skeleton3d::Skeleton3d> transform_skeleton_to_3d(int i, const PointCloud::ConstPtr& point_cloud);
    bool any_point_invalid(float x, float y, float z);
    
public:
    PointProjector();
    ~PointProjector();
};
