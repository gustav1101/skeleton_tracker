#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/Persons.h>
#include <ros/ros.h>
#include <skeleton3d/Skeleton3d.h>

class PointProjector
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber skeleton_subscriber_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher skeleton3d_publisher_;
    std::vector<tfpose_ros::Person> latest_tf_skeletons_;
    //std::vector<skeleton3d::Skeleton3d> created_3d_skeleton_;

    int output_counter_ = 0;
    
    void construct_3d_skeleton(const sensor_msgs::PointCloud2);
    void save_skeletons(const tfpose_ros::Persons& persons_msg);
    geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int x, const int y);
    skeleton3d::Skeleton3d transform_skeleton_to_3d(const tfpose_ros::Person tf_person, const sensor_msgs::PointCloud2 point_cloud);
    
public:
    PointProjector();
    ~PointProjector();
};
