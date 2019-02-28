#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <tfpose_ros/Persons.h>
#include <sensor_msgs/PointCloud2.h>

class MessageHeadInspector
{
private:
    ros::NodeHandle node_handle_;
    image_transport::Subscriber  image_subscriber_;
    ros::Subscriber skeleton_subscriber_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher image_publisher_;
    image_transport::ImageTransport image_transport_;
    ros::Time last_image_time_;
    
    void republish_once(const sensor_msgs::ImageConstPtr &image_msg);
    void read_skeleton_header(const tfpose_ros::Persons &persons_msg);
    void find_depth_corr_image(const sensor_msgs::PointCloud2::ConstPtr &point_cloud);
    static std::string get_time_string(const ros::Time &timestamp);
public:
    MessageHeadInspector();
    bool already_published_;
};
