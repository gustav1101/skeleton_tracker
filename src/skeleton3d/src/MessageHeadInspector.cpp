#include "MessageHeadInspector.h"
#include "boost/date_time/posix_time/posix_time.hpp"

MessageHeadInspector::MessageHeadInspector() : image_transport_(node_handle_)
{
    image_subscriber_ = image_transport_.subscribe("/myxtion/rgb/image_raw", 1, &MessageHeadInspector::republish_once, this);
    skeleton_subscriber_ = node_handle_.subscribe("/pose_estimator/pose", 1, &MessageHeadInspector::read_skeleton_header, this);
    pointcloud_subscriber_ = node_handle_.subscribe("/myxtion/depth_registered/points", 1, &MessageHeadInspector::find_depth_corr_image, this);
    
    image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("/republished_cam/image_raw", 1);
    already_published_ = false;
}

void MessageHeadInspector::republish_once(const sensor_msgs::ImageConstPtr &image_msg)
{
    if (already_published_)
    {
        return;
    }
    already_published_ = true;
    image_publisher_.publish(image_msg);
    ROS_INFO("Image time: %s", MessageHeadInspector::get_time_string(image_msg->header.stamp).c_str());
    last_image_time_ = image_msg->header.stamp;
}

void MessageHeadInspector::find_depth_corr_image(const sensor_msgs::PointCloud2::ConstPtr &point_cloud)
{
    double allowed_diff = 0.2;
    ros::Duration diff = last_image_time_ - point_cloud->header.stamp;
    if( (diff < ros::Duration(0) && diff > ros::Duration(allowed_diff)) || (diff > ros::Duration(0) && diff < ros::Duration(allowed_diff)))
    {
        ROS_INFO("Point time: %s", MessageHeadInspector::get_time_string(point_cloud->header.stamp).c_str());
    }
    }

void MessageHeadInspector::read_skeleton_header(const tfpose_ros::Persons &persons_msg)
{
    ROS_INFO("Pose  time: %s", MessageHeadInspector::get_time_string(persons_msg.header.stamp).c_str());
}

std::string MessageHeadInspector::get_time_string(const ros::Time &timestamp)
{
    boost::posix_time::ptime my_posix_time = timestamp.toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    return iso_time_str;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MessageHeadInspector");
    MessageHeadInspector message_head_inspector;
    ros::Rate rate(0.25); 
    while(ros::ok())
    {
        message_head_inspector.already_published_ = false;
        ros::spinOnce();
        rate.sleep();
    }
}
