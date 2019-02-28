#include "PointProjector.h"
#include <tfpose_ros/BodyPartElm.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeletons3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include "exceptions.h"

PointProjector::PointProjector()
{
    std::string pose_topic_name = get_param("~input_pose");
    std::string pointcloud_topic_name = get_param("~input_pointcloud");
    std::string skeleton_topic_name = get_param("~output_skeleton");
    
    skeleton_subscriber_ = new message_filters::Subscriber<tfpose_ros::Persons>(
        node_handle_,
        pose_topic_name,
        INPUT_QUEUE_SIZE_);
    pointcloud_subscriber_ = new message_filters::Subscriber<PointCloud>(
        node_handle_,
        pointcloud_topic_name,
        INPUT_QUEUE_SIZE_);
    skeleton3d_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>(
        skeleton_topic_name,
        50);
    
    message_synchronizer_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(INPUT_QUEUE_SIZE_), *skeleton_subscriber_, *pointcloud_subscriber_);
    message_synchronizer_->registerCallback(
        boost::bind(&PointProjector::construct_3d_skeleton, this, _1, _2));
}

PointProjector::~PointProjector() {
    delete message_synchronizer_;
    delete skeleton_subscriber_;
    delete pointcloud_subscriber_;
}


void PointProjector::construct_3d_skeleton(const tfpose_ros::Persons::ConstPtr &persons_msg, const PointCloud::ConstPtr &point_cloud)
{
    std::vector<skeleton3d::Skeleton3d> skeletons_3d;
    
    // For each skeleton created by openpose: transform into 3d skeleton
    for(const tfpose_ros::Person &person : persons_msg->persons)
    {
        unsigned int image_width = persons_msg->image_w;
        unsigned int image_height = persons_msg->image_h;
        
        boost::optional<skeleton3d::Skeleton3d> skeleton = transform_skeleton_to_3d(person, point_cloud, image_width, image_height);
        if (skeleton)
        {
            skeletons_3d.push_back(*skeleton);
        }
    }
    if (skeletons_3d.size() == 0)
    {
        return;
    }
    skeleton3d::Skeletons3d skeletons_msg;
    skeletons_msg.skeletons = skeletons_3d;
    skeletons_msg.header.stamp = ros::Time::now();
    skeletons_msg.header.frame_id = "/myxtion_depth_frame";

    //ROS_INFO("\npcloud time: %s\npose   time: %s", PointProjector::get_time_string(pcl_conversions::fromPCL(point_cloud->header.stamp)).c_str(), PointProjector::get_time_string(persons_msg->header.stamp).c_str());

    skeleton3d_publisher_.publish(skeletons_msg);
}

std::string PointProjector::get_time_string(const ros::Time &timestamp)
{
    boost::posix_time::ptime my_posix_time = timestamp.toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    return iso_time_str;
}


boost::optional<skeleton3d::Skeleton3d> PointProjector::transform_skeleton_to_3d(const tfpose_ros::Person &person, const PointCloud::ConstPtr &point_cloud, unsigned int image_width, unsigned int image_height)
{
    skeleton3d::Skeleton3d skeleton;
    skeleton.body_parts = std::vector<skeleton3d::BodyPart3d>(18);

    // First set all body parts to invalid (meaning not found), then overwrite with
    // data from the actual tfpose body parts

    for(skeleton3d::BodyPart3d &body_part : skeleton.body_parts)
    {
        body_part.part_is_valid = false;
    }

    for(const tfpose_ros::BodyPartElm &body_part_2d : person.body_part)
    {
        skeleton3d::BodyPart3d &body_part_3d = skeleton.body_parts.at(body_part_2d.part_id);
        body_part_3d.part_is_valid = true;
        body_part_3d.part_id = body_part_2d.part_id;
        int pos_x = body_part_2d.x * image_width;
        int pos_y = body_part_2d.y * image_height;
        
        pcl::PointXYZ point = point_cloud->at(pos_x, pos_y);
        
        geometry_msgs::Point geometry_point;
        if (any_point_invalid(point.x, point.y, point.z))
        {
            // Continue might be better here instead of omitting the whole skeleton.
            ROS_WARN("NAN encountered, discarding message");
            return boost::none;
        }
        geometry_point.x = point.x;
        geometry_point.y = point.y;
        geometry_point.z = point.z;
        body_part_3d.point = geometry_point;
        body_part_3d.confidence = body_part_2d.confidence;
    }
    return skeleton;
}

inline bool PointProjector::any_point_invalid(float x, float y, float z)
{
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
}

std::string PointProjector::get_param(const std::string &param_name)
{
    std::string param;
    if (!ros::param::get(param_name, param))
    {
        throw skeleton_exceptions::LackingRosParameter(param_name);
    }
    return param;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d");
    try
    {
        PointProjector point_projector;
        ros::spin();
    } catch (skeleton_exceptions::LackingRosParameter &e)
    {
        ROS_ERROR("Missing Node Parameter %s", e.get_info().c_str());
        return 1;
    }
}
