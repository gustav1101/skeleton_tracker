#include "SkeletonCreatorRosInteractor.hpp"
#include <skeleton3d/Skeletons3d.h>
#include "exceptions.hpp"
#include "FilterStatus.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using ApproximateTimePolicy =
    message_filters::sync_policies::ApproximateTime<tfpose_ros::Persons, PointCloud>;


void SkeletonCreatorRosInteractor::generate_skeleton(
    const tfpose_ros::Persons::ConstPtr &persons_msg,
    const PointCloud::ConstPtr &point_cloud)
{
    // On the first call of this method: Set the window size properties on the skeleton creator
    make_sure_window_boundaries_set(point_cloud);
    process_persons_to_skeletons(persons_msg, point_cloud);
}

void SkeletonCreatorRosInteractor::publish_skeletons(std::vector<skeleton3d::Skeleton3d> skeletons)
{
    skeleton3d::Skeletons3d skeletons_msg;
    skeletons_msg.header.stamp = ros::Time::now();
    skeletons_msg.header.frame_id = frame_id_;
    skeletons_msg.skeletons = skeletons;
    
    skeleton_publisher_.publish(skeletons_msg);
}

SkeletonCreatorRosInteractor::RosParams SkeletonCreatorRosInteractor::read_params()
{
    std::string pose_topic_name = get_param("~input_pose");
    std::string pointcloud_topic_name = get_param("~input_pointcloud");
    std::string skeleton_topic_name = get_param("~output_skeleton");
    std::string frame_id = get_param("~frame_id");
    double frame_offset;
    ros::param::param<double>("~x_frame_offset", frame_offset, 0.0);
    int scatter_step_distance;
    ros::param::param<int>("~scatter_step_distance", scatter_step_distance, 2);
    int scatter_steps;
    ros::param::param<int>("~scatter_steps", scatter_steps, 4);
    int number_of_messages_to_discard;
    ros::param::param<int>("~number_of_messages_to_discard", number_of_messages_to_discard, 10);
    int number_of_calibration_messages;
    ros::param::param<int>("~number_of_calibration_messages",
                           number_of_calibration_messages,
                           20);

    return RosParams {
        .pose_topic_name = pose_topic_name,
            .pointcloud_topic_name = pointcloud_topic_name,
            .skeleton_topic_name = skeleton_topic_name,
            .frame_id = frame_id,
            .scatter_step_distance = scatter_step_distance,
            .scatter_steps = scatter_steps,
            .number_of_messages_to_discard = number_of_messages_to_discard,
            .number_of_calibration_messages = number_of_calibration_messages,
            .x_frame_offset = frame_offset};
}

std::string SkeletonCreatorRosInteractor::get_param(const std::string &param_name)
{
    std::string param;
    if (!ros::param::get(param_name, param))
    {
        throw skeleton_exceptions::LackingRosParameter(param_name);
    }
    return param;
}

void SkeletonCreatorRosInteractor::create_publisher(const std::string &skeleton_topic_name)
{
    skeleton_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>(
        skeleton_topic_name,
        50);
    filtered_cloud_publisher_ = node_handle_.advertise<PointCloud>(
        "filtered_cloud",
        50);

    negative_cloud_publisher_ = node_handle_.advertise<PointCloud>(
        "negative_cloud",
        20);
}

void SkeletonCreatorRosInteractor::make_sure_window_boundaries_set(const PointCloud::ConstPtr &point_cloud)
{
    if (!window_boundaries_set_)
    {
        skeleton_creator_.set_image_size(point_cloud->width, point_cloud->height);
        window_boundaries_set_ = true;
    }
}

bool SkeletonCreatorRosInteractor::no_pose_found(const tfpose_ros::Persons::ConstPtr &persons_msg)
{
    return (persons_msg->persons.size() == 0);
}

void SkeletonCreatorRosInteractor::calibrate_filter(const PointCloud::ConstPtr &point_cloud)
{
    static_cloud_filter_.calibrate_filter(point_cloud);
    if (static_cloud_filter_.get_filter_status() == pointcloud_filter_status::Status::ready)
    {
        switch_to_skeleton_generation();
    }
}

void SkeletonCreatorRosInteractor::switch_to_skeleton_generation()
{
    pointcloud_subscriber_for_calibration_.shutdown();
    message_synchronizer_.registerCallback(
        boost::bind(&SkeletonCreatorRosInteractor::generate_skeleton, this, _1, _2));
    create_publisher(skeleton_topic_name_);
}

void SkeletonCreatorRosInteractor::process_persons_to_skeletons(
    const tfpose_ros::Persons::ConstPtr &persons_msg,
    const PointCloud::ConstPtr point_cloud)
{
    PointCloud filtered_cloud = *point_cloud;
    static_cloud_filter_.pass_filter(filtered_cloud);
    filtered_cloud_publisher_.publish(filtered_cloud);
    negative_cloud_publisher_.publish(static_cloud_filter_.get_negative());
    
    if ( no_pose_found(persons_msg) )
    {
        // ROS_WARN("Found Message with 0 Persons. Maybe resolution is set too low?");
        return;
    }
    
    std::vector<skeleton3d::Skeleton3d> skeletons =
        skeleton_creator_.generate_skeletons(persons_msg->persons, filtered_cloud);
    
    publish_skeletons(skeletons);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d");
    try
    {
        SkeletonCreatorRosInteractor interactor(SkeletonCreatorRosInteractor::read_params());
        ros::spin();
    } catch (skeleton_exceptions::LackingRosParameter &e)
    {
        ROS_ERROR("Missing Node Parameter %s", e.get_info().c_str());
        return 1;
    }
}
