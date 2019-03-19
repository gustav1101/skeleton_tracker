#include "SkeletonCreatorRosInteractor.h"
#include <skeleton3d/Skeletons3d.h>
#include "exceptions.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<tfpose_ros::Persons, PointCloud>;

void SkeletonCreatorRosInteractor::generate_skeleton(const tfpose_ros::Persons::ConstPtr &persons_msg, const PointCloud::ConstPtr &point_cloud)
{
    // On the first call of this method: Set the window size properties on the skeleton creator
    if (!window_boundaries_set_)
    {
        skeleton_creator_.set_image_size(point_cloud->width, point_cloud->height);
        window_boundaries_set_ = true;
    }
    std::vector<skeleton3d::Skeleton3d> skeletons = skeleton_creator_.generate_skeleton(persons_msg->persons, point_cloud);
    publish_skeletons(skeletons);
}

void SkeletonCreatorRosInteractor::publish_skeletons(std::vector<skeleton3d::Skeleton3d> skeletons)
{
    skeleton3d::Skeletons3d skeletons_msg;
    skeletons_msg.header.stamp = ros::Time::now();
    skeletons_msg.header.frame_id = "/" + camera_name_ + "_depth_frame";
    skeletons_msg.skeletons = skeletons;
    
    skeleton_publisher_.publish(skeletons_msg);
}

SkeletonCreatorRosInteractor::RosParams SkeletonCreatorRosInteractor::read_params()
{
    std::string pose_topic_name = get_param("~input_pose");
    std::string pointcloud_topic_name = get_param("~input_pointcloud");
    std::string skeleton_topic_name = get_param("~output_skeleton");
    std::string camera_name = get_param("~camera_name");
    double frame_offset;
    ros::param::param<double>("~x_frame_offset", frame_offset, 0.0);
    int scatter_distance;
    ros::param::param<int>("~scatter_distance", scatter_distance, 6);

    return RosParams {
        .pose_topic_name = pose_topic_name,
            .pointcloud_topic_name = pointcloud_topic_name,
            .skeleton_topic_name = skeleton_topic_name,
            .camera_name = camera_name,
            .scatter_distance = scatter_distance,
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
