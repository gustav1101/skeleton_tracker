#include "RosInteractor.h"
#include <skeleton3d/Skeletons3d.h>
#include "exceptions.h"


RosInteractor::~RosInteractor()
{
    delete message_synchronizer_;
    delete skeleton_subscriber_;
    delete pointcloud_subscriber_;
}

void RosInteractor::generate_skeleton(const tfpose_ros::Persons::ConstPtr &persons_msg, const PointCloud::ConstPtr &point_cloud)
{
    skeleton_creator_.set_image_size(persons_msg->image_w, persons_msg->image_h);
    std::vector<skeleton3d::Skeleton3d> skeletons = skeleton_creator_.generate_skeleton(persons_msg->persons, point_cloud);
    publish_skeletons(skeletons);
}

void RosInteractor::publish_skeletons(std::vector<skeleton3d::Skeleton3d> skeletons)
{
    skeleton3d::Skeletons3d skeletons_msg;
    skeletons_msg.skeletons = skeletons;
    skeletons_msg.header.stamp = ros::Time::now();
    skeletons_msg.header.frame_id = "/myxtion_depth_frame";

    skeleton3d_publisher_.publish(skeletons_msg);
}

RosInteractor::RosParams RosInteractor::read_params()
{
    std::string pose_topic_name = get_param("~input_pose");
    std::string pointcloud_topic_name = get_param("~input_pointcloud");
    std::string skeleton_topic_name = get_param("~output_skeleton");
    double frame_offset;
    ros::param::param<double>("~x_frame_offset", frame_offset, 10.0);
    int scatter_distance;
    ros::param::param<int>("~scatter_distance", scatter_distance, 6);

    return RosParams {
        .pose_topic_name = pose_topic_name,
            .pointcloud_topic_name = pointcloud_topic_name,
            .skeleton_topic_name = skeleton_topic_name,
            .scatter_distance = scatter_distance,
            .x_frame_offset = frame_offset};
}

std::string RosInteractor::get_param(const std::string &param_name)
{
    std::string param;
    if (!ros::param::get(param_name, param))
    {
        throw skeleton_exceptions::LackingRosParameter(param_name);
    }
    return param;
}

void RosInteractor::create_listeners(const std::string &pose_topic_name, const std::string &pointcloud_topic_name)
{
    skeleton_subscriber_ = new message_filters::Subscriber<tfpose_ros::Persons>(
        node_handle_,
        pose_topic_name,
        INPUT_QUEUE_SIZE_);
    pointcloud_subscriber_ = new message_filters::Subscriber<PointCloud>(
        node_handle_,
        pointcloud_topic_name,
        INPUT_QUEUE_SIZE_);

    message_synchronizer_ = new message_filters::Synchronizer<MySyncPolicy>(
        MySyncPolicy(INPUT_QUEUE_SIZE_),
        *skeleton_subscriber_,
        *pointcloud_subscriber_);
    message_synchronizer_->registerCallback(
        boost::bind(&RosInteractor::generate_skeleton, this, _1, _2));
}

void RosInteractor::create_publisher(const std::string &skeleton_topic_name)
{
    skeleton3d_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>(
        skeleton_topic_name,
        50);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d");
    RosInteractor::RosParams parameters = RosInteractor::read_params();
    try
    {
        RosInteractor ros_interactor(parameters);
        ros::spin();
    } catch (skeleton_exceptions::LackingRosParameter &e)
    {
        ROS_ERROR("Missing Node Parameter %s", e.get_info().c_str());
        return 1;
    }
}