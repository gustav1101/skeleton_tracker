#include "PointProjector.h"
#include <tfpose_ros/BodyPartElm.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeletons3d.h>



PointProjector::PointProjector()
{
    skeleton_subscriber_ = node_handle_.subscribe("/pose_estimator/pose", 10, &PointProjector::save_skeletons, this);
    pointcloud_subscriber_ = node_handle_.subscribe<PointCloud>("/myxtion/depth_registered/points", 10, &PointProjector::construct_3d_skeleton, this);
        //("/myxtion/depth_registered/points",10, &PointProjector::construct_3d_skeleton, this);
    skeleton3d_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>("skeleton_to_3d/skeletons", 50);
}

PointProjector::~PointProjector() {}

void PointProjector::construct_3d_skeleton(const PointCloud::ConstPtr& point_cloud)
{
    // make sure we have saved a skeleton before we start searching for points in 3d
    if (latest_persons_.persons.size() == 0 ) {
        return;
    }

    std::vector<skeleton3d::Skeleton3d> skeletons_3d;
    
    // For each skeleton created by openpose: transform into 3d skeleton
    for(int i=0; i < latest_persons_.persons.size(); i++)
    {
        boost::optional<skeleton3d::Skeleton3d> skeleton = transform_skeleton_to_3d(i, point_cloud);
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

    skeleton3d_publisher_.publish(skeletons_msg);
    
}

void PointProjector::save_skeletons(const tfpose_ros::Persons& persons_msg)
{
    latest_persons_ = tfpose_ros::Persons(persons_msg);
}

boost::optional<skeleton3d::Skeleton3d> PointProjector::transform_skeleton_to_3d(int i, const PointCloud::ConstPtr& point_cloud)
{
    skeleton3d::Skeleton3d skeleton;
    skeleton.body_parts = std::vector<skeleton3d::BodyPart3d>(18);

    // First set all body parts to invalid (meaning not found), then overwrite with
    // data from the actual tfpose body parts

    for(skeleton3d::BodyPart3d &body_part : skeleton.body_parts)
    {
        body_part.part_is_valid = false;
    }

    for(const tfpose_ros::BodyPartElm &body_part_2d : latest_persons_.persons.at(i).body_part)
    {
        skeleton3d::BodyPart3d &body_part_3d = skeleton.body_parts.at(body_part_2d.part_id);
        body_part_3d.part_is_valid = true;
        body_part_3d.part_id = body_part_2d.part_id;
        int pos_x = body_part_2d.x * latest_persons_.image_w;
        int pos_y = body_part_2d.y * latest_persons_.image_h;
        
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d");
    PointProjector point_projector;
    ros::spin();
}
