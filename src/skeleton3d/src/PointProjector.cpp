#include "PointProjector.h"
#include <tfpose_ros/BodyPartElm.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeletons3d.h>

PointProjector::PointProjector()
{
    skeleton_subscriber_ = node_handle_.subscribe("/pose_estimator/pose", 10, &PointProjector::save_skeletons, this);
    pointcloud_subscriber_ = node_handle_.subscribe("/myxtion/depth_registered/points",10, &PointProjector::construct_3d_skeleton, this);
    skeleton3d_publisher_ = node_handle_.advertise<skeleton3d::Skeletons3d>("skeleton_to_3d/skeletons", 50);
}

PointProjector::~PointProjector() {}

void PointProjector::construct_3d_skeleton(const sensor_msgs::PointCloud2 point_cloud)
{
    // make sure we have saved a skeleton before we start searching for points in 3d
    if (latest_tf_skeletons_.size() == 0 ) {
        return;
    }

    std::vector<skeleton3d::Skeleton3d> skeletons_3d;
    
    // For each skeleton created by openpose: transform into 3d skeleton
    for(const tfpose_ros::Person &tf_person : latest_tf_skeletons_)
    {
        skeletons_3d.push_back(transform_skeleton_to_3d(tf_person, point_cloud));
    }
    skeleton3d::Skeletons3d skeletons_msg;
    skeletons_msg.skeletons = skeletons_3d;
    skeletons_msg.header.stamp = ros::Time::now();
    skeletons_msg.header.frame_id = "world";

    /*
    if (output_counter_++ >= 50 )
    {
        output_counter_ = 0;
        ROS_INFO("Created %zu Skeletons", skeletons_msg.skeletons.size());
        if (skeletons_msg.skeletons.size() > 0)
        {
            ROS_INFO("Skeleton 1 has a list of %zu body parts (should be 18)",
                     skeletons_msg.skeletons.at(0).body_parts.size());
            int recognised_parts = 0;
            for (skeleton3d::BodyPart3d &body_part : skeletons_msg.skeletons.at(0).body_parts)
            {
                if (body_part.part_is_valid)
                {
                    recognised_parts++;
                }
            }
            ROS_INFO("%i parts are valid.", recognised_parts);
            
        }
        
        }*/
    skeleton3d_publisher_.publish(skeletons_msg);
    
}

/**
   Function to convert 2D pixel point to 3D point by extracting point
   from PointCloud2 corresponding to input pixel coordinate. This function
   can be used to get the X,Y,Z coordinates of a feature using an 
   RGBD camera, e.g., Kinect.
*/
geometry_msgs::Point PointProjector::pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int x_pos, const int y_pos)
{
    // get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = y_pos*pCloud.row_step + x_pos*pCloud.point_step;

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    float x, y, z;
    x = y = z = 0.0;
    memcpy(&x, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&z, &pCloud.data[arrayPosZ], sizeof(float));

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    if (output_counter_++ >= 50)
    {
        ROS_INFO("Point: x=%f \ny=%f\nz=%f\n",x, y, z);
        output_counter_ = 0;
    }
    return point;
}

void PointProjector::save_skeletons(const tfpose_ros::Persons& persons_msg)
{
    latest_tf_skeletons_ = std::vector<tfpose_ros::Person>(persons_msg.persons);
}

skeleton3d::Skeleton3d PointProjector::transform_skeleton_to_3d(const tfpose_ros::Person tf_person, const sensor_msgs::PointCloud2 point_cloud)
{
    skeleton3d::Skeleton3d skeleton;
    skeleton.body_parts = std::vector<skeleton3d::BodyPart3d>(18);

    // First set all body parts to invalid (meaning not found), then overwrite with
    // data from the actual tfpose body parts

    for(skeleton3d::BodyPart3d &body_part : skeleton.body_parts)
    {
        body_part.part_is_valid = false;
    }

    for(const tfpose_ros::BodyPartElm &body_part_2d : tf_person.body_part)
    {
        skeleton3d::BodyPart3d &body_part_3d = skeleton.body_parts.at(body_part_2d.part_id);
        body_part_3d.part_is_valid = true;
        body_part_3d.part_id = body_part_2d.part_id;
        body_part_3d.point = pixel_to_3d_point(
            point_cloud,
            body_part_2d.x,
            body_part_2d.y);
        body_part_3d.confidence = body_part_2d.confidence;
    }

    return skeleton;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skeleton_to_3d");
    PointProjector point_projector;
    ros::spin();
}
