#include "TFBroadcaster.hpp"

void TFBroadcaster::add_frame(tf::Transform frame_transformation, std::string name)
{
    frames_.push_back({.transform = frame_transformation, .name = name});
}

void TFBroadcaster::broadcast()
{
    for (const TFBroadcaster::Frame frame : frames_)
    {
        tf_broadcaster_.sendTransform(tf::StampedTransform(frame.transform , ros::Time::now(), "world", frame.name));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    TFBroadcaster tf_broadcaster;

    tf::Vector3 kinect2_transl(4.398, 0.253, 2.728);
    tf::Quaternion kinect2_quat(0.805, 0.374, -0.167, -0.430);
    tf::Transform kinect2_transform(kinect2_quat,kinect2_transl);
    tf_broadcaster.add_frame(kinect2_transform, "kinect2_link");

    tf::Vector3 xtion1_transl(4.669, 4.960, 2.740);
    tf::Quaternion xtion1_quat(-0.170, -0.233, 0.828, -0.481);
    tf::Transform xtion1_transform(xtion1_quat,xtion1_transl);
    tf_broadcaster.add_frame(xtion1_transform, "xtion1_link");

    tf::Vector3 xtion2_transl(0.400, 4.480, 2.630);
    tf::Quaternion xtion2_quat(0.133, 0.280, -0.409, 0.858);
    tf::Transform xtion2_transform(xtion2_quat,xtion2_transl);
    tf_broadcaster.add_frame(xtion2_transform, "xtion2_link");

    while(ros::ok())
    {
        ros::Duration(0.1).sleep();
        tf_broadcaster.broadcast();
    }
}
