#ifndef TFBROADCASTER_HPP
#define TFBROADCASTER_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class TFBroadcaster
{
public:
    TFBroadcaster() {};
    void broadcast();
    void add_frame(tf::Transform frame_transformation, std::string name);
    
private:
    struct Frame
    {
        tf::Transform transform;
        std::string name;
    };
    std::vector<Frame> frames_;
    ros::NodeHandle node_handle_;
    tf::TransformBroadcaster tf_broadcaster_;
};

#endif
