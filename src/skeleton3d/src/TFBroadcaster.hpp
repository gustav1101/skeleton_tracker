#ifndef TFBROADCASTER_HPP
#define TFBROADCASTER_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/**
 * Broadcasts static TF Frames.
 *
 * This class is intended to broadcast frame transforms relative to the global "world" frame.
 */
class TFBroadcaster
{
public:
    TFBroadcaster() {};
    /**
     * Broadcast all currently set frame transformations once.
     */
    void broadcast();

    /**
     * Add a frame to the list of frames to broadcast.
     *
     * @param frame_transformation Transformation from global to this frame.
     * @param name Frame name.
     */
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
