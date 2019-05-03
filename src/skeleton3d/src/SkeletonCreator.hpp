#ifndef SKELETONCREATOR_HPP
#define SKELETONCREATOR_HPP

#include <geometry_msgs/Point.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/Persons.h>
#include <skeleton3d/Skeleton3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>
#include "PointFinder.hpp"

/**
 * Generates 3d Skeletons from 2d pose and pointcloud.
 *
 * This class tries to generate a 3d skeleton from a pose recognised by tf pose in 2d. For each body part coordinate in 2d the depth is retrieved from the pointcloud to create points in 3d space for each body part. The depth of the pixel coordinates of a pose is found by the PointFinder class.
 */
class SkeletonCreator
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

public:
    /**
     * Constructor.
     *
     * @param [in] scatter_distance Distance in pixels around which additional points will be found
     * @param [in] frame_offset X correction in pixels between the rgb image and point cloud coordinate system, positive means right
     */
    SkeletonCreator(int scatter_step_distance, int scatter_steps, double frame_offset) :
        point_finder_(scatter_step_distance, scatter_steps, frame_offset) {}
    
    ~SkeletonCreator();

    /**
     * Generate the 3d skeleton from the 2d data and the 3d point cloud.
     *
     * \note Both arguments should closely correspond in time to get accurate point coordinates.
     *
     * @param [in] persons Persons from tf pose Persons msg
     * @param [in] point_cloud Current point cloud for depth information retrieval
     */
    std::vector<skeleton3d::Skeleton3d> generate_skeletons(
        const std::vector<tfpose_ros::Person> &persons,
        const PointCloud &point_cloud);

    /**
     * Set the image size of pointcloud image in point finder.
     *
     * @param [in] width Width of the PointCloud image
     * @param [in] height Height of the PointCloud image
     */
    void set_image_size(unsigned int width, unsigned int height);

private:
    /**
     * Instance of point finder which converts 2D coordinates into 3D points.
     */
    PointFinder point_finder_;

    /**
     * Transform a single 2d skeleton into a 3d one.
     *
     * To do this, transform each individual 2d body part position into a 3d point.
     * 
     * Since some 2D coordinates of individual body parts might not have a valid corresponding point in the point cloud, and more importantly, since the transformation from 2d coordinates to a 3d point in the point cloud is not perfect, several points scattered around the center point also have their depth information checked. The "best" is then used and returned.
     *
     * @param [in] person One Person as per tf pose.
     * @return Skeleton if any body part 3d conversion was successful, else return boost::none
     */
    boost::optional<skeleton3d::Skeleton3d> transform_skeleton_to_3d(
        const tfpose_ros::Person &person);

    /**
     * Check if any coordinate is NAN.
     *
     * @return true iff any coordinate is NAN.
     */
    bool any_coordinate_invalid(float x, float y, float z);
};

#endif
