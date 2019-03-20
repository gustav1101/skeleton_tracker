#ifndef POINTFINDER_HPP
#define POINTFINDER_HPP

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>

/**
 * Create 3d point from a corresponding 2d rgb image.
 *
 * Use a pointcloud created at roughly the same time and position as the rgb image to get the
 * missing z coordinate. Depth information is retrieved by checking z coordinate in the
 * pointcloud (the missing "detph" information) at the coordinates specified.
 */
class PointFinder
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using Point3d = geometry_msgs::Point;
    
public:
    /**
     * Constructor.
     *
     * @param[in] scatter_distance The distance in pixels around which a point is probed for
     *            better depth information retrieval. See
     *            PointFinder::find_best_point_around_coordinates() for more information.
     * @param[in] frame_offset Frame offset in pointcloud coordinates to correct horizontal
     *            shift between RGB image and pointcloud. Values > 0 move the pointcloud
     *            coordinates to the right.
     */
    PointFinder(
        int scatter_distance, double frame_offset) :
        SCATTER_DISTANCE_(scatter_distance), frame_offset_(frame_offset){ };
    ~PointFinder() { };

    /**
     * Set size of window. Necessary before any point can be found.
     *
     * @param[in] image_max_x Maximum width of the pointcloud coordinate system
     * @param[in] image_max_y Maximum height of the pointcloud coordinate system 
     */
    void set_window_boundaries(unsigned int image_max_x, unsigned int image_max_y);

    /**
     * Set current point cloud. Use this to update the PointFinder with current pointcloud for
     * accurate depth information retrieval.
     *
     * @param[in] point_cloud The current pointcloud.
     */
    void set_point_cloud(const PointCloud::ConstPtr &point_cloud);

    /**
     * Find the "best" point around the center point coordinates.
     *
     * For any one point in 2d the query of depth information might fail, either because the
     * pointcloud holds invalid points at those coordinates (NAN) or because of imperfect
     * transformation between the 2d coordinate system of tf pose and the 3d coordinate system
     * of the point cloud. A slightly off value might result in drastically different depth
     * information, for example if the wall behind a person is queried.
     *
     * To work around this problem, several points around the actual coordinates are queried as
     * well. If any of those additional points is significantly better than the center point
     * that point is returned instead.
     *
     * "Best" currently means closest to the camera. This is because there is always something
     * behind the person, but rarely something very close and in front. Chosing the point like
     * this solves problems of minimal coordinate deviations causing the query of the wall
     * behind the person instead of the person itself.
     *
     * @param[in] relative_x x coordinate in relation to the picture width. 0=left border
     *                         of pointcloud, 1=right border of pointcloud.
     * @param[in] relative_y y coordinate in relation to the picture width. 0=upper border
     *                         of pointcloud, 1=lower border of pointcloud.
     * @return Best Point in 3d around specified coordinates if any such point exists, none else.
     */
    boost::optional<Point3d> find_best_point_around_coordinates(
        float relative_x, float relative_y);

private:
    struct Point2d {
        unsigned int x;
        unsigned int y;
    };

    /** Largest valid x coordinate value */
    unsigned int image_max_x_;
    /** Largest valid y coordinate value */
    unsigned int image_max_y_;
    PointCloud::ConstPtr point_cloud_;
    const int SCATTER_DISTANCE_;
    const double frame_offset_;
    /** A given point has to be this many meters closer to the camera than
     * the original point to be consiedered "better". */
    const double DEPTH_TOLERANCE_ = 0.01;

    /**
     * Create absolute coordinates from coordinates relative to the pointcloud width and height.
     *
     * @param relative_x x coordinate relative to the pointcloud width.
     * @param relative_y y coordinate relative to the pointcloud height.
     * @return The 2d point in absolute coordinates in the pointcloud coordinate system.
     */
    Point2d absolute_coordinates(float &relative_x, float &relative_y);

    /**
     * Create the scattered points around the center point.
     *
     * Only valid points will be added to the returned vector, so any points that have NAN
     * values in their point cloud coordinates will not be added.
     *
     * \note A point with identical coordinates to the center_point is included in the returned
     *       vector.
     *
     * @param center_point The point around which to create the scattered points.
     * @return Up to 8 points arranged in a rectangular pattern around the center_point with
     *         a copy of the center_point in the middle.
     */
    std::vector<Point3d> create_possible_points(Point2d &center_point);

    /**
     * Convert 2d point to a 3d Point.
     *
     * @param point2d Point in 2d Space.
     * @return Point in 3d Space if the coordinates in the pointcloud hold a valid point,
               boost::none otherwise.
     */
    boost::optional<Point3d> create_point3d(Point2d &point2d);

    /**
     * From all given points, find the best suited one and return it.
     *
     * This method only evaluates the given points for their value as skeleton body part point.
     * For details on the rough idea see find_best_point_around_coordinates().
     *
     * The center point is slightly preferred over other points as a return value. Any other
     * points have to be DEPTH_TOLERANCE_ meters closer to the camera to be selected. This is
     * because usually the center point has the most accurate x and y coordinates.
     *
     * @param possible_points All points to consider, can include center point.
     * @param center_point Center point, 
     *
     * @return A copy of the best point found.
     */
    Point3d find_best_point(const std::vector<Point3d> &possible_points,
                            const Point3d &center_point);

    /**
     * Check if any coordinate value is NAN.
     */
    inline bool any_coordinate_invalid(float x, float y, float z);
};

#endif
