#ifndef STATICCLOUDFILTER_HPP
#define STATICCLOUDFILTER_HPP

#include <pcl_ros/point_cloud.h>
#include <boost/optional.hpp>
#include "FilterStatus.hpp"

class StaticCloudFilter
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using Point = pcl::PointXYZ;
    template<class T> using optional = boost::optional<T>;

public:
    StaticCloudFilter(int number_of_messages_to_discard, int number_of_calibration_messages) :
        number_of_messages_to_discard_(number_of_messages_to_discard),
        number_of_calibration_messages_(number_of_calibration_messages),
        percentage_of_pixels_to_calibrate_(0.95),
        discarded_messages_counter_(0),
        calibration_prepared_(false),
        successive_unsuccessful_calibration_attempts_(0),
        max_number_of_calibration_pixels_(0),
        current_number_of_calibrated_pixels(0),
        current_filter_status_(pointcloud_filter_status::Status::calibrating)
    {};
    pointcloud_filter_status::Status pass_filter(PointCloud &original_point_cloud_i);

private:
    const unsigned int number_of_messages_to_discard_;
    const unsigned int number_of_calibration_messages_;
    const double percentage_of_pixels_to_calibrate_;
    std::vector<std::vector<double>> background_z_value_;
    unsigned int discarded_messages_counter_;
    bool calibration_prepared_;
    unsigned int successive_unsuccessful_calibration_attempts_;
    bool only_null_values_;
    unsigned int max_number_of_calibration_pixels_;
    unsigned int current_number_of_calibrated_pixels;
    pointcloud_filter_status::Status current_filter_status_;

    void make_sure_filter_is_calibrated(
        const PointCloud &observed_point_cloud);
    bool calibration_finished();
    bool sufficient_pixels_calibrated();
    double percentage_of_pixels_calibrated();
    void calibrate_filter(const PointCloud &observed_point_cloud);
    void calibrate_filter_matrix(const PointCloud &observed_point_cloud);
    void calibrate_filter_for_position(const unsigned int &x_pos,
                                  const unsigned int &y_pos,
                                  const Point &point);
    void update_filter_depth_value(double &stored_value, const double &new_value);
    void apply_filter(PointCloud &original_point_cloud);
    void apply_filter_at(const unsigned int &x_pos,
                         const unsigned int &y_pos,
                         Point &point);
    bool point_should_be_masked(const Point &point,
                                const double &filter_z_value);
    void mask_point(Point &point);
    void prepare_calibration(unsigned int cloud_width, unsigned int cloud_height);
    void set_max_number_of_calibration_pixels(unsigned int width, unsigned int height);
    void initialise_background_vectors(unsigned int width, unsigned int height);
    bool point_has_nan_values(const Point &point);
    double& get_filter_depth_value(const unsigned int &x, const unsigned int &y);
};

#endif
