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
        percentage_of_pixels_to_calibrate_(95.0),
        discarded_messages_counter_(0),
        calibration_prepared_(false),
        successive_unsuccessful_calibration_attempts_(0),
        MAX_SUCCESSIVE_UNSUCCESSFUL_CALIBRATION_ATTEMPTS_(20),
        max_number_of_calibration_pixels_(0),
        current_number_of_calibrated_pixels(0),
        current_filter_status_(pointcloud_filter_status::Status::calibrating),
        number_of_messages_used_for_calibration_(0)
    {};
    void calibrate_filter(const PointCloud::ConstPtr &observed_point_cloud);
    void pass_filter(PointCloud &original_point_cloud);
    pointcloud_filter_status::Status get_filter_status();

    /****TO BE REMOVED**/
    PointCloud& get_negative();

private:
    const unsigned int number_of_messages_to_discard_;
    const unsigned int number_of_calibration_messages_;
    const double percentage_of_pixels_to_calibrate_;
    std::vector<std::vector<double>> background_z_value_;
    std::vector<std::vector<std::vector<double>>> preliminary_background_z_value_;
    unsigned int discarded_messages_counter_;
    bool calibration_prepared_;
    unsigned int successive_unsuccessful_calibration_attempts_;
    const unsigned int MAX_SUCCESSIVE_UNSUCCESSFUL_CALIBRATION_ATTEMPTS_;
    unsigned int max_number_of_calibration_pixels_;
    unsigned int current_number_of_calibrated_pixels;
    pointcloud_filter_status::Status current_filter_status_;
    unsigned int number_of_messages_used_for_calibration_;
    double minimum_calibration_pixel_step_;

    /************* CALIBRATION *****************/
    bool message_should_be_discarded();
    void do_calibration_iteration(
        const PointCloud::ConstPtr &observed_point_cloud);
    void check_if_calibration_finished();
    bool calibration_finished();
    bool sufficient_pixels_calibrated();
    double percentage_of_pixels_calibrated();
    void check_progress_on_current_iteration(const unsigned int
                                             pixels_calibrated_before_current_iter);
    void finalise_calibration();
    void set_final_filter_values();
    double median_depth_value(std::vector<double> &depth_values);
    bool made_progress_in_current_calibration_iteration(const unsigned int
                                                        pixels_calibrated_before_current_iter);
    void calibrate_filter_matrix(const PointCloud::ConstPtr &observed_point_cloud);
    void calibrate_filter_for_position(const unsigned int &x_pos,
                                       const unsigned int &y_pos,
                                       const Point &point,
                                       bool &only_null_values_encountered);
    void update_filter_depth_value(std::vector<double> &stored_values, const double &new_value);
    std::vector<double>& get_preliminary_filter_values(const unsigned int &x,
                                                       const unsigned int &y);
    
    /*********** APPLYING FILTER **************/
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


    /*** REMOVE LATER ***/
    PointCloud negative_cloud;
    void create_negative(const PointCloud::ConstPtr original_cloud);
};

#endif
