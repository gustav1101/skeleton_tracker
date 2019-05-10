#ifndef STATICCLOUDFILTER_HPP
#define STATICCLOUDFILTER_HPP

#include <pcl_ros/point_cloud.h>
#include <boost/optional.hpp>

class StaticCloudFilter
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using Point = pcl::PointXYZ;
    template<class T> using optional = boost::optional<T>;

public:
    StaticCloudFilter(int number_of_messages_to_discard, int number_of_calibration_messages) :
        number_of_messages_to_discard_(number_of_messages_to_discard),
        number_of_calibration_messages_(number_of_calibration_messages),
        message_counter_(0),
        background_vectors_initialised_(false)
    {};
    bool pass_filter(PointCloud &original_point_cloud_i);

private:
    const unsigned int number_of_messages_to_discard_;
    const unsigned int number_of_calibration_messages_;
    std::vector<std::vector<double>> background_z_value_;
    unsigned int message_counter_;
    bool background_vectors_initialised_;
    bool only_null_values_;

    void calibrate_filter(const PointCloud &original_point_cloud);
    void calibrate_depth_value_at(const unsigned int &x_pos,
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
    void initialise_background_vectors(unsigned int width, unsigned int height);
    bool point_has_nan_values(const Point &point);
    double& get_filter_depth_value(const unsigned int &x, const unsigned int &y);
};

#endif
