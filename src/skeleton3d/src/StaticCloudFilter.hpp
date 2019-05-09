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
    void apply_filter(PointCloud &original_point_cloud);
    void calibrate_depth_value_at(const Point &point,
                                  const unsigned int x_pos,
                                  const unsigned int y_pos);
    void apply_filter_at(Point &point);
    bool point_should_be_masked(const Point &point);
    void mask_point(Point &point);
    void initialise_background_vectors(unsigned int width, unsigned int height);
};

#endif
