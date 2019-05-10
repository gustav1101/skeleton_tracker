#include "StaticCloudFilter.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Point = pcl::PointXYZ;
template<class T> using optional = boost::optional<T>;

bool StaticCloudFilter::pass_filter(PointCloud &original_point_cloud)
{
    message_counter_++;
    if(message_counter_ <= number_of_messages_to_discard_)
    {
        ROS_INFO("Discarding message");
        return false;
    }
    if(!background_vectors_initialised_)
    {
        ROS_INFO("Creating Background Vector");
        initialise_background_vectors(
            original_point_cloud.width,
            original_point_cloud.height);
    }

    if(message_counter_ <= number_of_messages_to_discard_ + number_of_calibration_messages_)
    {
        ROS_INFO("Callibrating Filter");
        calibrate_filter(original_point_cloud);
        return false;
    }
    else
    {
        apply_filter(original_point_cloud);
        return true;
    }
}

void StaticCloudFilter::calibrate_filter(const PointCloud &original_point_cloud)
{
    only_null_values_ = true;
    for(unsigned int x = 0; x < original_point_cloud.width; x++)
    {
        for(unsigned int y=0; y < original_point_cloud.height; y++)
        {
            calibrate_depth_value_at(x, y, original_point_cloud.at(x,y));
        }
    }
    if (only_null_values_)
    {
        ROS_WARN("Only illegal points found during calibration!");
        message_counter_--;
    }
}

void StaticCloudFilter::calibrate_depth_value_at(const unsigned int &x_pos,
                                                 const unsigned int &y_pos,
                                                 const Point &point
)
{
    if (point_has_nan_values(point))
    {
        return;
    }
    only_null_values_ = false;
    double &stored_z_value = get_filter_depth_value(x_pos, y_pos);
    const double &new_z_value = point.z;
    update_filter_depth_value(stored_z_value, new_z_value);
}

void StaticCloudFilter::update_filter_depth_value(double &stored_value, const double &new_value)
{
    if(stored_value > new_value)
    {
        stored_value = new_value;
    }
}

void StaticCloudFilter::apply_filter(PointCloud &original_point_cloud)
{
    for(unsigned int x = 0; x < original_point_cloud.width; x++)
    {
        for(unsigned int y=0; y < original_point_cloud.height; y++)
        {
            apply_filter_at(x, y, original_point_cloud.at(x,y));
        }
    }
}

void StaticCloudFilter::apply_filter_at(const unsigned int &x_pos,
                                        const unsigned int &y_pos,
                                        Point &point)
{
    double &filter_z_value = get_filter_depth_value(x_pos, y_pos);
    if( point_should_be_masked(point, filter_z_value) )
    {
        mask_point(point);
    }
}

bool StaticCloudFilter::point_should_be_masked(const Point &point,
                                               const double &filter_z_value)
{
    return point.z <= filter_z_value;
}

void StaticCloudFilter::mask_point(Point &point)
{
    point.x = point.y = point.z = std::numeric_limits<double>::quiet_NaN();
}

void StaticCloudFilter::initialise_background_vectors(unsigned int width, unsigned int height)
{
    std::vector<double> sample_row_vector(width, 0.0);
    background_z_value_ = std::vector<std::vector<double>>(height, sample_row_vector);
    background_vectors_initialised_ = true;
}

bool StaticCloudFilter::point_has_nan_values(const Point &point)
{
    return (std::isnan(point.x) or std::isnan(point.y) or std::isnan(point.z));
}

double& StaticCloudFilter::get_filter_depth_value(const unsigned int &x, const unsigned int &y)
{
    return background_z_value_.at(y).at(x);
}
