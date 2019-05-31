#include "StaticCloudFilter.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Point = pcl::PointXYZ;
template<class T> using optional = boost::optional<T>;

void StaticCloudFilter::calibrate_filter(const PointCloud::ConstPtr &observed_point_cloud)
{
    if (message_should_be_discarded())
    {
        return;
    }
    
    if(!calibration_prepared_)
    {
        prepare_calibration(observed_point_cloud->width,
                            observed_point_cloud->height);
    }

    try {
        do_calibration_iteration(observed_point_cloud);
    }
    catch (const pointcloud_filter_status::CalibrationAborted&)
    {
        ROS_WARN("Filter: Calibration ended prematurely with %f%% pixels calibrated",
                 percentage_of_pixels_calibrated());
        finalise_calibration();
    }
    if(calibration_finished())
    {
        create_negative(observed_point_cloud);
    }
}

void StaticCloudFilter::pass_filter(PointCloud &original_point_cloud)
{
    if (!calibration_finished())
    {
        throw pointcloud_filter_status::FilterNotReady();
    }
    apply_filter(original_point_cloud);
}

pointcloud_filter_status::Status StaticCloudFilter::get_filter_status()
{
    return current_filter_status_;
}

bool StaticCloudFilter::message_should_be_discarded()
{
    if(discarded_messages_counter_ <= number_of_messages_to_discard_)
    {
        discarded_messages_counter_++;
        ROS_INFO("Filter: Discarding first messages...");
        return true;
    }
    else
    {
        return false;
    }
}

void StaticCloudFilter::do_calibration_iteration(const PointCloud::ConstPtr
                                                 &observed_point_cloud)
{
    const unsigned int calibrated_pixels_before_iteration = current_number_of_calibrated_pixels;

    calibrate_filter_matrix(observed_point_cloud);

    check_progress_on_current_iteration(calibrated_pixels_before_iteration);
    check_if_calibration_finished();
}

bool StaticCloudFilter::calibration_finished()
{
    return current_filter_status_ == pointcloud_filter_status::Status::ready;
}

void StaticCloudFilter::finalise_calibration()
{
    set_final_filter_values();
    current_filter_status_ = pointcloud_filter_status::Status::ready;
}

void StaticCloudFilter::set_final_filter_values()
{
    for (unsigned int x = 0; x < preliminary_background_z_value_.size(); x++)
    {
        for(unsigned int y = 0; y < preliminary_background_z_value_.size(); y++)
        {
            background_z_value_.at(x).at(y) = median_depth_value(preliminary_background_z_value_
                                                     .at(x).at(y))*0.95;
        }
    }
}

double StaticCloudFilter::median_depth_value(std::vector<double> &depth_values)
{
    if(depth_values.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    size_t middle_index = depth_values.size() / 2;
    nth_element(depth_values.begin(), depth_values.begin()+middle_index, depth_values.end());
    return depth_values.at(middle_index);
}

void StaticCloudFilter::check_if_calibration_finished()
{
    if(number_of_messages_used_for_calibration_++ > number_of_calibration_messages_)
    {
        throw pointcloud_filter_status::CalibrationAborted();
    }
    if(sufficient_pixels_calibrated())
    {
        ROS_INFO("Filter Calibration successfully finished with %f%% of pixels calibrated",
                 percentage_of_pixels_calibrated());
        finalise_calibration();
    }
}

bool StaticCloudFilter::sufficient_pixels_calibrated()
{
    return percentage_of_pixels_calibrated() >= percentage_of_pixels_to_calibrate_;
}

double StaticCloudFilter::percentage_of_pixels_calibrated()
{
    return static_cast<double>(current_number_of_calibrated_pixels)
        / max_number_of_calibration_pixels_ * 100.0;
}

void StaticCloudFilter::check_progress_on_current_iteration(const unsigned int pixels_calibrated_before_current_iter)
{
    if (made_progress_in_current_calibration_iteration(pixels_calibrated_before_current_iter))
    {
        successive_unsuccessful_calibration_attempts_ = 0;
    }
    else
    {
        if (++successive_unsuccessful_calibration_attempts_ >=
            MAX_SUCCESSIVE_UNSUCCESSFUL_CALIBRATION_ATTEMPTS_)
        {
            throw pointcloud_filter_status::CalibrationAborted();
        }
    }
}

bool StaticCloudFilter::made_progress_in_current_calibration_iteration(const unsigned int pixels_calibrated_before_current_iter)
{
    return pixels_calibrated_before_current_iter + minimum_calibration_pixel_step_
        <= current_number_of_calibrated_pixels;
}

void StaticCloudFilter::calibrate_filter_matrix(const PointCloud::ConstPtr
                                                &original_point_cloud)
{
    bool only_null_values_encountered = true;
    for(unsigned int x = 0; x < original_point_cloud->width; x++)
    {
        for(unsigned int y=0; y < original_point_cloud->height; y++)
        {
            calibrate_filter_for_position(x, y,
                                          original_point_cloud->at(x,y),
                                          only_null_values_encountered);
        }
    }
    if (only_null_values_encountered)
    {
        ROS_WARN("Only illegal points found during calibration!");
    }
}

void StaticCloudFilter::calibrate_filter_for_position(const unsigned int &x_pos,
                                                      const unsigned int &y_pos,
                                                      const Point &point,
                                                      bool &only_null_values_encountered)
{
    if (point_has_nan_values(point))
    {
        return;
    }
    only_null_values_encountered = false;

    std::vector<double> &stored_z_value = get_preliminary_filter_values(x_pos, y_pos);
    const double &new_z_value = point.z;
    update_filter_depth_value(stored_z_value, new_z_value);
}

void StaticCloudFilter::update_filter_depth_value(std::vector<double> &stored_values,
                                                  const double &new_value)
{
    if (stored_values.empty())
    {
        current_number_of_calibrated_pixels++;   
    }
    stored_values.push_back(new_value);
}

std::vector<double>& StaticCloudFilter::get_preliminary_filter_values(const unsigned int &x, const unsigned int &y)
{
    return preliminary_background_z_value_.at(y).at(x);
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
    if(point_has_nan_values(point))
    {
        return;
    }
    double &filter_z_value = get_filter_depth_value(x_pos, y_pos);
    if( point_should_be_masked(point, filter_z_value) )
    {
        mask_point(point);
    }
}

bool StaticCloudFilter::point_should_be_masked(const Point &point,
                                               const double &filter_z_value)
{
    return (!std::isnan(filter_z_value) && point.z > filter_z_value);
}

void StaticCloudFilter::mask_point(Point &point)
{
    point.x = point.y = point.z = std::numeric_limits<double>::quiet_NaN();
}

void StaticCloudFilter::prepare_calibration(unsigned int cloud_width, unsigned int cloud_height)
{
    set_max_number_of_calibration_pixels(cloud_width, cloud_height);
    initialise_background_vectors(cloud_width, cloud_height);
}

void StaticCloudFilter::set_max_number_of_calibration_pixels(unsigned int width,
                                                             unsigned int height)
{
    max_number_of_calibration_pixels_ = width * height;
    minimum_calibration_pixel_step_ = max_number_of_calibration_pixels_ * 0.01;
}

void StaticCloudFilter::initialise_background_vectors(unsigned int width, unsigned int height)
{
    {
        std::vector<double> sample_row_vector(width, 0.0);
        background_z_value_ = std::vector<std::vector<double>>(height, sample_row_vector);
    }
    {
        std::vector<std::vector<double>> sample_row_vector(width); 
        preliminary_background_z_value_ = std::vector<std::vector<std::vector<double>>>(height,sample_row_vector);
    }
    calibration_prepared_ = true;
}

bool StaticCloudFilter::point_has_nan_values(const Point &point)
{
    return (std::isnan(point.x) or std::isnan(point.y) or std::isnan(point.z));
}

double& StaticCloudFilter::get_filter_depth_value(const unsigned int &x, const unsigned int &y)
{
    return background_z_value_.at(y).at(x);
}

void StaticCloudFilter::create_negative(const PointCloud::ConstPtr original_cloud)
{
    negative_cloud = *original_cloud;
    for(int x = 0; x < original_cloud->width; x++)
    {
        for(int y = 0; y < original_cloud->height; y++)
        {
            negative_cloud.at(x,y).z = background_z_value_.at(y).at(x);
        }
    }
}

PointCloud& StaticCloudFilter::get_negative()
{
    return negative_cloud;
}
