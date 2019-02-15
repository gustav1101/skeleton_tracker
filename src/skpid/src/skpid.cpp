#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tfpose_ros/Persons.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/BodyPartElm.h>
#include "skpid.h"

const std::string ImageConverter::OPENCV_WINDOW = "Image window";
std::vector<ImageConverter::BodyPart> ImageConverter::last_recognised_bodyparts = std::vector<ImageConverter::BodyPart>(0);

ImageConverter::ImageConverter() : it_(nh_)
{
    image_sub_ = it_.subscribe("/myxtion/rgb/image_raw", 1,
                               &ImageConverter::imageCb, this);

    sub = nh_.subscribe("/pose_estimator/pose", 1000, findBodyParts);
    cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

// Draw an example circle on the video stream
    bool bodyPartsRecognised = ImageConverter::last_recognised_bodyparts.size() > 0;
    
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60 && bodyPartsRecognised)
    {
        for (const BodyPart &bodyPart : ImageConverter::last_recognised_bodyparts)
        {
            int posInPic_x = bodyPart.pos_x * ImageConverter::RES_X;
            int posInPic_y = bodyPart.pos_y * ImageConverter::RES_Y;

            cv::circle(cv_ptr->image, cv::Point(posInPic_x, posInPic_y), 5, CV_RGB(0,0,255));
            cv::putText(cv_ptr->image, std::to_string(bodyPart.ID), cv::Point(posInPic_x + 8, posInPic_y + 8), 0, 0.7, CV_RGB(255,0,0));
        }
    }
        
// Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}

void ImageConverter::findBodyParts(const tfpose_ros::Persons& msg) {
    // This should never be NULL, should eventually check nevertheless
    tfpose_ros::Person person = msg.persons[0];
    std::vector<tfpose_ros::BodyPartElm> bodyPartsElm = person.body_part;
    ImageConverter::last_recognised_bodyparts.clear();
    ImageConverter::last_recognised_bodyparts.resize(bodyPartsElm.size());
    for (const tfpose_ros::BodyPartElm &bodyPartElm : bodyPartsElm ) {
        BodyPart bodyPart;
        bodyPart.ID = bodyPartElm.part_id;
        bodyPart.pos_x = bodyPartElm.x;
        bodyPart.pos_y = bodyPartElm.y;
        last_recognised_bodyparts.push_back(bodyPart);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
