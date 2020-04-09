#include "ImageConverter.hpp"

static const std::string OPENCV_WINDOW = "Image window";

ImageConverter::ImageConverter(ros::NodeHandle nh)
    : it_(nh)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
}

ImageConverter::~ImageConverter()
{
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
//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
//    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//    cv::waitKey(3);

    // Output modified video stream
//    image_pub_.publish(cv_ptr->toImageMsg());
}