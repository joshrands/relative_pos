#include "ImageConverter.hpp"

ImageConverter::ImageConverter(ros::NodeHandle nh)
    : m_imageTransport(nh)
{
    // Subscrive to input video feed and publish output video feed
    this->m_isFrameReady = false;
}

void ImageConverter::start(std::string rosTopicName)
{
    ROS_WARN("Image converter not using threading.");

    this->m_topicName = rosTopicName;

    this->m_imageSub = m_imageTransport.subscribe(rosTopicName, 1,
        &ImageConverter::imageCallback, this);
}

ImageConverter::~ImageConverter()
{
}

void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        this->m_currentFrame = cv_ptr->image;
        this->m_isFrameReady = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

bool ImageConverter::isFrameReady()
{
    return this->m_isFrameReady;
}

cv::Mat ImageConverter::getCurrentFrame()
{
    return this->m_currentFrame;
}
