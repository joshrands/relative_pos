/* ImageConverter class for bridging ROS and OpenCV images */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
public:
    ImageConverter(ros::NodeHandle nh);

    ~ImageConverter();

    void start(std::string rosTopicName);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat getCurrentFrame();
    bool isFrameReady();

protected:
    image_transport::ImageTransport m_imageTransport;
    image_transport::Subscriber m_imageSub;
    cv::Mat m_currentFrame;
    bool m_isFrameReady;
    std::string m_topicName;

};