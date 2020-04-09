/* ImageConverter class for bridging ROS and OpenCV images */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter(ros::NodeHandle nh);// : it_(nh); 

    ~ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};