/* track_relative_bots.cpp */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ros/console.h>
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <relative_pos/ArucoRobot.h>

#include <iostream>
#include <cstdlib>

namespace {
const char* about = "Detect ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        ;
}

class Robot
{
public:
    Robot();
    Robot(relative_pos::ArucoRobot bot);

    float getArucoMarkerSideLength() { return m_info.marker_length_m.data; }

    void setInfo(relative_pos::ArucoRobot info) { this->m_info = info; }

private:
    relative_pos::ArucoRobot m_info;
};

Robot bot; 

void arucoRobotCallback(const relative_pos::ArucoRobot::ConstPtr& msg)
{
    ROS_INFO("Updating ArucoRobot info...");

    relative_pos::ArucoRobot info;
    info.id = msg->id;
    info.front_aruco_id = msg->front_aruco_id;
    info.left_aruco_id = msg->left_aruco_id;
    info.back_aruco_id = msg->back_aruco_id;
    info.right_aruco_id = msg->right_aruco_id;
    std_msgs::Float32 length;
    length.data = msg->marker_length_m.data;
    info.marker_length_m = length;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_relative_bots");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    // set up bot aruco config and subscribe to topic 
    ros::Subscriber bot_sub = n.subscribe<relative_pos::ArucoRobot>("robot1/info",1000,arucoRobotCallback);

    relative_pos::ArucoRobot botInfo;
    botInfo.id = 1;
    botInfo.front_aruco_id = 1;
    botInfo.right_aruco_id = 4;
    botInfo.left_aruco_id = 2;
    botInfo.back_aruco_id = 3;
    std_msgs::Float32 length;
    length.data = 0.11; 
    botInfo.marker_length_m = length; 

    bot.setInfo(botInfo);

    // OPENCV STUF
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    int dictionaryId = parser.get<int>("d");
    float marker_length_m = bot.getArucoMarkerSideLength(); 
    int wait_time = 10;

    if (marker_length_m <= 0) {
        std::cerr << "marker length must be a positive value in meter" 
                  << std::endl;
        return 1;
    }

    cv::String videoInput = "0";
    cv::VideoCapture in_video;
    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
            parser.printMessage();
            return 1;
        }
        char* end;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        if (!end || end == videoInput.c_str()) {
            in_video.open(videoInput); // url
        } else {
            in_video.open(source); // id
        }
    } else {
        in_video.open(0);
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

    // TODO: Make sure camera matrix and dist coeffs were successful
    ROS_DEBUG("Hello world!");

    int count = 0;
    while (ros::ok() && in_video.grab())
    {
        // update parameters from ros 
        marker_length_m = bot.getArucoMarkerSideLength();

        in_video.retrieve(image);
        image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs, tvecs);
            
            // Draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);

                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x: " << std::setw(8) << tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);
            }
        }

        imshow("Pose estimation", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    in_video.release();

    return 0;
}

Robot::Robot()
{
    std::cout << "[WARNING]: Initializing robot with no info." << std::endl;
}

Robot::Robot(relative_pos::ArucoRobot bot)
{
    this->m_info = bot;
}