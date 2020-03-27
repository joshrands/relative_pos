/* track_relative_bots.cpp */
bool g_debug = true;

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ros/console.h>
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <relative_pos/ArucoRobot.h>

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector> 

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

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
        "{n        |1    | Number of robots }"
        "{l        |      | Actual aruco side length in meters }"
        ;
}

class Robot
{
public:
    Robot();
    Robot(relative_pos::ArucoRobot bot);

    float getArucoMarkerSideLength() { return m_info.marker_length_m.data; }
    std::string getName() { return this->m_name; }
    relative_pos::ArucoRobot getInfo() { return this->m_info; }

    void setName(std::string name) { this->m_name = name; }
    void setInfo(relative_pos::ArucoRobot info) { this->m_info = info; }

    void setTranslationVector_M(cv::Vec3d t_xyz) { this->m_t_xyz_m = t_xyz; }
    void setRotationVector_Deg(cv::Vec3d r_xyz) { this->m_r_xyz_deg = r_xyz; }

    static int getRobotIdFromArucoId(int arucoId) { return floor((arucoId-1)/4); }

private:
    relative_pos::ArucoRobot m_info;

    // translation and rotation vectors 
    cv::Vec3d m_t_xyz_m;
    cv::Vec3d m_r_xyz_deg;

    std::string m_name;
};

std::vector<Robot*> robots;

void arucoRobotCallback(const relative_pos::ArucoRobot::ConstPtr& msg)
{
    ROS_INFO("Updating ArucoRobot info...");

    // get the correct robot from the vector 
    relative_pos::ArucoRobot info = (relative_pos::ArucoRobot)(*msg);
    uint32_t id = info.id;
    ROS_INFO("Received info for robot%d", id); 

    Robot* bot = robots.at(id);
    bot->setInfo(info);
}

void createBots(int numberOfBots,float marker_length_m)
{
    for (int i = 0; i < numberOfBots; i++)
    {
        // create a new bot 
        std::string name = "robot" + patch::to_string(i);

        relative_pos::ArucoRobot botInfo;
        botInfo.id = i;
        int offset = i*4;
        botInfo.front_aruco_id = offset+1;
        botInfo.right_aruco_id = offset+4;
        botInfo.left_aruco_id = offset+2;
        botInfo.back_aruco_id = offset+3;
        std_msgs::Float32 length;
        length.data = marker_length_m; 
        botInfo.marker_length_m = length; 

        Robot* newBot = new Robot(botInfo);
        newBot->setName(name);
        robots.push_back(newBot);

        std::cout << name << " created." << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_relative_bots");

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // create n bots 
    float marker_length_m = parser.get<float>("l"); 
    int numberOfBots = parser.get<int>("n");
    createBots(numberOfBots,marker_length_m);

    // create an array for all the bot subscribers 
    ros::Subscriber bot_subs[numberOfBots] = {};

    for (int i = 0; i < numberOfBots; i++)
    {
        std::string name = robots.at(i)->getName();
        bot_subs[i] = n.subscribe<relative_pos::ArucoRobot>(name + "/info",1000,arucoRobotCallback);
    }

    ros::Rate loop_rate(30);

    // OPENCV STUF
    int dictionaryId = parser.get<int>("d");
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
        in_video.open("http://192.168.1.20:8000/stream.mjpg");
//        in_video.open(0);
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

    int count = 0;
    while (ros::ok() && in_video.grab())
    { 
        // update parameters from ros
        // WARNING: Currently all robots must have the same marker sizes.  
        marker_length_m = robots.at(0)->getArucoMarkerSideLength();

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

            for(int i=0; i < ids.size(); i++)
            {
                // TODO: Determine which robot was detected 
                int robotId = Robot::getRobotIdFromArucoId(ids.at(i));
                std::cout << "Detected aruco" << ids.at(i) << " robot" << robotId << std::endl;

                // TODO: Determine robot x,y,z based off of what side you are viewing 
                

                // output the robot statistics 
                static int count = 0;
                if (g_debug && count++ % 30 == 0) {
                    std::cout << "Robot" << i << std::endl; 
                    std::cout << "Translation: x: " << tvecs[i](0) 
                            << " y: " << tvecs[i](1)
                            << " z: " << tvecs[i](2) << std::endl;
                    std::cout << "Rotation: x: " << rvecs[i](0) 
                            << " y: " << rvecs[i](1)
                            << " z: " << rvecs[i](2) << std::endl;
                    cv::Mat rot_mat = cv::Mat::zeros(3,3,CV_64F);
                    cv::Rodrigues(rvecs[0],rot_mat);

                    std::cout << "Rotation Matrix:"<<std::endl;
                    for(int i=0; i<3; i++)
                    {
                        for(int j=0; j<3; j++)
                            std::cout<< rot_mat.at<double>(i,j) << " ";
                        std::cout << std::endl;
                    }
                    std::cout << std::endl;

                    cv::Mat mtxR = cv::Mat::zeros(3,3,CV_64F);
                    cv::Mat mtxQ = cv::Mat::zeros(3,3,CV_64F);

                    cv::Vec3d ypr = cv::RQDecomp3x3(rot_mat, mtxR, mtxQ);
                    std::cout << "x_rot: " << ypr(0) << "deg y_rot: " << ypr(1) << "deg z_rot: " << ypr(2) << "deg\n";
                }

                // Draw axis for each marker
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);

/*
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
*/ 
            }
        }

        imshow("Relative Robot Tracker", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;

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