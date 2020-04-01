/* track_relative_bots.cpp */

bool g_debug = false;
#define PI 3.14159265

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ros/console.h>
#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <relative_pos/ArucoRobot.h>
#include <relative_pos/RobotPose.h>

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>
#include <math.h> 

// to_string didn't work for some reason 
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
        "{n        |1     | Number of robots }"
        "{l        |      | Actual aruco side length in meters }"
        "{p        |0     | Parent robot id}"
        ;
}

/* Robot class for storing information about a robot being tracked */
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

    void setRobotPose(geometry_msgs::Pose2D pose) { this->m_pose = pose; }
    geometry_msgs::Pose2D getRobotPose() { return this->m_pose; }
    geometry_msgs::Pose2D getRelativeRobotPoseFromArucoVectors(cv::Vec3d t_vec, cv::Vec3d r_vec, int markerId);

    /* DEPRECATED
    void setTranslationVector_M(cv::Vec3d t_xyz) { this->m_t_xyz_m = t_xyz; }
    void setRotationVector_Deg(cv::Vec3d r_xyz) { this->m_r_xyz_deg = r_xyz; }
    */

    static int getRobotIdFromArucoId(int arucoId) { return floor((arucoId-1)/4); }

    static int m_totalRobots;
    static int m_parentBot;

protected:
    relative_pos::ArucoRobot m_info;
    geometry_msgs::Pose2D m_pose;
    // translation and rotation vectors 
    cv::Vec3d m_t_xyz_m;
    cv::Vec3d m_r_xyz_deg;
    std::string m_name;
};

int Robot::m_totalRobots = 0;
int Robot::m_parentBot = 0;

std::vector<Robot*> robots;

// callback function for parent robot pose 
void parentBotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    geometry_msgs::Pose2D pose = (geometry_msgs::Pose2D)(*msg);

    ROS_INFO_STREAM("Reived pose update for parent robot " << Robot::m_parentBot);

    robots.at(Robot::m_parentBot)->setRobotPose(pose);
}

// callback function for setting parameters about a robot 
void arucoRobotCallback(const relative_pos::ArucoRobot::ConstPtr& msg)
{
    // get the correct robot from the vector 
    relative_pos::ArucoRobot info = (relative_pos::ArucoRobot)(*msg);
    uint32_t id = info.id;
    ROS_INFO("Received info for robot%d", id); 

    // make sure this robot exists with current parameters 
    if (id >= Robot::m_totalRobots)
        ROS_WARN("Invalid Robot ID detected.");

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

        ROS_INFO_STREAM(name << " created");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_relative_bots");

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    ros::NodeHandle n;

    // create n bots 
    float marker_length_m = parser.get<float>("l"); 
    int numberOfBots = parser.get<int>("n");
    if (0 == numberOfBots)
    {
        ROS_WARN("-n flag is 0, no robots can be detected.");
        return -1;
    }

    // create the robots 
    createBots(numberOfBots,marker_length_m);

    // create an array for all the bot subscribers 
    ros::Subscriber bot_subs[numberOfBots] = {};
    // create an array for all bot publishers 
    // the index of the parent node will be null
    ros::Publisher relative_positions[numberOfBots] = {};
    ros::Subscriber parentBotPose;

    // get parent bot id 
    Robot::m_parentBot = parser.get<int>("p");
    std::string parentBotName = "robot" + patch::to_string(Robot::m_parentBot);
    Robot* parentBot = robots.at(Robot::m_parentBot);

    for (int i = 0; i < numberOfBots; i++)
    {
        std::string name = robots.at(i)->getName();
        bot_subs[i] = n.subscribe<relative_pos::ArucoRobot>(name + "/info",1000,arucoRobotCallback);

        if (i != Robot::m_parentBot)
        {
            // create a publisher!
            relative_positions[i] = n.advertise<geometry_msgs::Pose2D>(parentBotName + "/" + name + "/relative_pose", 1000);
        }
        else 
            parentBotPose = n.subscribe<geometry_msgs::Pose2D>(parentBotName + "/pose",1000,parentBotPoseCallback);
    }

    ros::Rate loop_rate(30);

    // OPENCV STUF
    int dictionaryId = parser.get<int>("d");
    int wait_time = 10;

    if (marker_length_m <= 0) {
        ROS_WARN("marker length must be a positive value in meter");
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
                if (ids.at(i) > robots.size() * 4)
                {
                    ROS_WARN("ERROR: Invalid aruco id %d",ids.at(i));
                    continue;
                }

                // Determine which robot was detected 
                int robotId = Robot::getRobotIdFromArucoId(ids.at(i));
                ROS_INFO_STREAM("Robot" << Robot::m_parentBot << " detected Robot" << robotId);

                Robot* detectedBot = robots.at(robotId);

                // Determine robot x,y,theta based off of what side you are viewing 
                geometry_msgs::Pose2D detectedPose = parentBot->getRelativeRobotPoseFromArucoVectors(tvecs.at(i),rvecs.at(i),ids.at(i)); 
                relative_positions[robotId].publish(detectedPose);
                detectedBot->setRobotPose(detectedPose);

                // Draw axis for each marker
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);
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

geometry_msgs::Pose2D Robot::getRelativeRobotPoseFromArucoVectors(cv::Vec3d t_vec, cv::Vec3d r_vec, int markerId)
{
    float marker_length_m = this->getArucoMarkerSideLength();

    cv::Vec3d t_xyz = t_vec;
    if (g_debug)
        std::cout << "BEFORE TRANSFORMATION: " << t_xyz(0) << "," << t_xyz(1) << "," << t_xyz(2) << std::endl;

    // get rotation vector 
    cv::Mat rot_mat = cv::Mat::zeros(3,3,CV_64F);
    cv::Rodrigues(r_vec,rot_mat);

    // dumby matrices 
    cv::Mat mtxR = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat mtxQ = cv::Mat::zeros(3,3,CV_64F);

    cv::Vec3d r_xyz = cv::RQDecomp3x3(rot_mat, mtxR, mtxQ);
    if (g_debug)
        std::cout << "ROTATION: " << r_xyz(0) << "," << r_xyz(1) << "," << r_xyz(2) << std::endl;

    // modify z and x components (distance from camera) using the rotation around the y axis of the marker 
    double y_rotation_deg = r_xyz(1);
    t_xyz(2) += cos(y_rotation_deg * PI / 180.0) * marker_length_m / 2.0;
    t_xyz(0) += sin(y_rotation_deg * PI / 180.0) * marker_length_m / 2.0;

    if (g_debug)
        std::cout << "AFTER TRANSFORMATION: " << t_xyz(0) << "," << t_xyz(1) << "," << t_xyz(2) << std::endl;

    geometry_msgs::Pose2D parentPose = this->m_pose;

    // we are assuming the camera is facing forward (parallel with parent robot's heading)
    // z axis is straight out from robot 
    // +x is right, -x is left 
    geometry_msgs::Pose2D detectedPose;
    detectedPose.x = parentPose.x + t_xyz(0); 
    detectedPose.y = parentPose.y + t_xyz(2); 

    // get theta 
    // rotation around the aruco y axis gives relative 'z axis' rotation
    // but first we need to figure out what side we are viewing 
    int side = (markerId-1) % 4; 
    std::cout << "Side: " << side << std::endl;
    float relativeHeading = side * 90.0;
    relativeHeading += y_rotation_deg;
    std::cout << "Relative heading: " << relativeHeading << std::endl;

    detectedPose.theta = parentPose.theta + relativeHeading;

    return detectedPose;
}
