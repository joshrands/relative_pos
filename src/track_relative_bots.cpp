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

#include "ImageConverter.hpp"

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>
#include <math.h> 

std::string regexBaseName = "tb3_";

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
        "{n        |3     | Number of robots }"
        "{l        |0.11  | Actual aruco side length in meters }"
        "{p        |1     | Parent robot id}"
        "{z        |0     | The z offset of the aruco marker cube}"
        "{i        |1     | The initial starting robot index}"
        "{f        |calibration_params.yml     | The camera calibration file}" 
        "{path     |<none>| The path to the calibration file}"
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

    static int getRobotIdFromArucoId(int arucoId) { return floor((arucoId)/10); }

    static int m_totalRobots;
    static int m_parentBot;
    static int m_startingIndex;

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
int Robot::m_startingIndex = 0;

std::vector<Robot*> robots;

// callback function for parent robot pose 
void parentBotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    geometry_msgs::Pose2D pose = (geometry_msgs::Pose2D)(*msg);

    ROS_INFO_STREAM("Reived pose update for parent robot " << Robot::m_parentBot);

    robots.at(Robot::m_parentBot-Robot::m_startingIndex)->setRobotPose(pose);
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

    Robot* bot = robots.at(id-Robot::m_startingIndex);
    bot->setInfo(info);
}

void createBots(int numberOfBots,float marker_length_m)
{
    for (int i = Robot::m_startingIndex; i < numberOfBots + Robot::m_startingIndex; i++)
    {
        // create a new bot 
        std::string name = regexBaseName + patch::to_string(i);

        relative_pos::ArucoRobot botInfo;
        botInfo.id = i;
        int offset = i*10;
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

    Robot::m_startingIndex = parser.get<int>("i");

    ros::NodeHandle n;

    // create an image converter class in case we are using a ros topic
    ImageConverter imageConverter(n);

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
    std::string parentBotName = regexBaseName + patch::to_string(Robot::m_parentBot);
    Robot* parentBot = robots.at(Robot::m_parentBot-Robot::m_startingIndex);

    for (int i = 0; i < numberOfBots; i++)
    {
        std::string name = robots.at(i)->getName();
        bot_subs[i] = n.subscribe<relative_pos::ArucoRobot>(name + "/info",1000,arucoRobotCallback);

        if ((i+Robot::m_startingIndex) != Robot::m_parentBot)
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
    bool usingRosTopicImage = false;

    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) 
        {
            parser.printMessage();
            return 1;
        }
        char* end;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        // TODO: Check if v == ros
        std::string rosCheck = videoInput.substr(0,1);
        std::cout << "STRING START: " << rosCheck << std::endl;
        if (rosCheck == "/")
        {
            // a ros topic was passed in as the video input 
            ROS_INFO_STREAM("Using rostopic: " << videoInput);
            usingRosTopicImage = true;
            // video input must contain a valid rostopic name 
            imageConverter.start(videoInput);
        }
        else if (!end || end == videoInput.c_str())
        {
            // we are assuming this is some kind of web server...
            ROS_INFO_STREAM("Using live stream: " << videoInput);
            in_video.open(videoInput); 
        } 
        else 
        {
            // use a webcam 
            in_video.open(source); 
        }
    } else {
        in_video.open(0);
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if (!in_video.isOpened() && false == usingRosTopicImage) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    std::string fileName = parser.get<cv::String>("f");
    ROS_INFO_STREAM("Opening calibration file: " << fileName << std::endl;);

    if (parser.has("path")) {
        std::string filePath = parser.get<cv::String>("path");
        fileName = filePath + "/" + fileName;
    }

    ROS_INFO_STREAM("Opening calibration file: " << fileName << std::endl;);
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    ROS_INFO_STREAM("camera_matrix\n" << camera_matrix << std::endl);
    ROS_INFO_STREAM("\ndist coeffs\n" << dist_coeffs << std::endl);

    int count = 0;
    while (ros::ok() && (in_video.grab() || true == usingRosTopicImage))
    { 
        // update parameters from ros
        // WARNING: Currently all robots must have the same marker sizes.  
        marker_length_m = robots.at(0)->getArucoMarkerSideLength();

        if (true == usingRosTopicImage) 
        {
            bool warning = false;
            while (false == imageConverter.isFrameReady())
            {
                if (warning == false)
                {
                    ROS_WARN("ROS topic frame not ready.");
                    warning = true;
                }
                ros::spinOnce();
            }

            image = imageConverter.getCurrentFrame();
        }
        else 
        {
            in_video.retrieve(image);
        }

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
                if (ids.at(i) < Robot::m_startingIndex*10+1 
                || ids.at(i) > (robots.size()+Robot::m_startingIndex) * 10)
                {
                    ROS_WARN("ERROR: Invalid aruco id %d",ids.at(i));
                    continue;
                }

                // Determine which robot was detected 
                int robotId = Robot::getRobotIdFromArucoId(ids.at(i));
                ROS_INFO_STREAM("Robot" << Robot::m_parentBot << " detected Robot" << robotId);

                Robot* detectedBot = robots.at(robotId-Robot::m_startingIndex);

                // Determine robot x,y,theta based off of what side you are viewing 
                geometry_msgs::Pose2D detectedPose = parentBot->getRelativeRobotPoseFromArucoVectors(tvecs.at(i),rvecs.at(i),ids.at(i)); 
                relative_positions[robotId-Robot::m_startingIndex].publish(detectedPose);
                detectedBot->setRobotPose(detectedPose);

                // Draw axis for each marker
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);
            }
        }

        imshow("Relative Robot Tracker for Robot " + patch::to_string(Robot::m_parentBot), image_copy);
        char key = (char)cv::waitKey(1);
        if (key == 27)
            break;

        ros::spinOnce();

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

    std::cout << "AFTER TRANSFORMATION: " << t_xyz(0) << "," << t_xyz(1) << "," << t_xyz(2) << std::endl;

    geometry_msgs::Pose2D parentPose = this->m_pose;

    // we need to modify z to not consider vertical. 
    t_xyz(2) = sqrt(pow(t_xyz(2),2) - pow(t_xyz(1),2));

    // we are assuming the camera is facing forward (parallel with parent robot's heading)
    // z axis is straight out from robot 
    // +x is right, -x is left 
    geometry_msgs::Pose2D detectedPose;
//    std::cout << parentPose.theta << std::endl;
//    detectedPose.x = parentPose.x + cos(parentPose.theta*PI / 180.0)*t_xyz(0) + sin(parentPose.theta*PI / 180.0)*t_xyz(2); 
//    detectedPose.y = parentPose.y - sin(parentPose.theta*PI / 180.0)*t_xyz(0) + cos(parentPose.theta*PI / 180.0)*t_xyz(2); 
    detectedPose.x = t_xyz(0);
    detectedPose.y = t_xyz(2);

    // get theta 
    // rotation around the aruco y axis gives relative 'z axis' rotation
    // but first we need to figure out what side we are viewing 
    int side = (markerId-1) % 10; 
    std::cout << "Side: " << side << std::endl;
    // add two so same heading = 0 degrees 
    float relativeHeading = ((side+2)%4) * 90.0;
    relativeHeading += y_rotation_deg;
    std::cout << "Relative heading: " << relativeHeading << std::endl;

    if (relativeHeading < 0)
        relativeHeading += 360;

    // flip to be ccw :)
    if (relativeHeading < 180)
        detectedPose.theta = -1*relativeHeading;
    else 
        detectedPose.theta = 360 - relativeHeading;
//    detectedPose.theta = relativeHeading;// (int(1000*(parentPose.theta - 180.0 + relativeHeading + 360.0)) % (360*1000))/1000.0;

    return detectedPose;
}
